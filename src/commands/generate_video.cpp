// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/generate_video.hpp"

#include "bagwiz/core/image/compressed_image.hpp"
#include "bagwiz/core/image/image_decoder.hpp"
#include "bagwiz/core/image/raw_image.hpp"
#include "bagwiz/core/logging.hpp"
#include "bagwiz/core/output_path.hpp"
#include "bagwiz/core/video/frame_rate.hpp"
#include "bagwiz/core/video/video_encoder.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <cinttypes>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <utility>

namespace bagwiz::commands
{

namespace
{
constexpr const char * kLogger = "bagwiz.cmd.generate";
constexpr const char * kImageType = "sensor_msgs/msg/Image";
constexpr const char * kCompressedImageType = "sensor_msgs/msg/CompressedImage";

bool is_supported_type(const std::string & type)
{
  return type == kImageType || type == kCompressedImageType;
}

// Timestamps + count for a single topic, gathered by a payload-free scan.
struct TopicSpan
{
  std::int64_t first_ns = 0;
  std::int64_t last_ns = 0;
  std::uint64_t count = 0;
};

}  // namespace

VideoSourceCheck check_video_source(const std::filesystem::path & input, const std::string & topic)
{
  VideoSourceCheck check;

  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input);
  } catch (const std::exception & e) {
    check.status = VideoSourceStatus::kInputUnopenable;
    check.message = "failed to open '" + input.string() + "': " + e.what();
    return check;
  }

  const io::TopicInfo * found = nullptr;
  for (const auto & t : reader->topics()) {
    if (t.name == topic) {
      found = &t;
      break;
    }
  }
  if (found == nullptr) {
    check.status = VideoSourceStatus::kTopicNotFound;
    check.message = "topic '" + topic + "' not found in " + input.string();
    return check;
  }

  check.topic_type = found->type;
  if (!is_supported_type(found->type)) {
    check.status = VideoSourceStatus::kUnsupportedType;
    check.message = "topic '" + topic + "' has type '" + found->type +
                    "', which generate video cannot render; supported types are " + kImageType +
                    " and " + kCompressedImageType;
    return check;
  }

  check.status = VideoSourceStatus::kOk;
  return check;
}

namespace
{

// Pass 1: stream the topic's messages reading only their timestamps (no
// payload decode) to learn the count and time span for the frame-rate estimate.
int scan_topic_span(const std::filesystem::path & input, const std::string & topic, TopicSpan & out)
{
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "failed to open '%s': %s", input.string().c_str(), e.what());
    return 1;
  }
  io::ReadFilter filter;
  filter.topics.push_back(topic);
  reader->set_filter(filter);

  io::RawMessage raw;
  try {
    while (reader->next(raw)) {
      if (out.count == 0) {
        out.first_ns = raw.timestamp_ns;
      }
      out.last_ns = raw.timestamp_ns;
      ++out.count;
    }
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "error reading topic '%s': %s", topic.c_str(), e.what());
    return 1;
  }
  return 0;
}

}  // namespace

int run_generate_video(const GenerateVideoArgs & args)
{
  // 1. Validate the source topic and type before touching the output.
  const auto check = check_video_source(args.input_path, args.topic);
  if (!check.ok()) {
    BAGWIZ_LOG_ERROR(kLogger, "%s", check.message.c_str());
    return 1;
  }

  // 2. Fail fast on an output collision before the expensive encode. The actual
  //    removal happens just before the rename, so an existing file is only
  //    replaced once the new video is fully written.
  {
    std::error_code ec;
    if (std::filesystem::exists(args.output_path, ec) && !args.overwrite) {
      BAGWIZ_LOG_ERROR(
        kLogger, "output '%s' already exists; pass -w/--overwrite to replace it.",
        args.output_path.string().c_str());
      return 1;
    }
  }

  // 3. Create the output's parent directory if needed.
  if (const auto parent = args.output_path.parent_path(); !parent.empty()) {
    std::error_code ec;
    std::filesystem::create_directories(parent, ec);
    if (ec) {
      BAGWIZ_LOG_ERROR(
        kLogger, "could not create output directory '%s': %s", parent.string().c_str(),
        ec.message().c_str());
      return 1;
    }
  }

  // 4. Pass 1: derive the frame rate from the message timestamps.
  TopicSpan span;
  if (scan_topic_span(args.input_path, args.topic, span) != 0) {
    return 1;
  }
  if (span.count == 0) {
    BAGWIZ_LOG_ERROR(kLogger, "topic '%s' has no messages to render.", args.topic.c_str());
    return 1;
  }
  const auto fps = core::video::derive_frame_rate(span.first_ns, span.last_ns, span.count);

  // 5. Pass 2: decode + encode to a sibling temp path, renamed on success and
  //    removed on any failure (no partial output, no leftover temp).
  // Keep the real extension on the temp file: both the encoder's codec choice
  // and the libav muxer are selected from the extension, so a bare
  // ".bagwiz-partial" suffix would be rejected. e.g. out.avi -> out.bagwiz-partial.avi
  const std::filesystem::path tmp_path =
    args.output_path.parent_path() /
    (args.output_path.stem().string() + ".bagwiz-partial" + args.output_path.extension().string());
  auto cleanup_tmp = [&tmp_path]() {
    std::error_code ec;
    std::filesystem::remove(tmp_path, ec);
  };
  cleanup_tmp();  // clear any stale temp from a previous aborted run

  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(args.input_path);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(
      kLogger, "failed to open '%s': %s", args.input_path.string().c_str(), e.what());
    return 1;
  }
  io::ReadFilter filter;
  filter.topics.push_back(args.topic);
  reader->set_filter(filter);

  // A CompressedImage topic carries a JPEG/PNG bitstream per message; decode it
  // to packed BGR before encoding. A raw Image topic is fed straight through.
  const bool is_compressed = (check.topic_type == kCompressedImageType);

  std::unique_ptr<core::video::VideoEncoder> encoder;
  std::uint32_t enc_w = 0;
  std::uint32_t enc_h = 0;
  std::string enc_encoding;
  std::uint64_t written = 0;

  io::RawMessage raw;
  core::image::DecodedImage decoded;  // reused storage for the compressed path
  try {
    while (reader->next(raw)) {
      // Normalize either message type to a packed 8-bit frame the encoder
      // accepts: width, height, row stride, pixel layout, and the pixel bytes.
      std::uint32_t fw = 0;
      std::uint32_t fh = 0;
      std::uint32_t fstep = 0;
      auto fsrc = core::video::SourcePixelFormat::kBgr8;
      std::string fenc;
      std::span<const std::byte> fdata;

      if (is_compressed) {
        const auto cimg = core::image::extract_compressed_image(raw.payload);
        if (!cimg.ok()) {
          BAGWIZ_LOG_ERROR(kLogger, "frame %" PRIu64 ": %s", written, cimg.error.c_str());
          encoder.reset();
          cleanup_tmp();
          return 1;
        }
        auto dec = core::image::decode_compressed_image(cimg.image->data, cimg.image->format);
        if (!dec.ok()) {
          BAGWIZ_LOG_ERROR(kLogger, "frame %" PRIu64 ": %s", written, dec.error.c_str());
          encoder.reset();
          cleanup_tmp();
          return 1;
        }
        decoded = std::move(*dec.image);
        fw = decoded.width;
        fh = decoded.height;
        fstep = decoded.width * 3U;
        fsrc = core::video::SourcePixelFormat::kBgr8;  // the decoder always emits BGR24
        fenc = "bgr8";
        fdata = {decoded.bgr.data(), decoded.bgr.size()};
      } else {
        const auto img = core::image::extract_raw_image(raw.payload);
        if (!img.ok()) {
          BAGWIZ_LOG_ERROR(kLogger, "frame %" PRIu64 ": %s", written, img.error.c_str());
          encoder.reset();
          cleanup_tmp();
          return 1;
        }
        const auto & v = *img.image;
        if (v.encoding == "bgr8") {
          fsrc = core::video::SourcePixelFormat::kBgr8;
        } else if (v.encoding == "rgb8") {
          fsrc = core::video::SourcePixelFormat::kRgb8;
        } else {
          BAGWIZ_LOG_ERROR(
            kLogger, "image encoding '%s' is not supported in this release; only bgr8 and rgb8.",
            v.encoding.c_str());
          encoder.reset();
          cleanup_tmp();
          return 1;
        }
        fw = v.width;
        fh = v.height;
        fstep = v.step;
        fenc = v.encoding;
        fdata = v.data;
      }

      if (encoder == nullptr) {
        // The first frame fixes the geometry and pixel encoding for the run.
        enc_w = fw;
        enc_h = fh;
        enc_encoding = fenc;
        auto opened = core::video::open_video_encoder(tmp_path, enc_w, enc_h, fps.num, fps.den);
        if (!opened.ok()) {
          BAGWIZ_LOG_ERROR(kLogger, "%s", opened.error.c_str());
          cleanup_tmp();
          return 1;
        }
        encoder = std::move(opened.encoder);
      } else if (fw != enc_w || fh != enc_h || fenc != enc_encoding) {
        BAGWIZ_LOG_ERROR(
          kLogger,
          "frame %" PRIu64 " changed to %ux%u %s from the first frame's %ux%u %s; aborting.",
          written, fw, fh, fenc.c_str(), enc_w, enc_h, enc_encoding.c_str());
        encoder.reset();
        cleanup_tmp();
        return 1;
      }

      if (auto e = encoder->write_frame(fdata, fstep, fsrc); !e.empty()) {
        BAGWIZ_LOG_ERROR(kLogger, "frame %" PRIu64 ": %s", written, e.c_str());
        encoder.reset();
        cleanup_tmp();
        return 1;
      }
      ++written;
    }
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "error reading topic '%s': %s", args.topic.c_str(), e.what());
    encoder.reset();
    cleanup_tmp();
    return 1;
  }

  // Pass 1 saw messages, but if pass 2 yielded none (e.g. the bag changed
  // between passes) the encoder was never created. Nothing was rendered.
  if (encoder == nullptr) {
    BAGWIZ_LOG_ERROR(
      kLogger, "topic '%s' yielded no frames in the encode pass.", args.topic.c_str());
    cleanup_tmp();
    return 1;
  }
  if (auto e = encoder->finish(); !e.empty()) {
    BAGWIZ_LOG_ERROR(kLogger, "%s", e.c_str());
    encoder.reset();
    cleanup_tmp();
    return 1;
  }
  encoder.reset();  // close the temp file before the rename/clobber

  // 6. Now that the new video is complete, replace any existing output and move
  //    the temp into place.
  if (const auto r = core::prepare_output_path(args.output_path, args.overwrite); !r.ok) {
    BAGWIZ_LOG_ERROR(kLogger, "%s", r.error.c_str());
    cleanup_tmp();
    return 1;
  }
  {
    std::error_code ec;
    std::filesystem::rename(tmp_path, args.output_path, ec);
    if (ec) {
      // Fall back to copy + remove across filesystems.
      std::error_code copy_ec;
      std::filesystem::copy_file(
        tmp_path, args.output_path, std::filesystem::copy_options::overwrite_existing, copy_ec);
      cleanup_tmp();
      if (copy_ec) {
        BAGWIZ_LOG_ERROR(
          kLogger, "could not move output into place: %s", copy_ec.message().c_str());
        return 1;
      }
    }
  }

  const double fps_value = static_cast<double>(fps.num) / static_cast<double>(fps.den);
  BAGWIZ_LOG_INFO(
    kLogger, "generate video: wrote %" PRIu64 " frame(s) to %s (%ux%u %s @ %.3g fps).", written,
    args.output_path.string().c_str(), enc_w, enc_h, enc_encoding.c_str(), fps_value);
  return 0;
}

}  // namespace bagwiz::commands
