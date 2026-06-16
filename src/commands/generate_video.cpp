// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/generate_video.hpp"

#include "bagwiz/commands/generate_video_overlay.hpp"
#include "bagwiz/core/camera/camera_info.hpp"
#include "bagwiz/core/image/compressed_image.hpp"
#include "bagwiz/core/image/image_decoder.hpp"
#include "bagwiz/core/image/raw_image.hpp"
#include "bagwiz/core/logging.hpp"
#include "bagwiz/core/output_path.hpp"
#include "bagwiz/core/tf_static_loader.hpp"
#include "bagwiz/core/video/frame_rate.hpp"
#include "bagwiz/core/video/video_encoder.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <tf2/buffer_core.hpp>

#include <cinttypes>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{
constexpr const char * kLogger = "bagwiz.cmd.generate";
constexpr const char * kImageType = "sensor_msgs/msg/Image";
constexpr const char * kCompressedImageType = "sensor_msgs/msg/CompressedImage";
constexpr const char * kPointCloud2Type = "sensor_msgs/msg/PointCloud2";
constexpr const char * kCameraInfoType = "sensor_msgs/msg/CameraInfo";

bool is_supported_type(const std::string & type)
{
  return type == kImageType || type == kCompressedImageType;
}

const io::TopicInfo * find_topic(const io::BagReader & reader, const std::string & name)
{
  for (const auto & t : reader.topics()) {
    if (t.name == name) {
      return &t;
    }
  }
  return nullptr;
}

VideoSourceCheck check_image_topic(const io::BagReader & reader, const std::string & topic)
{
  VideoSourceCheck check;

  const io::TopicInfo * found = find_topic(reader, topic);
  if (found == nullptr) {
    check.status = VideoSourceStatus::kTopicNotFound;
    return check;
  }

  check.topic_type = found->type;
  if (!is_supported_type(found->type)) {
    check.status = VideoSourceStatus::kUnsupportedType;
    return check;
  }

  check.status = VideoSourceStatus::kOk;
  return check;
}

// Try common image-topic suffix replacements to locate the matching CameraInfo
// topic. Longer, more specific suffixes are tried first so that a topic like
// /cam/image_raw/compressed resolves to /cam/camera_info, not a nonsensical
// intermediate name.
std::optional<std::string> infer_camera_info_topic(
  const io::BagReader & reader, const std::string & image_topic)
{
  static constexpr std::array<std::string_view, 6> kSuffixes{
    "/image_raw/compressed",
    "/image_rect/compressed",
    "/image/compressed",
    "/image_raw",
    "/image_rect",
    "/image",
  };

  for (const std::string_view suffix : kSuffixes) {
    if (image_topic.size() < suffix.size()) {
      continue;
    }
    if (image_topic.compare(image_topic.size() - suffix.size(), suffix.size(), suffix) != 0) {
      continue;
    }
    const std::string inferred =
      image_topic.substr(0, image_topic.size() - suffix.size()) + "/camera_info";
    if (find_topic(reader, inferred) != nullptr) {
      return inferred;
    }
  }
  return std::nullopt;
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

  check = check_image_topic(*reader, topic);
  if (!check.ok()) {
    check.message = "topic '" + topic + "' not found in " + input.string();
    if (check.status == VideoSourceStatus::kUnsupportedType) {
      check.message =
        "topic '" + topic + "' has type '" + check.topic_type +
        "', which generate video cannot render; supported types are " + kImageType + " and " +
        kCompressedImageType;
    }
    return check;
  }

  return check;
}

VideoSourceCheck check_video_source(const std::filesystem::path & input, const GenerateVideoArgs & args)
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

  check = check_image_topic(*reader, args.topic);
  if (!check.ok()) {
    if (check.status == VideoSourceStatus::kTopicNotFound) {
      check.message = "topic '" + args.topic + "' not found in " + input.string();
    } else if (check.status == VideoSourceStatus::kUnsupportedType) {
      check.message =
        "topic '" + args.topic + "' has type '" + check.topic_type +
        "', which generate video cannot render; supported types are " + kImageType + " and " +
        kCompressedImageType;
    }
    return check;
  }

  for (const auto & pcd_topic : args.pcd_topics) {
    const io::TopicInfo * t = find_topic(*reader, pcd_topic);
    if (t == nullptr) {
      check.status = VideoSourceStatus::kPcdTopicInvalid;
      check.message = "pointcloud topic '" + pcd_topic + "' not found in " + input.string();
      return check;
    }
    if (t->type != kPointCloud2Type) {
      check.status = VideoSourceStatus::kPcdTopicInvalid;
      check.message =
        "pointcloud topic '" + pcd_topic + "' has type '" + t->type +
        "', expected " + kPointCloud2Type;
      return check;
    }
  }

  if (!args.pcd_topics.empty()) {
    if (args.camera_info_topic.has_value()) {
      check.camera_info_topic = args.camera_info_topic.value();
    } else {
      check.camera_info_topic = infer_camera_info_topic(*reader, args.topic);
    }

    if (!check.camera_info_topic.has_value()) {
      check.status = VideoSourceStatus::kCameraInfoTopicInvalid;
      check.message =
        "could not infer CameraInfo topic from image topic '" + args.topic +
        "'; pass --camera-info explicitly";
      return check;
    }

    const io::TopicInfo * ci = find_topic(*reader, *check.camera_info_topic);
    if (ci == nullptr) {
      check.status = VideoSourceStatus::kCameraInfoTopicInvalid;
      check.message = "camera info topic '" + *check.camera_info_topic + "' not found";
      return check;
    }
    if (ci->type != kCameraInfoType) {
      check.status = VideoSourceStatus::kCameraInfoTopicInvalid;
      check.message =
        "camera info topic '" + *check.camera_info_topic + "' has type '" + ci->type +
        "', expected " + kCameraInfoType;
      return check;
    }
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

std::optional<core::camera::CameraInfo> load_camera_info(
  const std::filesystem::path & input, const std::string & topic)
{
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "failed to open '%s': %s", input.string().c_str(), e.what());
    return std::nullopt;
  }

  io::ReadFilter filter;
  filter.topics.push_back(topic);
  reader->set_filter(filter);

  io::RawMessage raw;
  if (!reader->next(raw)) {
    BAGWIZ_LOG_ERROR(kLogger, "camera info topic '%s' has no messages", topic.c_str());
    return std::nullopt;
  }

  const auto result = core::camera::extract_camera_info(raw.payload);
  if (!result.ok()) {
    BAGWIZ_LOG_ERROR(
      kLogger, "failed to decode camera info on '%s': %s", topic.c_str(), result.error.c_str());
    return std::nullopt;
  }

  return *result.image;
}

}  // namespace

int run_generate_video(const GenerateVideoArgs & args)
{
  // 1. Validate the source topic and type before touching the output. When
  //    pointcloud overlay is requested this also validates --pcd topics and
  //    resolves the CameraInfo topic.
  const auto check = check_video_source(args.input_path, args);
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
        kLogger, "output '%s' already exists; pass --overwrite to replace it.",
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

  // 5. Load static TF and CameraInfo once before Pass 2 when overlay is needed.
  const bool overlay_pcd = !args.pcd_topics.empty();
  std::unique_ptr<tf2::BufferCore> tf_buffer;
  std::optional<core::camera::CameraInfo> camera_info;
  if (overlay_pcd) {
    auto tf_result = core::load_static_tf(args.input_path);
    if (!tf_result.ok) {
      BAGWIZ_LOG_ERROR(kLogger, "failed to load static TF: %s", tf_result.error.c_str());
      return 1;
    }
    tf_buffer = std::move(tf_result.buffer);

    camera_info = load_camera_info(args.input_path, *check.camera_info_topic);
    if (!camera_info.has_value()) {
      return 1;
    }
  }

  // 6. Pass 2: decode + encode to a sibling temp path, renamed on success and
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
  for (const auto & pcd_topic : args.pcd_topics) {
    filter.topics.push_back(pcd_topic);
  }
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
  std::vector<std::byte> raw_buffer;  // mutable copy for the raw-image path
  std::vector<PcdOverlayState> latest_pcds(args.pcd_topics.size());

  try {
    while (reader->next(raw)) {
      // Update the per-topic pointcloud cache. Messages are returned in global
      // timestamp order, so the latest stored message for each topic is always
      // the most recent one whose timestamp is <= any later image timestamp.
      if (raw.topic->name != args.topic) {
        for (std::size_t i = 0; i < args.pcd_topics.size(); ++i) {
          if (raw.topic->name == args.pcd_topics[i]) {
            if (auto err = latest_pcds[i].update(raw.payload); !err.empty()) {
              BAGWIZ_LOG_ERROR(
                kLogger, "frame %" PRIu64 ", topic '%s': %s", written, args.pcd_topics[i].c_str(),
                err.c_str());
              encoder.reset();
              cleanup_tmp();
              return 1;
            }
            break;
          }
        }
        continue;
      }

      // Normalize either message type to a packed 8-bit frame the encoder
      // accepts: width, height, row stride, pixel layout, and the pixel bytes.
      std::uint32_t fw = 0;
      std::uint32_t fh = 0;
      std::uint32_t fstep = 0;
      auto fsrc = core::video::SourcePixelFormat::kBgr8;
      std::string fenc;
      std::span<std::byte> mutable_pixels;

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
        mutable_pixels = std::span<std::byte>{decoded.bgr.data(), decoded.bgr.size()};
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
        raw_buffer.assign(v.data.begin(), v.data.end());
        mutable_pixels = std::span<std::byte>{raw_buffer.data(), raw_buffer.size()};
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

      if (overlay_pcd) {
        if (auto err = paint_pointcloud_overlays(
              mutable_pixels, fw, fh, fstep, fsrc, written, args.pcd_topics, latest_pcds,
              *camera_info, *tf_buffer, raw.timestamp_ns, args.color_by, args.color_map);
            !err.empty()) {
          BAGWIZ_LOG_ERROR(kLogger, "frame %" PRIu64 ": %s", written, err.c_str());
          encoder.reset();
          cleanup_tmp();
          return 1;
        }
      }

      const std::span<const std::byte> fdata{mutable_pixels.data(), mutable_pixels.size()};
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

  // 7. Now that the new video is complete, replace any existing output and move
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
