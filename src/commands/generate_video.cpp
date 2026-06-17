// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/generate_video.hpp"

#include "bagwiz/core/image/camera_info.hpp"
#include "bagwiz/core/image/compressed_image.hpp"
#include "bagwiz/core/image/image_decoder.hpp"
#include "bagwiz/core/image/raw_image.hpp"
#include "bagwiz/core/logging.hpp"
#include "bagwiz/core/output_path.hpp"
#include "bagwiz/core/video/frame_rate.hpp"
#include "bagwiz/core/video/video_encoder.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <cinttypes>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <string>
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
constexpr const char * kCameraInfoType = "sensor_msgs/msg/CameraInfo";

bool is_supported_type(const std::string & type)
{
  return type == kImageType || type == kCompressedImageType;
}

// Returns true for extensions that the encoder maps to H.264. Used only for
// user-facing playback guidance.
bool is_h264_extension(const std::filesystem::path & output)
{
  std::string ext = output.extension().string();
  for (auto & c : ext) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return ext == ".mp4" || ext == ".mkv" || ext == ".mov";
}

// Best-effort check for a vlc executable on the host. Used only to decide
// whether to append an install hint to the H.264 playback guidance.
bool is_vlc_available()
{
#ifdef _WIN32
  return std::system("where vlc >nul 2>nul") == 0;
#else
  return std::system("command -v vlc >/dev/null 2>&1") == 0;
#endif
}

// Platform-specific one-line hint for installing VLC.
const char * vlc_install_hint()
{
#ifdef __linux__
  return "Install VLC with your package manager (e.g. 'sudo apt install vlc').";
#elif __APPLE__
  return "Install VLC with: brew install vlc";
#elif _WIN32
  return "Install VLC from https://www.videolan.org/vlc/";
#else
  return "Install VLC from https://www.videolan.org/vlc/";
#endif
}

// Timestamps + count for a single topic, gathered by a payload-free scan.
struct TopicSpan
{
  std::int64_t first_ns = 0;
  std::int64_t last_ns = 0;
  std::uint64_t count = 0;
};

// Auto-resolve a CameraInfo topic from the image topic name. The rules are
// deliberately narrow: only the three known image suffixes map to a sibling
// /camera_info topic.
std::optional<std::string> resolve_camera_info_topic(
  const std::string & image_topic, std::span<const io::TopicInfo> topics)
{
  constexpr std::string_view kImageRawCompressed = "/image_raw/compressed";
  constexpr std::string_view kImageRectColorCompressed = "/image_rect_color/compressed";
  constexpr std::string_view kImageRectColor = "/image_rect_color";
  constexpr std::string_view kCameraInfoSuffix = "/camera_info";

  std::string_view stem{image_topic};
  if (stem.ends_with(kImageRawCompressed)) {
    stem.remove_suffix(kImageRawCompressed.size());
  } else if (stem.ends_with(kImageRectColorCompressed)) {
    stem.remove_suffix(kImageRectColorCompressed.size());
  } else if (stem.ends_with(kImageRectColor)) {
    stem.remove_suffix(kImageRectColor.size());
  } else {
    return std::nullopt;
  }

  std::string candidate{stem};
  candidate += kCameraInfoSuffix;
  for (const auto & t : topics) {
    if (t.name == candidate && t.type == kCameraInfoType) {
      return candidate;
    }
  }
  return std::nullopt;
}

// Validate an explicit camera-info topic: it must exist in the bag and have the
// expected type. Returns a human-readable error on failure, or std::nullopt on
// success.
std::optional<std::string> validate_camera_info_topic(
  const std::filesystem::path & input, const std::string & topic)
{
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input);
  } catch (const std::exception & e) {
    return "failed to open '" + input.string() + "': " + e.what();
  }

  for (const auto & t : reader->topics()) {
    if (t.name == topic) {
      if (t.type != kCameraInfoType) {
        return "topic '" + topic + "' has type '" + t.type + "', but --camera-info requires " +
               kCameraInfoType;
      }
      return std::nullopt;
    }
  }
  return "camera-info topic '" + topic + "' not found in " + input.string();
}

// Read the first CameraInfo message for `topic` from `input`.
core::image::CameraInfoResult load_camera_info(
  const std::filesystem::path & input, const std::string & topic)
{
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input);
  } catch (const std::exception & e) {
    core::image::CameraInfoResult result;
    result.error = "failed to open '" + input.string() + "': " + e.what();
    return result;
  }

  io::ReadFilter filter;
  filter.topics.push_back(topic);
  reader->set_filter(filter);

  io::RawMessage raw;
  try {
    if (reader->next(raw)) {
      return core::image::extract_camera_info(raw.payload);
    }
  } catch (const std::exception & e) {
    core::image::CameraInfoResult result;
    result.error = "error reading camera-info topic '" + topic + "': " + e.what();
    return result;
  }

  core::image::CameraInfoResult result;
  result.error = "camera-info topic '" + topic + "' has no messages";
  return result;
}

// OpenCV undistortion helper. Initializes distortion/rectification maps from
// the first frame's dimensions and the loaded CameraInfo, then remaps each
// subsequent frame. Owns the output buffer passed to the video encoder.
class UndistortHelper
{
public:
  UndistortHelper(const core::image::CameraInfo & info, std::uint32_t width, std::uint32_t height)
  : width_(width), height_(height)
  {
    // cv::Mat's external-data constructor takes a non-const void* even when the
    // matrix is only read. initUndistortRectifyMap does not mutate these inputs,
    // so the const_cast is safe and lets us wrap the fixed arrays directly.
    const cv::Mat k(3, 3, CV_64F, const_cast<double *>(info.k.data()));
    const cv::Mat r(3, 3, CV_64F, const_cast<double *>(info.r.data()));
    const cv::Mat p(3, 4, CV_64F, const_cast<double *>(info.p.data()));

    cv::Mat d;
    if (!info.d.empty()) {
      d = cv::Mat(static_cast<int>(info.d.size()), 1, CV_64F, const_cast<double *>(info.d.data()));
    }

    cv::initUndistortRectifyMap(
      k, d, r, p, cv::Size{static_cast<int>(width), static_cast<int>(height)}, CV_32FC1, map1_,
      map2_);

    output_.resize(static_cast<std::size_t>(width) * height * 3, std::byte{0});
  }

  UndistortHelper(const UndistortHelper &) = delete;
  UndistortHelper & operator=(const UndistortHelper &) = delete;
  UndistortHelper(UndistortHelper &&) = default;
  UndistortHelper & operator=(UndistortHelper &&) = default;

  // Apply undistortion to a packed 8-bit BGR/RGB frame. `src_step` is the row
  // stride in bytes. Returns a span over the packed output buffer.
  std::span<const std::byte> remap(std::span<const std::byte> src, std::uint32_t src_step)
  {
    const cv::Mat in(
      static_cast<int>(height_), static_cast<int>(width_), CV_8UC3,
      const_cast<std::byte *>(src.data()), src_step);
    cv::Mat out(
      static_cast<int>(height_), static_cast<int>(width_), CV_8UC3, output_.data(), width_ * 3);
    cv::remap(in, out, map1_, map2_, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar{});
    return {output_.data(), output_.size()};
  }

private:
  std::uint32_t width_ = 0;
  std::uint32_t height_ = 0;
  cv::Mat map1_;
  cv::Mat map2_;
  std::vector<std::byte> output_;
};

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

int run_generate_video(const GenerateVideoArgs & args)
{
  // 1. Validate the source topic and type before touching the output.
  const auto check = check_video_source(args.input_path, args.topic);
  if (!check.ok()) {
    BAGWIZ_LOG_ERROR(kLogger, "%s", check.message.c_str());
    return 1;
  }

  // 2. Resolve and validate the camera-info topic when needed.
  std::optional<std::string> camera_info_topic = args.camera_info_topic;
  if (camera_info_topic.has_value()) {
    if (const auto err = validate_camera_info_topic(args.input_path, *camera_info_topic);
        err.has_value()) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", err->c_str());
      return 1;
    }
  } else {
    std::unique_ptr<io::BagReader> reader;
    try {
      reader = io::open_read(args.input_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(
        kLogger, "failed to open '%s': %s", args.input_path.string().c_str(), e.what());
      return 1;
    }
    camera_info_topic = resolve_camera_info_topic(args.topic, reader->topics());
  }

  if (args.undistort && !camera_info_topic.has_value()) {
    BAGWIZ_LOG_ERROR(
      kLogger,
      "--undistort requires a camera-info topic, but none could be derived from '%s'. "
      "Pass it explicitly with --camera-info.",
      args.topic.c_str());
    return 1;
  }

  // 3. Fail fast on an output collision before the expensive encode. The actual
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

  // 4. Create the output's parent directory if needed.
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

  // 5. Pass 1: derive the frame rate from the message timestamps.
  TopicSpan span;
  if (scan_topic_span(args.input_path, args.topic, span) != 0) {
    return 1;
  }
  if (span.count == 0) {
    BAGWIZ_LOG_ERROR(kLogger, "topic '%s' has no messages to render.", args.topic.c_str());
    return 1;
  }
  const auto fps = core::video::derive_frame_rate(span.first_ns, span.last_ns, span.count);

  // 6. Load camera info if it will be used. Doing this before Pass 2 avoids
  //    aborting half-way through the encode.
  std::optional<core::image::CameraInfo> camera_info;
  if (camera_info_topic.has_value()) {
    auto ci = load_camera_info(args.input_path, *camera_info_topic);
    if (!ci.ok()) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", ci.error.c_str());
      return 1;
    }
    camera_info = std::move(*ci.info);
  }

  // 7. Pass 2: decode + encode to a sibling temp path, renamed on success and
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
  std::unique_ptr<UndistortHelper> undistort_helper;
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

        if (args.undistort) {
          undistort_helper = std::make_unique<UndistortHelper>(*camera_info, enc_w, enc_h);
        }
      } else if (fw != enc_w || fh != enc_h || fenc != enc_encoding) {
        BAGWIZ_LOG_ERROR(
          kLogger,
          "frame %" PRIu64 " changed to %ux%u %s from the first frame's %ux%u %s; aborting.",
          written, fw, fh, fenc.c_str(), enc_w, enc_h, enc_encoding.c_str());
        encoder.reset();
        cleanup_tmp();
        return 1;
      }

      if (undistort_helper != nullptr) {
        fdata = undistort_helper->remap(fdata, fstep);
        fstep = enc_w * 3U;
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

  // 8. Now that the new video is complete, replace any existing output and move
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

  if (is_h264_extension(args.output_path)) {
    if (is_vlc_available()) {
      BAGWIZ_LOG_INFO(
        kLogger, "H.264 output saved. If mpv fails to play, try VLC or run mpv --hwdec=no.");
    } else {
      BAGWIZ_LOG_WARN(
        kLogger,
        "H.264 output saved. If mpv fails to play, run mpv --hwdec=no, or install VLC (%s)",
        vlc_install_hint());
    }
  }

  return 0;
}

}  // namespace bagwiz::commands
