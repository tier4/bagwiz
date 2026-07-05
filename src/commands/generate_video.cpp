// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/generate_video.hpp"

#include "bagwiz/core/camera_info_resolver.hpp"
#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/image/camera_info.hpp"
#include "bagwiz/core/image/packed_raster.hpp"
#include "bagwiz/core/image/undistort.hpp"
#include "bagwiz/core/logging.hpp"
#include "bagwiz/core/output_path.hpp"
#include "bagwiz/core/pointcloud/fetcher.hpp"
#include "bagwiz/core/pointcloud/overlay.hpp"
#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/core/pointcloud/projector.hpp"
#include "bagwiz/core/pointcloud/projector_helpers.hpp"
#include "bagwiz/core/tf_buffer_loader.hpp"
#include "bagwiz/core/tf_value_extract.hpp"
#include "bagwiz/core/video/frame_rate.hpp"
#include "bagwiz/core/video/video_encoder.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/buffer_core.hpp>
#include <tf2/exceptions.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <future>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{
constexpr const char * kLogger = "bagwiz.cmd.generate";
constexpr const char * kImageType = "sensor_msgs/msg/Image";
constexpr const char * kCompressedImageType = "sensor_msgs/msg/CompressedImage";
constexpr const char * kPointCloudType = "sensor_msgs/msg/PointCloud2";
constexpr std::string_view kTfStaticSuffix = "tf_static";

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

using PointCloudIndexEntry = core::pointcloud::PointCloudIndexEntry;
using PointCloudSpan = core::pointcloud::PointCloudIndex;

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

using PointCloudFetcher = core::pointcloud::PointCloudFetcher;

// Pass 1: scan the point-cloud topic, record every timestamp, and compute the
// global min/max of the selected property unless the user supplied --min/--max.
int scan_pointcloud_span(
  const std::filesystem::path & input, const std::string & topic,
  core::pointcloud::PointCloudProperty property, const std::optional<double> & manual_min,
  const std::optional<double> & manual_max, PointCloudSpan & out)
{
  std::string error;
  auto idx = core::pointcloud::build_point_cloud_index(
    input, topic, property, manual_min, manual_max, error);
  if (!idx.has_value()) {
    BAGWIZ_LOG_ERROR(kLogger, "%s", error.c_str());
    return 1;
  }
  out = std::move(*idx);
  return 0;
}

// Result of point-cloud transform/projection work. Kept separate from
// ProjectionResult so callers can return an error string without throwing.
struct ProjectionWorkResult
{
  std::vector<core::pointcloud::ProjectedPoint> points;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return error.empty(); }
};

// The clock a camera frame is matched against for one cloud topic: capture time
// (header.stamp) when both the frame and that topic carry header stamps, else
// bag record time so the comparison stays within a single clock.
struct FrameMatch
{
  std::int64_t target_ns;
  core::pointcloud::PointCloudMatchKey key;
};

FrameMatch choose_frame_match(
  std::int64_t header_stamp_ns, std::int64_t record_ns, bool topic_has_stamps)
{
  if (header_stamp_ns > 0 && topic_has_stamps) {
    return {header_stamp_ns, core::pointcloud::PointCloudMatchKey::kHeaderStamp};
  }
  return {record_ns, core::pointcloud::PointCloudMatchKey::kRecordTime};
}

// Fetch, parse, transform, and project the point cloud nearest the frame for
// every listed topic. Each topic is matched in its own clock (see
// choose_frame_match): `frame_header_stamp_ns` (0 if unset) and `frame_record_ns`
// are the frame's two clocks, and `topic_has_stamps[i]` says whether topic i can
// be matched by capture time. Each call opens its own BagReader(s) so the work
// can safely run on a background thread; the caller supplies the read-only
// camera info and TF buffer.
ProjectionWorkResult run_projection_work(
  const std::filesystem::path & input, const std::vector<std::string> & pointcloud_topics,
  const std::vector<std::vector<PointCloudIndexEntry>> & entries_per_topic,
  const std::vector<bool> & topic_has_stamps, std::int64_t frame_header_stamp_ns,
  std::int64_t frame_record_ns, const core::image::CameraInfo & camera_info,
  tf2::BufferCore & tf_buffer, std::uint32_t image_width, std::uint32_t image_height,
  core::pointcloud::PointCloudProperty property, bool use_rectified)
{
  try {
    ProjectionWorkResult combined;
    for (std::size_t i = 0; i < pointcloud_topics.size(); ++i) {
      PointCloudFetcher fetcher(input, pointcloud_topics[i], entries_per_topic[i]);
      std::string error;
      const auto match =
        choose_frame_match(frame_header_stamp_ns, frame_record_ns, topic_has_stamps[i]);
      const auto * cloud = fetcher.fetch(match.target_ns, match.key, error);
      if (cloud == nullptr) {
        return {{}, std::move(error)};
      }
      const auto projected = core::pointcloud::project_cloud_for_frame(
        *cloud, camera_info, tf_buffer, image_width, image_height, property, use_rectified);
      if (!projected.ok()) {
        return {{}, std::move(projected.error)};
      }
      combined.points.insert(
        combined.points.end(), projected.points.begin(), projected.points.end());
    }
    return combined;
  } catch (const std::exception & e) {
    return {{}, std::string("point-cloud projection failed: ") + e.what()};
  }
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
    if (const auto err =
          core::camera_info::validate_camera_info_topic(args.input_path, *camera_info_topic);
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
    camera_info_topic =
      core::camera_info::resolve_camera_info_topic(args.topic, reader->topics()).topic;
  }

  const bool needs_camera_info = args.undistort || !args.pointcloud_topics.empty();
  if (needs_camera_info && !camera_info_topic.has_value()) {
    BAGWIZ_LOG_ERROR(
      kLogger,
      "A camera-info topic is required for --undistort or --pcd, but none could be derived from "
      "'%s'. Pass it explicitly with --cam-info.",
      args.topic.c_str());
    return 1;
  }

  // 2b. Validate every optional point-cloud topic.
  for (const auto & topic : args.pointcloud_topics) {
    std::unique_ptr<io::BagReader> reader;
    try {
      reader = io::open_read(args.input_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(
        kLogger, "failed to open '%s': %s", args.input_path.string().c_str(), e.what());
      return 1;
    }
    bool found = false;
    for (const auto & t : reader->topics()) {
      if (t.name == topic) {
        found = true;
        if (t.type != kPointCloudType) {
          BAGWIZ_LOG_ERROR(
            kLogger, "pcd topic '%s' has type '%s', expected %s", topic.c_str(), t.type.c_str(),
            kPointCloudType);
          return 1;
        }
        break;
      }
    }
    if (!found) {
      BAGWIZ_LOG_ERROR(
        kLogger, "pcd topic '%s' not found in %s", topic.c_str(), args.input_path.string().c_str());
      return 1;
    }
  }

  // 2c. Point-cloud overlay needs a camera-info topic to project into the image.
  if (!args.pointcloud_topics.empty() && !camera_info_topic.has_value()) {
    BAGWIZ_LOG_ERROR(
      kLogger,
      "point-cloud overlay requires a cam-info topic, but none could be derived from '%s'. "
      "Pass it explicitly with --cam-info.",
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

  // 5b. Pass 1 for the point-cloud overlay: scan timestamps and the selected
  //     property's global min/max across all topics.
  std::vector<PointCloudSpan> pcd_spans;
  double global_property_min = 0.0;
  double global_property_max = 0.0;
  // Per topic: whether it can be matched by capture time (every cloud carried a
  // header.stamp). Topics that fall back to record time are matched by record
  // time on both sides so the overlay stays in one clock.
  std::vector<bool> pcd_topic_has_stamps;
  if (!args.pointcloud_topics.empty()) {
    pcd_spans.resize(args.pointcloud_topics.size());
    pcd_topic_has_stamps.resize(args.pointcloud_topics.size());
    double running_min = std::numeric_limits<double>::infinity();
    double running_max = -std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < args.pointcloud_topics.size(); ++i) {
      if (
        scan_pointcloud_span(
          args.input_path, args.pointcloud_topics[i], args.property, args.property_min,
          args.property_max, pcd_spans[i]) != 0) {
        return 1;
      }
      pcd_topic_has_stamps[i] = pcd_spans[i].header_stamps_present;
      if (!args.property_min.has_value()) {
        running_min = std::min(running_min, pcd_spans[i].property_min);
      }
      if (!args.property_max.has_value()) {
        running_max = std::max(running_max, pcd_spans[i].property_max);
      }
    }
    global_property_min = args.property_min.value_or(running_min);
    global_property_max = args.property_max.value_or(running_max);
  }

  // 6. Load camera info if it will be used. Doing this before Pass 2 avoids
  //    aborting half-way through the encode.
  std::optional<core::image::CameraInfo> camera_info;
  if (camera_info_topic.has_value()) {
    auto ci = core::camera_info::load_camera_info(args.input_path, *camera_info_topic);
    if (!ci.ok()) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", ci.error.c_str());
      return 1;
    }
    camera_info = core::image::scale_camera_info(*ci.info, static_cast<double>(args.resize_scale));
  }

  // 6b. Load TF data when a point-cloud overlay is requested so each cloud can be
  //     transformed into the camera frame before projection.
  std::optional<tf2::BufferCore> tf_buffer;
  if (!args.pointcloud_topics.empty()) {
    tf_buffer.emplace();
    if (const auto err = core::load_tf_buffer(args.input_path, *tf_buffer); err.has_value()) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", err->c_str());
      return 1;
    }
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

  // Each message (raw Image or CompressedImage) is normalized to a canonical
  // packed BGR24 raster by core::image::to_packed_raster before encoding.
  std::unique_ptr<core::video::VideoEncoder> encoder;
  std::unique_ptr<core::image::UndistortHelper> undistort_helper;
  std::uint32_t enc_w = 0;
  std::uint32_t enc_h = 0;
  // to_packed_raster yields canonical BGR24, so every frame's encoding is "bgr8";
  // the encoder still tracks it to guard against a mid-run geometry change.
  std::string enc_encoding;
  std::uint64_t written = 0;
  const bool use_rectified = args.undistort || !args.pointcloud_topics.empty();

  // Owned decode buffer that survives across BagReader::next() calls, which
  // invalidate raw payload spans. Used by both the synchronous and the threaded
  // point-cloud overlay paths.
  struct FrameBuffer
  {
    std::int64_t timestamp_ns = 0;     // bag record time
    std::int64_t header_stamp_ns = 0;  // image header.stamp (0 if unset)
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t step = 0;
    core::video::SourcePixelFormat pixel_format = core::video::SourcePixelFormat::kBgr8;
    std::string encoding;
    std::vector<std::byte> data;
  };

  auto decode_to_buffer = [&](
                            std::int64_t timestamp_ns,
                            std::span<const std::byte> payload) -> std::optional<FrameBuffer> {
    // Normalize either message type to a canonical packed BGR24 raster via the
    // shared core::image::to_packed_raster seam; rgb8 inputs are swapped so
    // every frame the encoder sees is BGR24.
    auto pr = core::image::to_packed_raster(check.topic_type, payload);
    if (!pr.ok()) {
      BAGWIZ_LOG_ERROR(kLogger, "frame %" PRIu64 ": %s", written, pr.error.c_str());
      return std::nullopt;
    }
    FrameBuffer frame;
    frame.timestamp_ns = timestamp_ns;
    frame.header_stamp_ns = pr.raster->header_stamp_ns;
    frame.width = pr.raster->width;
    frame.height = pr.raster->height;
    frame.step = pr.raster->width * 3U;
    frame.pixel_format = core::video::SourcePixelFormat::kBgr8;
    frame.encoding = "bgr8";
    frame.data = std::move(pr.raster->bgr);
    return frame;
  };

  // Resize a decoded frame in-place by `args.resize_scale`, preserving aspect ratio.
  // Returns false and logs on failure.
  auto resize_decoded_frame = [&](FrameBuffer & frame) -> bool {
    const double scale = static_cast<double>(args.resize_scale);
    if (scale == 1.0) {
      return true;
    }
    const std::uint32_t out_w = static_cast<std::uint32_t>(std::lround(frame.width * scale));
    const std::uint32_t out_h = static_cast<std::uint32_t>(std::lround(frame.height * scale));
    if (out_w == 0 || out_h == 0) {
      BAGWIZ_LOG_ERROR(
        kLogger, "resize scale %.3f would produce a zero-size frame (%ux%u)", scale, out_w, out_h);
      return false;
    }

    const cv::Mat in(
      static_cast<int>(frame.height), static_cast<int>(frame.width), CV_8UC3, frame.data.data(),
      frame.step);
    cv::Mat out;
    const int interpolation = (scale < 1.0) ? cv::INTER_AREA : cv::INTER_LINEAR;
    cv::resize(
      in, out, cv::Size{static_cast<int>(out_w), static_cast<int>(out_h)}, 0, 0, interpolation);

    frame.width = out_w;
    frame.height = out_h;
    frame.step = out_w * 3U;
    frame.data.assign(
      reinterpret_cast<std::byte *>(out.data),
      reinterpret_cast<std::byte *>(out.data) + out.total() * out.elemSize());
    return true;
  };

  auto encode_frame = [&](
                        FrameBuffer & frame,
                        const std::vector<core::pointcloud::ProjectedPoint> * projected) -> bool {
    std::span<const std::byte> fdata{frame.data.data(), frame.data.size()};
    std::uint32_t fstep = frame.step;

    if (encoder == nullptr) {
      // The first frame fixes the geometry and pixel encoding for the run.
      enc_w = frame.width;
      enc_h = frame.height;
      enc_encoding = frame.encoding;
      auto opened = core::video::open_video_encoder(tmp_path, enc_w, enc_h, fps.num, fps.den);
      if (!opened.ok()) {
        BAGWIZ_LOG_ERROR(kLogger, "%s", opened.error.c_str());
        return false;
      }
      encoder = std::move(opened.encoder);

      if (args.undistort || !args.pointcloud_topics.empty()) {
        undistort_helper =
          std::make_unique<core::image::UndistortHelper>(*camera_info, enc_w, enc_h);
      }
    } else if (frame.width != enc_w || frame.height != enc_h || frame.encoding != enc_encoding) {
      BAGWIZ_LOG_ERROR(
        kLogger, "frame %" PRIu64 " changed to %ux%u %s from the first frame's %ux%u %s; aborting.",
        written, frame.width, frame.height, frame.encoding.c_str(), enc_w, enc_h,
        enc_encoding.c_str());
      return false;
    }

    if (undistort_helper != nullptr) {
      fdata = undistort_helper->remap(fdata, fstep);
      fstep = enc_w * 3U;
    }

    core::image::PackedRaster overlay_output;
    if (projected != nullptr) {
      core::image::PackedRaster src;
      src.width = frame.width;
      src.height = frame.height;
      src.encoding = frame.encoding;
      const std::size_t row_bytes = static_cast<std::size_t>(frame.width) * 3U;
      src.bgr.resize(row_bytes * frame.height);
      for (std::uint32_t y = 0; y < frame.height; ++y) {
        std::copy_n(fdata.data() + y * fstep, row_bytes, src.bgr.data() + y * row_bytes);
      }

      overlay_output.width = frame.width;
      overlay_output.height = frame.height;
      overlay_output.encoding = frame.encoding;
      overlay_output.bgr.resize(src.bgr.size());
      if (const auto err = core::pointcloud::overlay_projected_points(
            src, *projected, global_property_min, global_property_max, args.colorscheme,
            args.point_size, args.alpha, overlay_output);
          !err.empty()) {
        BAGWIZ_LOG_ERROR(kLogger, "overlay failed: %s", err.c_str());
        return false;
      }
      fdata = std::span<const std::byte>(overlay_output.bgr.data(), overlay_output.bgr.size());
      fstep = static_cast<std::uint32_t>(row_bytes);
    }

    if (auto e = encoder->write_frame(fdata, fstep, frame.pixel_format); !e.empty()) {
      BAGWIZ_LOG_ERROR(kLogger, "frame %" PRIu64 ": %s", written, e.c_str());
      return false;
    }
    ++written;
    return true;
  };

  // Threading is only worthwhile when there is enough work to hide the overhead
  // of launching a thread and opening a fresh BagReader per frame.
  constexpr std::uint64_t kThreadingMinFrames = 4;
  const bool use_threaded_projection =
    !args.pointcloud_topics.empty() && args.enable_threaded_projection &&
    span.count >= kThreadingMinFrames && std::thread::hardware_concurrency() > 1;

  // The synchronous path keeps one cached fetcher per topic for small bags or
  // when threading is disabled; the async path opens a fresh reader per frame.
  std::vector<PointCloudFetcher> pcd_fetchers;
  if (!args.pointcloud_topics.empty() && !use_threaded_projection) {
    pcd_fetchers.reserve(args.pointcloud_topics.size());
    for (std::size_t i = 0; i < args.pointcloud_topics.size(); ++i) {
      pcd_fetchers.emplace_back(
        args.input_path, args.pointcloud_topics[i], std::move(pcd_spans[i].entries));
    }
  }

  // The async path needs the per-topic index entries after moving them out of
  // pcd_spans; collect them before the encode loop.
  std::vector<std::vector<PointCloudIndexEntry>> entries_per_topic;
  if (use_threaded_projection) {
    entries_per_topic.reserve(pcd_spans.size());
    for (auto & pcd_span : pcd_spans) {
      entries_per_topic.push_back(std::move(pcd_span.entries));
    }
  }

  io::RawMessage raw;
  try {
    if (use_threaded_projection) {
      // Async path: keep one frame of projection work running ahead so that
      // fetch/parse/project for frame N+1 overlaps with encoding frame N.
      auto launch_projection = [&](
                                 std::int64_t header_stamp_ns, std::int64_t record_ns,
                                 std::uint32_t w,
                                 std::uint32_t h) -> std::future<ProjectionWorkResult> {
        return std::async(
          std::launch::async, [&, header_stamp_ns, record_ns, w, h, rectified = use_rectified]() {
            return run_projection_work(
              args.input_path, args.pointcloud_topics, entries_per_topic, pcd_topic_has_stamps,
              header_stamp_ns, record_ns, *camera_info, *tf_buffer, w, h, args.property, rectified);
          });
      };

      if (!reader->next(raw)) {
        BAGWIZ_LOG_ERROR(
          kLogger, "topic '%s' yielded no frames in the encode pass.", args.topic.c_str());
        cleanup_tmp();
        return 1;
      }
      auto current = decode_to_buffer(raw.timestamp_ns, raw.payload);
      if (!current) {
        cleanup_tmp();
        return 1;
      }
      if (!resize_decoded_frame(*current)) {
        cleanup_tmp();
        return 1;
      }
      auto pending_projection = launch_projection(
        current->header_stamp_ns, current->timestamp_ns, current->width, current->height);

      while (true) {
        auto projected = pending_projection.get();
        if (!projected.ok()) {
          BAGWIZ_LOG_ERROR(kLogger, "frame %" PRIu64 ": %s", written, projected.error.c_str());
          encoder.reset();
          cleanup_tmp();
          return 1;
        }

        io::RawMessage next_raw;
        const bool has_next = reader->next(next_raw);
        std::optional<FrameBuffer> next_frame;
        std::optional<std::future<ProjectionWorkResult>> next_projection;
        if (has_next) {
          next_frame = decode_to_buffer(next_raw.timestamp_ns, next_raw.payload);
          if (!next_frame) {
            encoder.reset();
            cleanup_tmp();
            return 1;
          }
          if (!resize_decoded_frame(*next_frame)) {
            encoder.reset();
            cleanup_tmp();
            return 1;
          }
          next_projection = launch_projection(
            next_frame->header_stamp_ns, next_frame->timestamp_ns, next_frame->width,
            next_frame->height);
        }

        if (!encode_frame(*current, &projected.points)) {
          encoder.reset();
          cleanup_tmp();
          return 1;
        }

        if (!has_next) {
          break;
        }
        current = std::move(*next_frame);
        pending_projection = std::move(*next_projection);
      }
    } else {
      // Synchronous path: decode, optionally project, and encode frame-by-frame.
      // This keeps the original cached PointCloudFetcher so small bags or
      // single-threaded runs do not pay the per-frame BagReader open/close cost.
      while (reader->next(raw)) {
        auto frame = decode_to_buffer(raw.timestamp_ns, raw.payload);
        if (!frame) {
          encoder.reset();
          cleanup_tmp();
          return 1;
        }
        if (!resize_decoded_frame(*frame)) {
          encoder.reset();
          cleanup_tmp();
          return 1;
        }

        std::vector<core::pointcloud::ProjectedPoint> projected_storage;
        const std::vector<core::pointcloud::ProjectedPoint> * projected_ptr = nullptr;
        if (!pcd_fetchers.empty()) {
          for (std::size_t i = 0; i < pcd_fetchers.size(); ++i) {
            std::string pcd_error;
            const auto match = choose_frame_match(
              frame->header_stamp_ns, frame->timestamp_ns, pcd_topic_has_stamps[i]);
            const auto * cloud = pcd_fetchers[i].fetch(match.target_ns, match.key, pcd_error);
            if (cloud == nullptr) {
              BAGWIZ_LOG_ERROR(kLogger, "frame %" PRIu64 ": %s", written, pcd_error.c_str());
              encoder.reset();
              cleanup_tmp();
              return 1;
            }

            const auto projected = core::pointcloud::project_cloud_for_frame(
              *cloud, *camera_info, *tf_buffer, frame->width, frame->height, args.property,
              use_rectified);
            if (!projected.ok()) {
              BAGWIZ_LOG_ERROR(kLogger, "frame %" PRIu64 ": %s", written, projected.error.c_str());
              encoder.reset();
              cleanup_tmp();
              return 1;
            }
            projected_storage.insert(
              projected_storage.end(), projected.points.begin(), projected.points.end());
          }
          projected_ptr = &projected_storage;
        }

        if (!encode_frame(*frame, projected_ptr)) {
          encoder.reset();
          cleanup_tmp();
          return 1;
        }
      }
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
    kLogger, "generate video: wrote %" PRIu64 " frame(s) to %s (%ux%u bgr8 @ %.3g fps).", written,
    args.output_path.string().c_str(), enc_w, enc_h, fps_value);

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
