// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "generate_video_common.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/base/output_path.hpp"
#include "bagwiz/core/image/camera_info_resolver.hpp"
#include "bagwiz/core/image/packed_raster.hpp"
#include "bagwiz/core/pointcloud/overlay.hpp"
#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/core/pointcloud/projector_helpers.hpp"
#include "bagwiz/core/tf/tf_buffer_loader.hpp"
#include "bagwiz/io/topics.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <future>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <thread>
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

// Below this many frames the threaded projection pipeline cannot hide the
// overhead of launching a thread and opening a fresh BagReader per frame.
constexpr std::uint64_t kThreadingMinFrames = 4;

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

// Pass 1: scan the point-cloud topic, record every timestamp, and compute the
// global min/max of the selected property unless the user supplied --min/--max.
int scan_pointcloud_span(
  const std::filesystem::path & input, const std::string & topic,
  core::pointcloud::PointCloudProperty property, const std::optional<double> & manual_min,
  const std::optional<double> & manual_max, core::pointcloud::PointCloudIndex & out)
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

// Fetch, parse, transform, and project the point cloud nearest the frame for
// every listed topic. Each topic is matched in its own clock (see
// core::pointcloud::choose_frame_match): `frame_header_stamp_ns` (0 if unset)
// and `frame_record_ns` are the frame's two clocks, and `topic_has_stamps[i]`
// says whether topic i can be matched by capture time. Each call opens its own
// BagReader(s) so the work can safely run on a background thread; the caller
// supplies the read-only camera info and TF buffer.
ProjectionWorkResult run_projection_work(
  const std::filesystem::path & input, const std::vector<std::string> & pointcloud_topics,
  const std::vector<std::vector<core::pointcloud::PointCloudIndexEntry>> & entries_per_topic,
  const std::vector<bool> & topic_has_stamps, std::int64_t frame_header_stamp_ns,
  std::int64_t frame_record_ns, const core::image::CameraInfo & camera_info,
  tf2::BufferCore & tf_buffer, std::uint32_t image_width, std::uint32_t image_height,
  core::pointcloud::PointCloudProperty property, bool use_rectified)
{
  try {
    ProjectionWorkResult combined;
    for (std::size_t i = 0; i < pointcloud_topics.size(); ++i) {
      core::pointcloud::PointCloudFetcher fetcher(
        input, pointcloud_topics[i], entries_per_topic[i]);
      std::string error;
      const auto match = core::pointcloud::choose_frame_match(
        frame_header_stamp_ns, frame_record_ns, topic_has_stamps[i]);
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

VideoInputValidation validate_video_inputs(const GenerateVideoArgs & args)
{
  VideoInputValidation out;

  // Validate the source topic and type before touching anything else.
  out.check = check_video_source(args.input_path, args.topic);
  if (!out.check.ok()) {
    BAGWIZ_LOG_ERROR(kLogger, "%s", out.check.message.c_str());
    out.error = out.check.message;
    return out;
  }

  // Resolve and validate the camera-info topic when needed.
  if (args.camera_info_topic.has_value()) {
    if (const auto err =
          core::camera_info::validate_camera_info_topic(args.input_path, *args.camera_info_topic);
        err.has_value()) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", err->c_str());
      out.error = *err;
      return out;
    }
    out.camera_info_topic = args.camera_info_topic;
  } else {
    std::unique_ptr<io::BagReader> reader;
    try {
      reader = io::open_read(args.input_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(
        kLogger, "failed to open '%s': %s", args.input_path.string().c_str(), e.what());
      out.error = "failed to open '" + args.input_path.string() + "': " + e.what();
      return out;
    }
    out.camera_info_topic =
      core::camera_info::resolve_camera_info_topic(args.topic, reader->topics()).topic;
  }

  const bool needs_camera_info = args.undistort || !args.pointcloud_topics.empty();
  if (needs_camera_info && !out.camera_info_topic.has_value()) {
    BAGWIZ_LOG_ERROR(
      kLogger,
      "A camera-info topic is required for --undistort or --pcd, but none could be derived from "
      "'%s'. Pass it explicitly with --cam-info.",
      args.topic.c_str());
    out.error =
      "A camera-info topic is required for --undistort or --pcd, but none could be derived from "
      "'" +
      args.topic + "'. Pass it explicitly with --cam-info.";
    return out;
  }

  // Validate every optional point-cloud topic.
  for (const auto & topic : args.pointcloud_topics) {
    std::unique_ptr<io::BagReader> reader;
    try {
      reader = io::open_read(args.input_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(
        kLogger, "failed to open '%s': %s", args.input_path.string().c_str(), e.what());
      out.error = "failed to open '" + args.input_path.string() + "': " + e.what();
      return out;
    }
    const io::TopicInfo * info = io::find_topic(*reader, topic);
    if (info == nullptr) {
      BAGWIZ_LOG_ERROR(
        kLogger, "pcd topic '%s' not found in %s", topic.c_str(), args.input_path.string().c_str());
      out.error = "pcd topic '" + topic + "' not found in " + args.input_path.string();
      return out;
    }
    if (info->type != kPointCloudType) {
      BAGWIZ_LOG_ERROR(
        kLogger, "pcd topic '%s' has type '%s', expected %s", topic.c_str(), info->type.c_str(),
        kPointCloudType);
      out.error =
        "pcd topic '" + topic + "' has type '" + info->type + "', expected " + kPointCloudType;
      return out;
    }
  }

  // Point-cloud overlay needs a camera-info topic to project into the image.
  if (!args.pointcloud_topics.empty() && !out.camera_info_topic.has_value()) {
    BAGWIZ_LOG_ERROR(
      kLogger,
      "point-cloud overlay requires a cam-info topic, but none could be derived from '%s'. "
      "Pass it explicitly with --cam-info.",
      args.topic.c_str());
    out.error = "point-cloud overlay requires a cam-info topic, but none could be derived from '" +
                args.topic + "'. Pass it explicitly with --cam-info.";
    return out;
  }

  return out;
}

std::string validate_video_output_path(const std::filesystem::path & output_path, bool overwrite)
{
  // Fail fast on an output collision before the expensive encode. The actual
  // removal happens just before the rename, so an existing file is only
  // replaced once the new video is fully written.
  {
    std::error_code ec;
    if (std::filesystem::exists(output_path, ec) && !overwrite) {
      BAGWIZ_LOG_ERROR(
        kLogger, "output '%s' already exists; pass -w/--overwrite to replace it.",
        output_path.string().c_str());
      return "output '" + output_path.string() +
             "' already exists; pass -w/--overwrite to replace it.";
    }
  }

  // Create the output's parent directory if needed.
  if (const auto parent = output_path.parent_path(); !parent.empty()) {
    std::error_code ec;
    std::filesystem::create_directories(parent, ec);
    if (ec) {
      BAGWIZ_LOG_ERROR(
        kLogger, "could not create output directory '%s': %s", parent.string().c_str(),
        ec.message().c_str());
      return "could not create output directory '" + parent.string() + "': " + ec.message();
    }
  }
  return "";
}

VideoInputScan scan_video_inputs(const GenerateVideoArgs & args)
{
  VideoInputScan out;

  // Derive the frame rate from the message timestamps.
  if (scan_topic_span(args.input_path, args.topic, out.span) != 0) {
    out.error = "failed to scan topic '" + args.topic + "'";
    return out;
  }
  if (out.span.count == 0) {
    BAGWIZ_LOG_ERROR(kLogger, "topic '%s' has no messages to render.", args.topic.c_str());
    out.error = "topic '" + args.topic + "' has no messages to render.";
    return out;
  }
  out.fps = core::video::derive_frame_rate(out.span.first_ns, out.span.last_ns, out.span.count);

  // Point-cloud overlay: scan timestamps and the selected property's global
  // min/max across all topics.
  if (!args.pointcloud_topics.empty()) {
    out.pcd_spans.resize(args.pointcloud_topics.size());
    out.pcd_topic_has_stamps.resize(args.pointcloud_topics.size());
    double running_min = std::numeric_limits<double>::infinity();
    double running_max = -std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < args.pointcloud_topics.size(); ++i) {
      if (
        scan_pointcloud_span(
          args.input_path, args.pointcloud_topics[i], args.property, args.property_min,
          args.property_max, out.pcd_spans[i]) != 0) {
        out.error = "failed to scan point-cloud topic '" + args.pointcloud_topics[i] + "'";
        return out;
      }
      out.pcd_topic_has_stamps[i] = out.pcd_spans[i].header_stamps_present;
      if (!args.property_min.has_value()) {
        running_min = std::min(running_min, out.pcd_spans[i].property_min);
      }
      if (!args.property_max.has_value()) {
        running_max = std::max(running_max, out.pcd_spans[i].property_max);
      }
    }
    out.global_property_min = args.property_min.value_or(running_min);
    out.global_property_max = args.property_max.value_or(running_max);
  }
  return out;
}

bool should_use_threaded_projection(
  bool has_pointcloud_topics, bool enable_threaded, std::uint64_t frame_count,
  unsigned int hardware_concurrency)
{
  return has_pointcloud_topics && enable_threaded && frame_count >= kThreadingMinFrames &&
         hardware_concurrency > 1;
}

std::string load_video_geometry(
  const GenerateVideoArgs & args, const std::optional<std::string> & camera_info_topic,
  VideoGeometry & out)
{
  if (camera_info_topic.has_value()) {
    auto ci = core::camera_info::load_camera_info(args.input_path, *camera_info_topic);
    if (!ci.ok()) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", ci.error.c_str());
      return ci.error;
    }
    out.camera_info =
      core::image::scale_camera_info(*ci.info, static_cast<double>(args.resize_scale));
  }
  if (!args.pointcloud_topics.empty()) {
    out.tf_buffer.emplace();
    if (const auto err = core::load_tf_buffer(args.input_path, *out.tf_buffer); err.has_value()) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", err->c_str());
      return *err;
    }
  }
  return "";
}

std::filesystem::path partial_tmp_path_for(const std::filesystem::path & output)
{
  return output.parent_path() /
         (output.stem().string() + ".bagwiz-partial" + output.extension().string());
}

PartialFileGuard::PartialFileGuard(std::filesystem::path tmp_path) : tmp_path_(std::move(tmp_path))
{
  // Clear any stale temp from a previous aborted run.
  std::error_code ec;
  std::filesystem::remove(tmp_path_, ec);
}

PartialFileGuard::~PartialFileGuard()
{
  std::error_code ec;
  std::filesystem::remove(tmp_path_, ec);
}

std::string finalize_video_output(
  const std::filesystem::path & tmp_path, const std::filesystem::path & output_path, bool overwrite)
{
  // Now that the new video is complete, replace any existing output and move
  // the temp into place.
  if (const auto r = core::prepare_output_path(output_path, overwrite); !r.ok) {
    BAGWIZ_LOG_ERROR(kLogger, "%s", r.error.c_str());
    return r.error;
  }
  std::error_code ec;
  std::filesystem::rename(tmp_path, output_path, ec);
  if (ec) {
    // Fall back to copy + remove across filesystems.
    std::error_code copy_ec;
    std::filesystem::copy_file(
      tmp_path, output_path, std::filesystem::copy_options::overwrite_existing, copy_ec);
    std::error_code remove_ec;
    std::filesystem::remove(tmp_path, remove_ec);
    if (copy_ec) {
      BAGWIZ_LOG_ERROR(kLogger, "could not move output into place: %s", copy_ec.message().c_str());
      return "could not move output into place: " + copy_ec.message();
    }
  }
  return "";
}

std::unique_ptr<io::BagReader> open_encode_reader(const GenerateVideoArgs & args)
{
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(args.input_path);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(
      kLogger, "failed to open '%s': %s", args.input_path.string().c_str(), e.what());
    return nullptr;
  }
  io::ReadFilter filter;
  filter.topics.push_back(args.topic);
  reader->set_filter(filter);
  return reader;
}

std::optional<FrameBuffer> FrameNormalizer::decode(
  std::int64_t timestamp_ns, std::span<const std::byte> payload, std::uint64_t frame_index) const
{
  // Normalize either message type to a canonical packed BGR24 raster via the
  // shared core::image::to_packed_raster seam; rgb8 inputs are swapped so
  // every frame the encoder sees is BGR24.
  auto pr = core::image::to_packed_raster(topic_type_, payload);
  if (!pr.ok()) {
    BAGWIZ_LOG_ERROR(kLogger, "frame %" PRIu64 ": %s", frame_index, pr.error.c_str());
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
}

bool FrameNormalizer::resize(FrameBuffer & frame) const
{
  const double scale = resize_scale_;
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
}

VideoFrameEncoder::VideoFrameEncoder(
  const std::filesystem::path & tmp_path, core::video::FrameRate fps,
  const GenerateVideoArgs & args, const core::image::CameraInfo * camera_info, double overlay_min,
  double overlay_max)
: tmp_path_(tmp_path),
  fps_(fps),
  rectify_(args.undistort || !args.pointcloud_topics.empty()),
  camera_info_(camera_info),
  overlay_min_(overlay_min),
  overlay_max_(overlay_max),
  colorscheme_(args.colorscheme),
  point_size_(args.point_size),
  alpha_(args.alpha)
{
}

bool VideoFrameEncoder::encode(
  FrameBuffer & frame, const std::vector<core::pointcloud::ProjectedPoint> * projected)
{
  std::span<const std::byte> fdata{frame.data.data(), frame.data.size()};
  std::uint32_t fstep = frame.step;

  if (encoder_ == nullptr) {
    // The first frame fixes the geometry and pixel encoding for the run.
    enc_w_ = frame.width;
    enc_h_ = frame.height;
    enc_encoding_ = frame.encoding;
    auto opened = core::video::open_video_encoder(tmp_path_, enc_w_, enc_h_, fps_.num, fps_.den);
    if (!opened.ok()) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", opened.error.c_str());
      return false;
    }
    encoder_ = std::move(opened.encoder);

    if (rectify_) {
      undistort_helper_ =
        std::make_unique<core::image::UndistortHelper>(*camera_info_, enc_w_, enc_h_);
    }
  } else if (frame.width != enc_w_ || frame.height != enc_h_ || frame.encoding != enc_encoding_) {
    BAGWIZ_LOG_ERROR(
      kLogger, "frame %" PRIu64 " changed to %ux%u %s from the first frame's %ux%u %s; aborting.",
      written_, frame.width, frame.height, frame.encoding.c_str(), enc_w_, enc_h_,
      enc_encoding_.c_str());
    return false;
  }

  if (undistort_helper_ != nullptr) {
    fdata = undistort_helper_->remap(fdata, fstep);
    fstep = enc_w_ * 3U;
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
          src, *projected, overlay_min_, overlay_max_, colorscheme_, point_size_, alpha_,
          overlay_output);
        !err.empty()) {
      BAGWIZ_LOG_ERROR(kLogger, "overlay failed: %s", err.c_str());
      return false;
    }
    fdata = std::span<const std::byte>(overlay_output.bgr.data(), overlay_output.bgr.size());
    fstep = static_cast<std::uint32_t>(row_bytes);
  }

  if (auto e = encoder_->write_frame(fdata, fstep, frame.pixel_format); !e.empty()) {
    BAGWIZ_LOG_ERROR(kLogger, "frame %" PRIu64 ": %s", written_, e.c_str());
    return false;
  }
  ++written_;
  return true;
}

std::string VideoFrameEncoder::finish()
{
  if (auto e = encoder_->finish(); !e.empty()) {
    BAGWIZ_LOG_ERROR(kLogger, "%s", e.c_str());
    encoder_.reset();
    return e;
  }
  encoder_.reset();  // close the temp file before the rename/clobber
  return "";
}

int run_encode_loop_sync(
  io::BagReader & reader, const GenerateVideoArgs & args, VideoInputScan & scan,
  const core::image::CameraInfo * camera_info, tf2::BufferCore * tf_buffer,
  const FrameNormalizer & normalizer, VideoFrameEncoder & encoder)
{
  // One cached fetcher per topic, so small bags or single-threaded runs do not
  // pay the per-frame BagReader open/close cost.
  std::vector<core::pointcloud::PointCloudFetcher> pcd_fetchers;
  if (!args.pointcloud_topics.empty()) {
    pcd_fetchers.reserve(args.pointcloud_topics.size());
    for (std::size_t i = 0; i < args.pointcloud_topics.size(); ++i) {
      pcd_fetchers.emplace_back(
        args.input_path, args.pointcloud_topics[i], std::move(scan.pcd_spans[i].entries));
    }
  }
  const bool use_rectified = args.undistort || !args.pointcloud_topics.empty();

  io::RawMessage raw;
  while (reader.next(raw)) {
    auto frame = normalizer.decode(raw.timestamp_ns, raw.payload, encoder.written());
    if (!frame) {
      return 1;
    }
    if (!normalizer.resize(*frame)) {
      return 1;
    }

    std::vector<core::pointcloud::ProjectedPoint> projected_storage;
    const std::vector<core::pointcloud::ProjectedPoint> * projected_ptr = nullptr;
    if (!pcd_fetchers.empty()) {
      for (std::size_t i = 0; i < pcd_fetchers.size(); ++i) {
        std::string pcd_error;
        const auto match = core::pointcloud::choose_frame_match(
          frame->header_stamp_ns, frame->timestamp_ns, scan.pcd_topic_has_stamps[i]);
        const auto * cloud = pcd_fetchers[i].fetch(match.target_ns, match.key, pcd_error);
        if (cloud == nullptr) {
          BAGWIZ_LOG_ERROR(kLogger, "frame %" PRIu64 ": %s", encoder.written(), pcd_error.c_str());
          return 1;
        }

        const auto projected = core::pointcloud::project_cloud_for_frame(
          *cloud, *camera_info, *tf_buffer, frame->width, frame->height, args.property,
          use_rectified);
        if (!projected.ok()) {
          BAGWIZ_LOG_ERROR(
            kLogger, "frame %" PRIu64 ": %s", encoder.written(), projected.error.c_str());
          return 1;
        }
        projected_storage.insert(
          projected_storage.end(), projected.points.begin(), projected.points.end());
      }
      projected_ptr = &projected_storage;
    }

    if (!encoder.encode(*frame, projected_ptr)) {
      return 1;
    }
  }
  return 0;
}

int run_encode_loop_async(
  io::BagReader & reader, const GenerateVideoArgs & args, VideoInputScan & scan,
  const core::image::CameraInfo & camera_info, tf2::BufferCore & tf_buffer,
  const FrameNormalizer & normalizer, VideoFrameEncoder & encoder)
{
  // The async path needs the per-topic index entries after moving them out of
  // the scan; collect them before the encode loop.
  std::vector<std::vector<core::pointcloud::PointCloudIndexEntry>> entries_per_topic;
  entries_per_topic.reserve(scan.pcd_spans.size());
  for (auto & pcd_span : scan.pcd_spans) {
    entries_per_topic.push_back(std::move(pcd_span.entries));
  }
  const bool use_rectified = args.undistort || !args.pointcloud_topics.empty();

  // Keep one frame of projection work running ahead so that fetch/parse/
  // project for frame N+1 overlaps with encoding frame N.
  auto launch_projection = [&](
                             std::int64_t header_stamp_ns, std::int64_t record_ns, std::uint32_t w,
                             std::uint32_t h) -> std::future<ProjectionWorkResult> {
    return std::async(
      std::launch::async, [&, header_stamp_ns, record_ns, w, h, rectified = use_rectified]() {
        return run_projection_work(
          args.input_path, args.pointcloud_topics, entries_per_topic, scan.pcd_topic_has_stamps,
          header_stamp_ns, record_ns, camera_info, tf_buffer, w, h, args.property, rectified);
      });
  };

  io::RawMessage raw;
  if (!reader.next(raw)) {
    BAGWIZ_LOG_ERROR(
      kLogger, "topic '%s' yielded no frames in the encode pass.", args.topic.c_str());
    return 1;
  }
  auto current = normalizer.decode(raw.timestamp_ns, raw.payload, encoder.written());
  if (!current) {
    return 1;
  }
  if (!normalizer.resize(*current)) {
    return 1;
  }
  auto pending_projection = launch_projection(
    current->header_stamp_ns, current->timestamp_ns, current->width, current->height);

  while (true) {
    auto projected = pending_projection.get();
    if (!projected.ok()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "frame %" PRIu64 ": %s", encoder.written(), projected.error.c_str());
      return 1;
    }

    io::RawMessage next_raw;
    const bool has_next = reader.next(next_raw);
    std::optional<FrameBuffer> next_frame;
    std::optional<std::future<ProjectionWorkResult>> next_projection;
    if (has_next) {
      next_frame = normalizer.decode(next_raw.timestamp_ns, next_raw.payload, encoder.written());
      if (!next_frame) {
        return 1;
      }
      if (!normalizer.resize(*next_frame)) {
        return 1;
      }
      next_projection = launch_projection(
        next_frame->header_stamp_ns, next_frame->timestamp_ns, next_frame->width,
        next_frame->height);
    }

    if (!encoder.encode(*current, &projected.points)) {
      return 1;
    }

    if (!has_next) {
      break;
    }
    current = std::move(*next_frame);
    pending_projection = std::move(*next_projection);
  }
  return 0;
}

int run_encode_pass(
  io::BagReader & reader, const GenerateVideoArgs & args, VideoInputScan & scan,
  const core::image::CameraInfo * camera_info, tf2::BufferCore * tf_buffer,
  const FrameNormalizer & normalizer, VideoFrameEncoder & encoder)
{
  try {
    if (should_use_threaded_projection(
          !args.pointcloud_topics.empty(), args.enable_threaded_projection, scan.span.count,
          std::thread::hardware_concurrency())) {
      return run_encode_loop_async(
        reader, args, scan, *camera_info, *tf_buffer, normalizer, encoder);
    }
    return run_encode_loop_sync(reader, args, scan, camera_info, tf_buffer, normalizer, encoder);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "error reading topic '%s': %s", args.topic.c_str(), e.what());
    return 1;
  }
}

std::string finish_video_encode(
  VideoFrameEncoder & encoder, const std::string & topic, const std::filesystem::path & tmp_path,
  const std::filesystem::path & output_path, bool overwrite)
{
  // Pass 1 saw messages, but if pass 2 yielded none (e.g. the bag changed
  // between passes) the encoder was never created. Nothing was rendered.
  if (!encoder.started()) {
    BAGWIZ_LOG_ERROR(kLogger, "topic '%s' yielded no frames in the encode pass.", topic.c_str());
    return "topic '" + topic + "' yielded no frames in the encode pass.";
  }
  if (const auto err = encoder.finish(); !err.empty()) {
    return err;
  }
  return finalize_video_output(tmp_path, output_path, overwrite);
}

void log_video_summary(
  const std::filesystem::path & output_path, std::uint64_t written, std::uint32_t width,
  std::uint32_t height, core::video::FrameRate fps)
{
  const double fps_value = static_cast<double>(fps.num) / static_cast<double>(fps.den);
  BAGWIZ_LOG_INFO(
    kLogger, "generate video: wrote %" PRIu64 " frame(s) to %s (%ux%u bgr8 @ %.3g fps).", written,
    output_path.string().c_str(), width, height, fps_value);

  if (is_h264_extension(output_path)) {
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
}

}  // namespace bagwiz::commands
