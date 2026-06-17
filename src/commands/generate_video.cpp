// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/generate_video.hpp"

#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/image/camera_info.hpp"
#include "bagwiz/core/image/compressed_image.hpp"
#include "bagwiz/core/image/image_decoder.hpp"
#include "bagwiz/core/image/raw_image.hpp"
#include "bagwiz/core/logging.hpp"
#include "bagwiz/core/output_path.hpp"
#include "bagwiz/core/pointcloud/color_mapper.hpp"
#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/core/pointcloud/projector.hpp"
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
constexpr const char * kCameraInfoType = "sensor_msgs/msg/CameraInfo";
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

// Per-message timestamp entry for the point-cloud topic, gathered during Pass 1.
struct PointCloudIndexEntry
{
  std::int64_t timestamp_ns = 0;
};

// Timestamps of every point-cloud message plus the global field min/max used for
// color normalization. When the user supplies --min/--max the min/max values are
// taken from the arguments and only the timestamps are scanned.
struct PointCloudSpan
{
  std::vector<PointCloudIndexEntry> entries;
  double property_min = 0.0;
  double property_max = 0.0;
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
        return "topic '" + topic + "' has type '" + t.type + "', but --cam-info requires " +
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

// Introduced in Task 6; consumed by the point-cloud overlay path in Task 7.
// Load /tf and /tf_static into a BufferCore. Returns an error string on failure.
std::optional<std::string> load_tf_buffer(
  const std::filesystem::path & input, tf2::BufferCore & buffer)
{
  constexpr const char * kTfMessageType = "tf2_msgs/msg/TFMessage";
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input);
  } catch (const std::exception & e) {
    return "failed to open '" + input.string() + "': " + e.what();
  }

  std::vector<const io::TopicInfo *> tf_topics;
  for (const auto & t : reader->topics()) {
    if (t.type == kTfMessageType) {
      tf_topics.push_back(&t);
    }
  }
  if (tf_topics.empty()) {
    return "no tf2_msgs/msg/TFMessage topics found; cannot resolve point-cloud transform";
  }

  io::ReadFilter filter;
  for (const auto * t : tf_topics) {
    filter.topics.push_back(t->name);
  }
  reader->set_filter(filter);

  std::unordered_map<std::string, std::unique_ptr<core::decoder::Decoder>> decoders;
  for (const auto * t : tf_topics) {
    auto open = core::decoder::open_decoder(*t);
    if (!open.ok()) {
      return "could not open decoder for TF topic '" + t->name + "': " + open.error;
    }
    decoders.emplace(t->name, std::move(open.decoder));
  }

  const auto is_static_tf_topic = [](std::string_view name) -> bool {
    return name.size() >= kTfStaticSuffix.size() &&
           name.compare(
             name.size() - kTfStaticSuffix.size(), kTfStaticSuffix.size(), kTfStaticSuffix) == 0;
  };

  io::RawMessage raw;
  try {
    while (reader->next(raw)) {
      auto it = decoders.find(raw.topic->name);
      if (it == decoders.end()) {
        continue;
      }
      const auto decoded = it->second->decode(raw.payload);
      if (!decoded.ok()) {
        return "failed to decode TF message on '" + raw.topic->name + "': " + decoded.error;
      }
      const auto transforms = core::extract_tf_message(*decoded.value);
      const bool is_static = is_static_tf_topic(raw.topic->name);
      for (const auto & t : transforms) {
        buffer.setTransform(t, "bagwiz", is_static);
      }
    }
  } catch (const std::exception & e) {
    return "error reading TF topics: " + std::string(e.what());
  }
  return std::nullopt;
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

const core::pointcloud::PointField * find_point_field(
  const core::pointcloud::PointCloud2 & cloud, const std::string & name)
{
  for (const auto & f : cloud.fields) {
    if (f.name == name) {
      return &f;
    }
  }
  return nullptr;
}

float read_point_field(
  const core::pointcloud::PointCloud2 & cloud, std::uint32_t point_idx, std::uint32_t offset,
  core::pointcloud::PointFieldType type)
{
  const std::byte * base = cloud.data.data() + point_idx * cloud.point_step + offset;
  switch (type) {
    case core::pointcloud::PointFieldType::kFloat32:
      return *reinterpret_cast<const float *>(base);
    case core::pointcloud::PointFieldType::kFloat64:
      return static_cast<float>(*reinterpret_cast<const double *>(base));
    case core::pointcloud::PointFieldType::kInt8:
      return static_cast<float>(*reinterpret_cast<const std::int8_t *>(base));
    case core::pointcloud::PointFieldType::kUint8:
      return static_cast<float>(*reinterpret_cast<const std::uint8_t *>(base));
    case core::pointcloud::PointFieldType::kInt16:
      return static_cast<float>(*reinterpret_cast<const std::int16_t *>(base));
    case core::pointcloud::PointFieldType::kUint16:
      return static_cast<float>(*reinterpret_cast<const std::uint16_t *>(base));
    case core::pointcloud::PointFieldType::kInt32:
      return static_cast<float>(*reinterpret_cast<const std::int32_t *>(base));
    case core::pointcloud::PointFieldType::kUint32:
      return static_cast<float>(*reinterpret_cast<const std::uint32_t *>(base));
  }
  return 0.0f;
}

// Compute the scalar value that project_pointcloud() would store for the given
// property so Pass 1's global min/max matches Pass 2's per-point values.
float compute_property_value(
  const core::pointcloud::PointCloud2 & cloud, std::uint32_t point_idx,
  const core::pointcloud::PointField * field_x, const core::pointcloud::PointField * field_y,
  const core::pointcloud::PointField * field_z,
  const core::pointcloud::PointField * field_intensity, std::uint32_t off_x, std::uint32_t off_y,
  std::uint32_t off_z, std::optional<std::uint32_t> off_intensity,
  core::pointcloud::PointCloudProperty property)
{
  const float px = read_point_field(cloud, point_idx, off_x, field_x->datatype);
  const float py = read_point_field(cloud, point_idx, off_y, field_y->datatype);
  const float pz = read_point_field(cloud, point_idx, off_z, field_z->datatype);

  switch (property) {
    case core::pointcloud::PointCloudProperty::kX:
      return px;
    case core::pointcloud::PointCloudProperty::kY:
      return py;
    case core::pointcloud::PointCloudProperty::kZ:
      return pz;
    case core::pointcloud::PointCloudProperty::kDistance:
      return std::sqrt(px * px + py * py + pz * pz);
    case core::pointcloud::PointCloudProperty::kIntensity:
      return read_point_field(cloud, point_idx, *off_intensity, field_intensity->datatype);
  }
  return 0.0f;
}

// Pass 1: scan the point-cloud topic, record every timestamp, and compute the
// global min/max of the selected property unless the user supplied --min/--max.
int scan_pointcloud_span(
  const std::filesystem::path & input, const std::string & topic,
  core::pointcloud::PointCloudProperty property, const std::optional<double> & manual_min,
  const std::optional<double> & manual_max, PointCloudSpan & out)
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

  const bool need_auto_min = !manual_min.has_value();
  const bool need_auto_max = !manual_max.has_value();
  const bool need_parse = need_auto_min || need_auto_max;

  double running_min = std::numeric_limits<double>::infinity();
  double running_max = -std::numeric_limits<double>::infinity();

  const bool need_intensity = (property == core::pointcloud::PointCloudProperty::kIntensity);

  io::RawMessage raw;
  try {
    while (reader->next(raw)) {
      PointCloudIndexEntry entry;
      entry.timestamp_ns = raw.timestamp_ns;
      out.entries.push_back(entry);

      if (!need_parse) {
        continue;
      }

      const auto parsed = core::pointcloud::parse_pointcloud2(raw.payload);
      if (!parsed.ok()) {
        BAGWIZ_LOG_ERROR(kLogger, "failed to parse point cloud: %s", parsed.error.c_str());
        return 1;
      }
      const auto & cloud = *parsed.cloud;

      const auto off_x = cloud.field_offset("x");
      const auto off_y = cloud.field_offset("y");
      const auto off_z = cloud.field_offset("z");
      if (!off_x || !off_y || !off_z) {
        BAGWIZ_LOG_ERROR(kLogger, "point cloud is missing required x/y/z fields");
        return 1;
      }
      const auto * field_x = find_point_field(cloud, "x");
      const auto * field_y = find_point_field(cloud, "y");
      const auto * field_z = find_point_field(cloud, "z");

      std::optional<std::uint32_t> off_intensity;
      const core::pointcloud::PointField * field_intensity = nullptr;
      if (need_intensity) {
        off_intensity = cloud.field_offset("intensity");
        if (!off_intensity) {
          BAGWIZ_LOG_ERROR(kLogger, "point cloud has no intensity field");
          return 1;
        }
        field_intensity = find_point_field(cloud, "intensity");
      }

      const std::uint32_t n = cloud.height * cloud.width;
      for (std::uint32_t i = 0; i < n; ++i) {
        const float value = compute_property_value(
          cloud, i, field_x, field_y, field_z, field_intensity, *off_x, *off_y, *off_z,
          off_intensity, property);
        running_min = std::min(running_min, static_cast<double>(value));
        running_max = std::max(running_max, static_cast<double>(value));
      }
    }
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "error reading point-cloud topic '%s': %s", topic.c_str(), e.what());
    return 1;
  }

  if (out.entries.empty()) {
    BAGWIZ_LOG_ERROR(kLogger, "point-cloud topic '%s' has no messages", topic.c_str());
    return 1;
  }

  out.property_min = need_auto_min ? running_min : *manual_min;
  out.property_max = need_auto_max ? running_max : *manual_max;
  return 0;
}

// Caches the most recently fetched point cloud so sequential image frames that
// map to the same cloud do not reopen the bag. Each fetch opens a fresh reader
// filtered tightly around the target timestamp.
class PointCloudFetcher
{
public:
  PointCloudFetcher(
    const std::filesystem::path & input, std::string topic,
    std::vector<PointCloudIndexEntry> entries)
  : input_(input), topic_(std::move(topic)), entries_(std::move(entries))
  {
  }

  // Returns the point cloud whose timestamp is closest to target_ns. The
  // returned pointer is valid until the next fetch() call or destruction.
  const core::pointcloud::PointCloud2 * fetch(std::int64_t target_ns, std::string & error)
  {
    if (entries_.empty()) {
      error = "no point-cloud messages available";
      return nullptr;
    }

    const std::size_t idx = find_nearest_index(target_ns);
    const std::int64_t target_ts = entries_[idx].timestamp_ns;

    if (cached_cloud_.has_value() && cached_timestamp_ns_ == target_ts) {
      return &*cached_cloud_;
    }

    auto cloud = load_at(target_ts, error);
    if (!cloud.has_value()) {
      return nullptr;
    }
    cached_cloud_ = std::move(*cloud);
    cached_timestamp_ns_ = target_ts;
    return &*cached_cloud_;
  }

private:
  std::size_t find_nearest_index(std::int64_t target_ns) const
  {
    auto it = std::lower_bound(
      entries_.begin(), entries_.end(), target_ns,
      [](const PointCloudIndexEntry & e, std::int64_t ns) { return e.timestamp_ns < ns; });

    if (it == entries_.begin()) {
      return 0;
    }
    if (it == entries_.end()) {
      return entries_.size() - 1;
    }

    const auto prev = it - 1;
    const std::int64_t prev_delta = target_ns - prev->timestamp_ns;
    const std::int64_t next_delta = it->timestamp_ns - target_ns;
    if (prev_delta <= next_delta) {
      return static_cast<std::size_t>(prev - entries_.begin());
    }
    return static_cast<std::size_t>(it - entries_.begin());
  }

  std::optional<core::pointcloud::PointCloud2> load_at(std::int64_t ts, std::string & error)
  {
    std::unique_ptr<io::BagReader> reader;
    try {
      reader = io::open_read(input_);
    } catch (const std::exception & e) {
      error = "failed to open '" + input_.string() + "': " + e.what();
      return std::nullopt;
    }

    io::ReadFilter filter;
    filter.topics.push_back(topic_);
    // Tight bracket around the exact timestamp so MCAP/SQLite can skip to the
    // right chunk while still capturing the message.
    filter.start_ns = ts - 1;
    filter.end_ns = ts + 1;
    reader->set_filter(filter);

    io::RawMessage raw;
    try {
      while (reader->next(raw)) {
        if (raw.timestamp_ns == ts) {
          const auto parsed = core::pointcloud::parse_pointcloud2(raw.payload);
          if (!parsed.ok()) {
            error = parsed.error;
            return std::nullopt;
          }
          return std::move(*parsed.cloud);
        }
      }
    } catch (const std::exception & e) {
      error = "error reading point-cloud topic '" + topic_ + "': " + e.what();
      return std::nullopt;
    }

    error = "point-cloud message at timestamp " + std::to_string(ts) + " not found";
    return std::nullopt;
  }

  const std::filesystem::path input_;
  const std::string topic_;
  const std::vector<PointCloudIndexEntry> entries_;
  std::optional<core::pointcloud::PointCloud2> cached_cloud_;
  std::int64_t cached_timestamp_ns_ = 0;
};

// Result of point-cloud transform/projection work. Kept separate from
// ProjectionResult so callers can return an error string without throwing.
struct ProjectionWorkResult
{
  std::vector<core::pointcloud::ProjectedPoint> points;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return error.empty(); }
};

// Transform the cloud into the camera frame and project it onto the image.
// This helper is shared by the synchronous and the threaded overlay paths.
// `use_rectified` should be true when the target image has been undistorted, so
// the projection aligns with the rectified image using `camera_info.p`.
ProjectionWorkResult project_cloud_for_frame(
  const core::pointcloud::PointCloud2 & cloud, const core::image::CameraInfo & camera_info,
  tf2::BufferCore & tf_buffer, std::uint32_t image_width, std::uint32_t image_height,
  core::pointcloud::PointCloudProperty property, bool use_rectified)
{
  const std::string & image_frame = camera_info.frame_id;
  const std::string & cloud_frame = cloud.frame_id;
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer.lookupTransform(image_frame, cloud_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException & e) {
    return {
      {}, std::string("cannot transform ") + cloud_frame + " -> " + image_frame + ": " + e.what()};
  }

  std::array<double, 16> transform{};
  tf2::Quaternion q(
    tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z,
    tf.transform.rotation.w);
  tf2::Matrix3x3(q).getOpenGLSubMatrix(transform.data());
  transform[12] = tf.transform.translation.x;
  transform[13] = tf.transform.translation.y;
  transform[14] = tf.transform.translation.z;

  const auto projected = core::pointcloud::project_pointcloud(
    cloud, camera_info, transform, image_width, image_height, property, use_rectified);
  if (!projected.ok()) {
    return {{}, std::move(projected.error)};
  }
  return {std::move(projected.points), {}};
}

// Fetch, parse, transform, and project the point cloud nearest to `target_ns`.
// Each call opens its own BagReader so the work can safely run on a background
// thread; the caller supplies the read-only camera info and TF buffer.
ProjectionWorkResult run_projection_work(
  const std::filesystem::path & input, const std::string & pointcloud_topic,
  const std::vector<PointCloudIndexEntry> & entries, std::int64_t target_ns,
  const core::image::CameraInfo & camera_info, tf2::BufferCore & tf_buffer,
  std::uint32_t image_width, std::uint32_t image_height,
  core::pointcloud::PointCloudProperty property, bool use_rectified)
{
  try {
    PointCloudFetcher fetcher(input, pointcloud_topic, entries);
    std::string error;
    const auto * cloud = fetcher.fetch(target_ns, error);
    if (cloud == nullptr) {
      return {{}, std::move(error)};
    }
    return project_cloud_for_frame(
      *cloud, camera_info, tf_buffer, image_width, image_height, property, use_rectified);
  } catch (const std::exception & e) {
    return {{}, std::string("point-cloud projection failed: ") + e.what()};
  }
}

// Draw projected points on top of a decoded frame. `step` is the source row
// stride in bytes. The returned cv::Mat owns the drawn buffer.
cv::Mat overlay_points(
  std::span<const std::byte> image_data, std::uint32_t width, std::uint32_t height,
  std::uint32_t step, const std::vector<core::pointcloud::ProjectedPoint> & projected,
  double property_min, double property_max, core::pointcloud::ColorScheme scheme,
  std::uint32_t point_size, float alpha)
{
  cv::Mat input(
    static_cast<int>(height), static_cast<int>(width), CV_8UC3,
    const_cast<std::byte *>(image_data.data()), step);
  cv::Mat canvas = input.clone();

  core::pointcloud::ColorMapper mapper(scheme);
  std::vector projected_sorted(projected.begin(), projected.end());
  std::sort(projected_sorted.begin(), projected_sorted.end(), [](const auto & a, const auto & b) {
    return a.depth < b.depth;
  });

  const int radius = static_cast<int>(point_size) / 2;
  const int width_i = static_cast<int>(width);
  const int height_i = static_cast<int>(height);

  auto in_bounds = [width_i, height_i, radius](const core::pointcloud::ProjectedPoint & p) -> bool {
    return p.u >= -radius && p.u < width_i + radius && p.v >= -radius && p.v < height_i + radius;
  };

  if (alpha >= 0.999f) {
    for (const auto & p : projected_sorted) {
      if (!in_bounds(p)) {
        continue;
      }
      const auto color = mapper.map(p.value, property_min, property_max);
      const cv::Scalar bgr(color[0], color[1], color[2]);
      cv::circle(canvas, {p.u, p.v}, radius, bgr, cv::FILLED);
    }
  } else {
    // Draw every point onto a single transparent overlay and blend it once.
    // Per-point copies of a full HD/4K frame are too expensive and risk memory pressure.
    cv::Mat overlay(height, width, CV_8UC3, cv::Scalar{0, 0, 0});
    for (const auto & p : projected_sorted) {
      if (!in_bounds(p)) {
        continue;
      }
      const auto color = mapper.map(p.value, property_min, property_max);
      const cv::Scalar bgr(color[0], color[1], color[2]);
      cv::circle(overlay, {p.u, p.v}, radius, bgr, cv::FILLED);
    }
    cv::Mat blended;
    cv::addWeighted(canvas, 1.0 - alpha, overlay, alpha, 0.0, blended);
    return blended;
  }
  return canvas;
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

  const bool needs_camera_info = args.undistort || args.pointcloud_topic.has_value();
  if (needs_camera_info && !camera_info_topic.has_value()) {
    BAGWIZ_LOG_ERROR(
      kLogger,
      "A camera-info topic is required for --undistort or --pcd, but none could be derived from "
      "'%s'. Pass it explicitly with --cam-info.",
      args.topic.c_str());
    return 1;
  }

  // 2b. Validate the optional point-cloud topic when one was supplied.
  std::optional<std::string> pointcloud_topic = args.pointcloud_topic;
  if (pointcloud_topic.has_value()) {
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
      if (t.name == *pointcloud_topic) {
        found = true;
        if (t.type != kPointCloudType) {
          BAGWIZ_LOG_ERROR(
            kLogger, "pcd topic '%s' has type '%s', expected %s", pointcloud_topic->c_str(),
            t.type.c_str(), kPointCloudType);
          return 1;
        }
        break;
      }
    }
    if (!found) {
      BAGWIZ_LOG_ERROR(
        kLogger, "pcd topic '%s' not found in %s", pointcloud_topic->c_str(),
        args.input_path.string().c_str());
      return 1;
    }
  }

  // 2c. Point-cloud overlay needs a camera-info topic to project into the image.
  if (pointcloud_topic.has_value() && !camera_info_topic.has_value()) {
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
  //     property's global min/max.
  PointCloudSpan pcd_span;
  if (pointcloud_topic.has_value()) {
    if (
      scan_pointcloud_span(
        args.input_path, *pointcloud_topic, args.property, args.property_min, args.property_max,
        pcd_span) != 0) {
      return 1;
    }
  }

  // 6. Load camera info if it will be used. Doing this before Pass 2 avoids
  //    aborting half-way through the encode.
  std::optional<core::image::CameraInfo> camera_info;
  if (camera_info_topic.has_value()) {
    auto ci = load_camera_info(args.input_path, *camera_info_topic);
    if (!ci.ok()) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", ci.error.c_str());
      return 1;
    }
    camera_info = core::image::scale_camera_info(*ci.info, static_cast<double>(args.resize_scale));
  }

  // 6b. Load TF data when a point-cloud overlay is requested so the cloud can be
  //     transformed into the camera frame before projection.
  std::optional<tf2::BufferCore> tf_buffer;
  if (pointcloud_topic.has_value()) {
    tf_buffer.emplace();
    if (const auto err = load_tf_buffer(args.input_path, *tf_buffer); err.has_value()) {
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

  // A CompressedImage topic carries a JPEG/PNG bitstream per message; decode it
  // to packed BGR before encoding. A raw Image topic is fed straight through.
  const bool is_compressed = (check.topic_type == kCompressedImageType);

  std::unique_ptr<core::video::VideoEncoder> encoder;
  std::unique_ptr<UndistortHelper> undistort_helper;
  std::uint32_t enc_w = 0;
  std::uint32_t enc_h = 0;
  std::string enc_encoding;
  std::uint64_t written = 0;
  const bool use_rectified = args.undistort || pointcloud_topic.has_value();

  // Owned decode buffer that survives across BagReader::next() calls, which
  // invalidate raw payload spans. Used by both the synchronous and the threaded
  // point-cloud overlay paths.
  struct FrameBuffer
  {
    std::int64_t timestamp_ns = 0;
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
    FrameBuffer frame;
    frame.timestamp_ns = timestamp_ns;
    if (is_compressed) {
      const auto cimg = core::image::extract_compressed_image(payload);
      if (!cimg.ok()) {
        BAGWIZ_LOG_ERROR(kLogger, "frame %" PRIu64 ": %s", written, cimg.error.c_str());
        return std::nullopt;
      }
      auto dec = core::image::decode_compressed_image(cimg.image->data, cimg.image->format);
      if (!dec.ok()) {
        BAGWIZ_LOG_ERROR(kLogger, "frame %" PRIu64 ": %s", written, dec.error.c_str());
        return std::nullopt;
      }
      frame.width = dec.image->width;
      frame.height = dec.image->height;
      frame.step = dec.image->width * 3U;
      frame.pixel_format = core::video::SourcePixelFormat::kBgr8;
      frame.encoding = "bgr8";
      frame.data = std::move(dec.image->bgr);
    } else {
      const auto img = core::image::extract_raw_image(payload);
      if (!img.ok()) {
        BAGWIZ_LOG_ERROR(kLogger, "frame %" PRIu64 ": %s", written, img.error.c_str());
        return std::nullopt;
      }
      const auto & v = *img.image;
      if (v.encoding == "bgr8") {
        frame.pixel_format = core::video::SourcePixelFormat::kBgr8;
      } else if (v.encoding == "rgb8") {
        frame.pixel_format = core::video::SourcePixelFormat::kRgb8;
      } else {
        BAGWIZ_LOG_ERROR(
          kLogger, "image encoding '%s' is not supported in this release; only bgr8 and rgb8.",
          v.encoding.c_str());
        return std::nullopt;
      }
      frame.width = v.width;
      frame.height = v.height;
      frame.step = v.step;
      frame.encoding = v.encoding;
      frame.data.assign(v.data.begin(), v.data.end());
    }
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

      if (args.undistort || pointcloud_topic.has_value()) {
        undistort_helper = std::make_unique<UndistortHelper>(*camera_info, enc_w, enc_h);
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

    if (projected != nullptr) {
      cv::Mat overlay_canvas = overlay_points(
        fdata, frame.width, frame.height, fstep, *projected, pcd_span.property_min,
        pcd_span.property_max, args.colorscheme, args.point_size, args.alpha);
      fdata = std::span<const std::byte>(
        reinterpret_cast<const std::byte *>(overlay_canvas.data),
        overlay_canvas.total() * overlay_canvas.elemSize());
      fstep = static_cast<std::uint32_t>(overlay_canvas.step[0]);
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
    pointcloud_topic.has_value() && args.enable_threaded_projection &&
    span.count >= kThreadingMinFrames && std::thread::hardware_concurrency() > 1;

  // The synchronous path keeps the original cached fetcher for small bags or
  // when threading is disabled; the async path opens a fresh reader per frame.
  std::unique_ptr<PointCloudFetcher> pcd_fetcher;
  if (pointcloud_topic.has_value() && !use_threaded_projection) {
    pcd_fetcher = std::make_unique<PointCloudFetcher>(
      args.input_path, *pointcloud_topic, std::move(pcd_span.entries));
  }

  io::RawMessage raw;
  try {
    if (use_threaded_projection) {
      // Async path: keep one frame of projection work running ahead so that
      // fetch/parse/project for frame N+1 overlaps with encoding frame N.
      auto launch_projection = [&](
                                 std::int64_t target_ns, std::uint32_t w,
                                 std::uint32_t h) -> std::future<ProjectionWorkResult> {
        return std::async(
          std::launch::async,
          [&, target_ns, w, h, topic = *pointcloud_topic, rectified = use_rectified]() {
            return run_projection_work(
              args.input_path, topic, pcd_span.entries, target_ns, *camera_info, *tf_buffer, w, h,
              args.property, rectified);
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
      auto pending_projection =
        launch_projection(current->timestamp_ns, current->width, current->height);

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
          next_projection =
            launch_projection(next_frame->timestamp_ns, next_frame->width, next_frame->height);
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

        const std::vector<core::pointcloud::ProjectedPoint> * projected_ptr = nullptr;
        std::vector<core::pointcloud::ProjectedPoint> projected_storage;
        if (pcd_fetcher != nullptr) {
          std::string pcd_error;
          const auto * cloud = pcd_fetcher->fetch(raw.timestamp_ns, pcd_error);
          if (cloud == nullptr) {
            BAGWIZ_LOG_ERROR(kLogger, "frame %" PRIu64 ": %s", written, pcd_error.c_str());
            encoder.reset();
            cleanup_tmp();
            return 1;
          }

          const auto projected = project_cloud_for_frame(
            *cloud, *camera_info, *tf_buffer, frame->width, frame->height, args.property,
            use_rectified);
          if (!projected.ok()) {
            BAGWIZ_LOG_ERROR(kLogger, "frame %" PRIu64 ": %s", written, projected.error.c_str());
            encoder.reset();
            cleanup_tmp();
            return 1;
          }
          projected_storage = std::move(projected.points);
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
