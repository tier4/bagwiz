// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/map_slam.hpp"

#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/base/output_path.hpp"
#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/image/camera_info_resolver.hpp"
#include "bagwiz/core/image/packed_raster.hpp"
#include "bagwiz/core/pointcloud/color_propagation.hpp"
#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/core/slam/cloud_mapper.hpp"
#include "bagwiz/core/slam/cuda_device.hpp"
#include "bagwiz/core/slam/gnss_projector.hpp"
#include "bagwiz/core/slam/gnss_sample.hpp"
#include "bagwiz/core/slam/imu_sample.hpp"
#include "bagwiz/core/slam/lidar_scan.hpp"
#include "bagwiz/core/slam/map_colorizer.hpp"
#include "bagwiz/core/slam/map_viewer.hpp"
#include "bagwiz/core/slam/progress_bar.hpp"
#include "bagwiz/core/slam/propagation_radius.hpp"
#include "bagwiz/core/slam/scan_image_pairer.hpp"
#include "bagwiz/core/slam/scan_to_world.hpp"
#include "bagwiz/core/slam/sensor_transform.hpp"
#include "bagwiz/core/tf/tf_chain.hpp"
#include "bagwiz/core/tf/tf_topics.hpp"
#include "bagwiz/core/tf/tf_value_extract.hpp"
#include "bagwiz/core/tf/trajectory.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/bag_open.hpp"
#include "map_slam_colorize.hpp"  // NOLINT(build/include_subdir) src-local shared header
#include "map_slam_mapping.hpp"   // NOLINT(build/include_subdir) src-local shared header
#include "map_slam_threads.hpp"   // NOLINT(build/include_subdir) src-local shared header

#include <tf2/buffer_core.hpp>
#include <tf2/time.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cinttypes>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <exception>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bagwiz::commands
{
namespace
{
constexpr const char * kLogger = "bagwiz.cmd.map";
constexpr const char * kPointCloud2Type = "sensor_msgs/msg/PointCloud2";
constexpr const char * kImuType = "sensor_msgs/msg/Imu";
constexpr const char * kNavSatFixType = "sensor_msgs/msg/NavSatFix";
constexpr const char * kTfMessageType = "tf2_msgs/msg/TFMessage";
// Static transforms are timeless; a year-long cache dwarfs any bag and matches
// `tf walk` / `tf static calc` buffer sizing.
constexpr std::chrono::hours kTfBufferCacheTime{24 * 365};

// tf2's lookupTransform(target=cloud, source=imu) yields p_cloud = T * p_imu,
// which is exactly GLIM's T_lidar_imu (p_lidar = T_lidar_imu * p_imu).
core::slam::SensorTransform to_sensor_transform(const geometry_msgs::msg::TransformStamped & ts)
{
  core::slam::SensorTransform out;
  out.translation = {
    ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z};
  out.rotation_xyzw = {
    ts.transform.rotation.x, ts.transform.rotation.y, ts.transform.rotation.z,
    ts.transform.rotation.w};
  return out;
}

// One camera image dispatched to a colorize worker thread: the undecoded
// message plus the world points of its paired scan. The scan points are OWNED
// here (copied out of the pairer's decision), because the pairer recycles its
// scan slot storage on the next push_scan while the worker may still be
// queued behind earlier images.
struct ColorizeWorkItem
{
  std::int64_t stamp_ns = 0;
  std::string type;
  std::vector<std::byte> payload;
  std::vector<std::array<float, 3>> dynamic_points;
};

// Bounded blocking queue for the camera-parallel colorize pipeline: push()
// blocks while `capacity` items are outstanding (backpressure toward the bag
// reader, so a slow camera cannot balloon memory), and close() lets consumers
// drain what remains — pop() returns false only once the queue is closed AND
// empty.
class ColorizeWorkQueue
{
public:
  explicit ColorizeWorkQueue(std::size_t capacity) : capacity_(capacity) {}

  void push(ColorizeWorkItem item)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    not_full_.wait(lock, [&]() { return closed_ || items_.size() < capacity_; });
    if (closed_) {
      return;
    }
    items_.push_back(std::move(item));
    not_empty_.notify_one();
  }

  bool pop(ColorizeWorkItem & item)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    not_empty_.wait(lock, [&]() { return closed_ || !items_.empty(); });
    if (items_.empty()) {
      return false;
    }
    item = std::move(items_.front());
    items_.pop_front();
    not_full_.notify_one();
    return true;
  }

  void close()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    closed_ = true;
    not_full_.notify_all();
    not_empty_.notify_all();
  }

private:
  std::size_t capacity_;
  std::mutex mutex_;
  std::condition_variable not_full_;
  std::condition_variable not_empty_;
  std::deque<ColorizeWorkItem> items_;
  bool closed_ = false;
};

// Drives a single `bagwiz map slam` invocation. Holds the parsed arguments plus
// the output paths derived from output_root, and owns the bag reading + GLIM
// feeding. One instance per run().
class MapSlamRunner
{
public:
  explicit MapSlamRunner(const MapSlamArgs & args) : args_(args) {}

  int run()
  {
    // Cross-field numeric validation the per-option CLI checks can't express, run
    // before any bag work. Each of --input-res / --min-range / --max-range is
    // already CLI-checked > 0 and --fill-min-inliers is range-checked to
    // [0, 1]; here we enforce the relations between them.
    if (!(args_.range_min < args_.range_max)) {
      BAGWIZ_LOG_ERROR(
        kLogger, "--min-range (%g) must be strictly less than --max-range (%g).", args_.range_min,
        args_.range_max);
      return 1;
    }
    if (!(args_.fill_min_inlier_fraction > 0.0)) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "--fill-min-inliers must be > 0 (got %g); 0 would disable the fill acceptance gate "
        "entirely.",
        args_.fill_min_inlier_fraction);
      return 1;
    }

    // Resolve the effective backend (CPU/GPU) from --backend plus a CUDA device
    // probe, before any bag work. A forced 'cuda' that cannot run errors here;
    // 'auto' degrades to CPU.
    if (!resolve_backend()) {
      return 1;
    }
    if (use_gpu_ && args_.imu_topic.empty()) {
      BAGWIZ_LOG_INFO(
        kLogger,
        "GPU backend without --imu: odometry runs on CPU (CT; GLIM has no GPU LiDAR-only "
        "backend); GPU acceleration applies to mapping registration and export voxelization.");
    }

    auto reader = io::open_read_or_log(args_.input_path, kLogger);
    if (!reader) {
      return 1;
    }

    if (!topic_present_with_type(*reader, args_.cloud_topic, kPointCloud2Type)) {
      return 1;
    }
    if (!args_.imu_topic.empty() && !topic_present_with_type(*reader, args_.imu_topic, kImuType)) {
      return 1;
    }
    if (!args_.gnss_topic.empty()) {
      if (!topic_present_with_type(*reader, args_.gnss_topic, kNavSatFixType)) {
        return 1;
      }
    }
    if (!args_.image_topics.empty() && !validate_camera_inputs(*reader)) {
      return 1;
    }

    // Validate / create the output root before any heavy work. A file at the
    // path is an error; an existing directory is accepted so the user can target a
    // project folder, and individual output files are guarded by prepare_output_path.
    {
      std::error_code ec;
      if (std::filesystem::exists(args_.output_root, ec)) {
        if (!std::filesystem::is_directory(args_.output_root, ec)) {
          BAGWIZ_LOG_ERROR(
            kLogger, "Output path '%s' exists but is not a directory", args_.output_root.c_str());
          return 1;
        }
      } else {
        if (!std::filesystem::create_directories(args_.output_root, ec)) {
          BAGWIZ_LOG_ERROR(
            kLogger, "Could not create output root '%s': %s", args_.output_root.c_str(),
            ec.message().c_str());
          return 1;
        }
      }
    }

    output_path_ = args_.output_root / "traj.tum";
    const auto prepared = core::prepare_output_path(output_path_, args_.overwrite);
    if (!prepared.ok) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", prepared.error.c_str());
      return 1;
    }

    // Derive and guard the map output path.
    map_path_ = args_.output_root / "map.pcd";
    const auto prepared_map = core::prepare_output_path(map_path_, args_.overwrite);
    if (!prepared_map.ok) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", prepared_map.error.c_str());
      return 1;
    }

    // Resolve the LiDAR<-IMU extrinsic from the bag's static TF before feeding
    // GLIM (the estimator needs T_lidar_imu at construction). Errors here abort
    // before any output is written.
    std::optional<core::slam::SensorTransform> t_lidar_imu;
    if (!args_.imu_topic.empty()) {
      core::slam::SensorTransform extrinsic;
      if (!resolve_extrinsic(extrinsic)) {
        return 1;
      }
      t_lidar_imu = extrinsic;
    }

    // Resolve every cloud<-camera extrinsic before feeding GLIM so an absent
    // TF chain aborts before hours of SLAM, not after. The colorization
    // itself runs after the global optimization.
    if (!args_.image_topics.empty() && !resolve_camera_extrinsics()) {
      return 1;
    }

    return run_mapping(*reader, t_lidar_imu);
  }

private:
  // Verify `topic` exists in the bag and has the expected type (both errors are
  // logged). Returns false on any mismatch.
  bool topic_present_with_type(
    io::BagReader & reader, const std::string & topic, const char * expected_type)
  {
    const io::TopicInfo * info = nullptr;
    for (const auto & t : reader.topics()) {
      if (t.name == topic) {
        info = &t;
        break;
      }
    }
    if (info == nullptr) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Topic '%s' is not present in %s", topic.c_str(), args_.input_path.c_str());
      return false;
    }
    if (info->type != expected_type) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Topic '%s' is %s, expected %s", topic.c_str(), info->type.c_str(), expected_type);
      return false;
    }
    return true;
  }

  // First decodable header.frame_id of the cloud topic. Empty on failure.
  bool peek_cloud_frame(std::string & cloud_frame)
  {
    std::unique_ptr<io::BagReader> reader;
    try {
      reader = io::open_read(args_.input_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to reopen %s: %s", args_.input_path.c_str(), e.what());
      return false;
    }
    io::ReadFilter filter;
    filter.topics.push_back(args_.cloud_topic);
    reader->set_filter(filter);

    std::int64_t cloud_fail = 0;
    io::RawMessage raw;
    while (cloud_frame.empty() && reader->next(raw)) {
      if (raw.topic->name == args_.cloud_topic) {
        const auto parsed = core::pointcloud::parse_pointcloud2(raw.payload);
        if (parsed.ok()) {
          cloud_frame = parsed.cloud->frame_id;
        } else {
          ++cloud_fail;
        }
      }
    }
    if (cloud_frame.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Could not read a PointCloud2 frame_id from '%s' (%s message(s) failed to parse)",
        args_.cloud_topic.c_str(), std::to_string(cloud_fail).c_str());
      return false;
    }
    return true;
  }

  // Validate every --cam image topic and resolve + load its CameraInfo (into
  // camera_info_topics_ / camera_infos_, parallel to args_.image_topics).
  // Errors are logged; false aborts before any heavy work.
  bool validate_camera_inputs(io::BagReader & reader)
  {
    if (
      !args_.camera_info_topics.empty() &&
      args_.camera_info_topics.size() != args_.image_topics.size()) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "--cam-info was given %zu time(s) for %zu --cam topic(s); pass exactly one --cam-info "
        "per --cam (in the same order), or none to auto-resolve all of them.",
        args_.camera_info_topics.size(), args_.image_topics.size());
      return false;
    }

    for (std::size_t cam = 0; cam < args_.image_topics.size(); ++cam) {
      const std::string & image_topic = args_.image_topics[cam];
      for (std::size_t prev = 0; prev < cam; ++prev) {
        if (args_.image_topics[prev] == image_topic) {
          BAGWIZ_LOG_ERROR(
            kLogger, "--cam topic '%s' was given more than once.", image_topic.c_str());
          return false;
        }
      }

      const io::TopicInfo * info = nullptr;
      for (const auto & t : reader.topics()) {
        if (t.name == image_topic) {
          info = &t;
          break;
        }
      }
      if (info == nullptr) {
        BAGWIZ_LOG_ERROR(
          kLogger, "Topic '%s' is not present in %s", image_topic.c_str(),
          args_.input_path.c_str());
        return false;
      }
      // Gate on the shared to_packed_raster() decoder's type set — the same
      // check `walk`'s image preview uses — so --cam and the preview can never
      // drift apart in what they accept.
      if (!core::image::is_supported_image_type(info->type)) {
        BAGWIZ_LOG_ERROR(
          kLogger,
          "Topic '%s' is %s, which map slam --cam cannot decode; supported types are "
          "sensor_msgs/msg/Image and sensor_msgs/msg/CompressedImage.",
          image_topic.c_str(), info->type.c_str());
        return false;
      }

      std::string camera_info_topic;
      if (!args_.camera_info_topics.empty()) {
        camera_info_topic = args_.camera_info_topics[cam];
        const auto error =
          core::camera_info::validate_camera_info_topic(args_.input_path, camera_info_topic);
        if (error.has_value()) {
          BAGWIZ_LOG_ERROR(kLogger, "%s", error->c_str());
          return false;
        }
      } else {
        const auto resolved =
          core::camera_info::resolve_camera_info_topic(image_topic, reader.topics());
        if (!resolved.topic.has_value()) {
          BAGWIZ_LOG_ERROR(
            kLogger,
            "Could not auto-resolve a CameraInfo topic for '%s'%s%s. Pass it explicitly "
            "with --cam-info.",
            image_topic.c_str(), resolved.error.has_value() ? ": " : "",
            resolved.error.has_value() ? resolved.error->c_str() : "");
          return false;
        }
        camera_info_topic = *resolved.topic;
      }

      const auto loaded = core::camera_info::load_camera_info(args_.input_path, camera_info_topic);
      if (!loaded.ok()) {
        BAGWIZ_LOG_ERROR(
          kLogger, "Could not read CameraInfo from '%s': %s", camera_info_topic.c_str(),
          loaded.error.c_str());
        return false;
      }
      if (!(loaded.info->k[0] > 0.0) || !(loaded.info->k[4] > 0.0)) {
        BAGWIZ_LOG_ERROR(
          kLogger,
          "CameraInfo on '%s' has a degenerate intrinsic matrix (fx=%g, fy=%g); cannot project "
          "the map for colorization.",
          camera_info_topic.c_str(), loaded.info->k[0], loaded.info->k[4]);
        return false;
      }
      camera_info_topics_.push_back(camera_info_topic);
      camera_infos_.push_back(*loaded.info);
    }
    return true;
  }

  // Resolve T_cloud_cam (cloud frame <- camera optical frame) for every --cam
  // from the bag's static TF, into t_cloud_cams_ (parallel to image_topics).
  // Mirrors resolve_extrinsic: --cam is an explicit request, so any failure is
  // fatal rather than silently writing an uncolored map. The cloud frame and
  // the static TF buffer are resolved once and shared across cameras.
  bool resolve_camera_extrinsics()
  {
    std::string cloud_frame;
    if (!peek_cloud_frame(cloud_frame)) {
      return false;
    }

    // Build the static TF buffer lazily: an all-identity setup (every camera
    // sharing the cloud frame) needs no TF topic at all.
    std::optional<tf2::BufferCore> buffer;
    for (std::size_t cam = 0; cam < camera_infos_.size(); ++cam) {
      const std::string & cam_frame = camera_infos_[cam].frame_id;
      if (cam_frame.empty()) {
        BAGWIZ_LOG_ERROR(
          kLogger,
          "CameraInfo on '%s' has an empty header.frame_id; cannot resolve the camera "
          "extrinsic from the bag's static TF.",
          camera_info_topics_[cam].c_str());
        return false;
      }
      if (cloud_frame == cam_frame) {
        t_cloud_cams_.push_back(core::slam::SensorTransform{});
        continue;
      }

      if (!buffer.has_value()) {
        buffer.emplace(kTfBufferCacheTime);
        if (!build_static_tf_buffer(*buffer, "the camera extrinsic")) {
          return false;
        }
      }

      const auto missing = core::missing_frames(*buffer, cloud_frame, cam_frame);
      if (!missing.empty()) {
        std::string names;
        for (std::size_t i = 0; i < missing.size(); ++i) {
          names += (i ? ", " : "") + missing[i];
        }
        BAGWIZ_LOG_ERROR(
          kLogger, "Frame(s) not present in the bag's static TF tree: %s", names.c_str());
        return false;
      }

      try {
        const auto ts = buffer->lookupTransform(cloud_frame, cam_frame, tf2::TimePointZero);
        t_cloud_cams_.push_back(to_sensor_transform(ts));
      } catch (const std::exception & e) {
        BAGWIZ_LOG_ERROR(
          kLogger, "No static TF chain from '%s' to '%s': %s", cloud_frame.c_str(),
          cam_frame.c_str(), e.what());
        return false;
      }
    }
    return true;
  }

  // First decodable header.frame_id of the cloud and IMU topics, captured in a
  // single bounded pass (stops once both are known). Empty strings on failure.
  bool peek_frames(std::string & cloud_frame, std::string & imu_frame)
  {
    std::unique_ptr<io::BagReader> reader;
    try {
      reader = io::open_read(args_.input_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to reopen %s: %s", args_.input_path.c_str(), e.what());
      return false;
    }
    io::ReadFilter filter;
    filter.topics.push_back(args_.cloud_topic);
    filter.topics.push_back(args_.imu_topic);
    reader->set_filter(filter);

    std::int64_t cloud_fail = 0;
    std::int64_t imu_fail = 0;
    io::RawMessage raw;
    while ((cloud_frame.empty() || imu_frame.empty()) && reader->next(raw)) {
      if (cloud_frame.empty() && raw.topic->name == args_.cloud_topic) {
        const auto parsed = core::pointcloud::parse_pointcloud2(raw.payload);
        if (parsed.ok()) {
          cloud_frame = parsed.cloud->frame_id;
        } else {
          ++cloud_fail;
        }
      } else if (imu_frame.empty() && raw.topic->name == args_.imu_topic) {
        const auto parsed = core::slam::parse_imu(raw.payload);
        if (parsed.ok()) {
          imu_frame = parsed.sample->frame_id;
        } else {
          ++imu_fail;
        }
      }
    }
    // On exhaustion the count of failed parses points at a schema/format mismatch
    // rather than an empty topic.
    if (cloud_frame.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Could not read a PointCloud2 frame_id from '%s' (%s message(s) failed to parse)",
        args_.cloud_topic.c_str(), std::to_string(cloud_fail).c_str());
      return false;
    }
    if (imu_frame.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Could not read an Imu frame_id from '%s' (%s message(s) failed to parse)",
        args_.imu_topic.c_str(), std::to_string(imu_fail).c_str());
      return false;
    }
    return true;
  }

  // Build a tf2 buffer from every static TF topic in the bag. Returns false (and
  // logs) when the bag has no static TF topic or a TF message fails to decode.
  // `purpose` is included in the "no static TF topic" error so the message matches
  // the caller's context (IMU extrinsic, GNSS lever-arm, output-frame remap, ...).
  bool build_static_tf_buffer(tf2::BufferCore & buffer, std::string_view purpose)
  {
    std::unique_ptr<io::BagReader> reader;
    try {
      reader = io::open_read(args_.input_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to reopen %s: %s", args_.input_path.c_str(), e.what());
      return false;
    }

    std::vector<std::string> static_topics;
    for (const auto & t : reader->topics()) {
      if (t.type == kTfMessageType && core::is_static_tf_topic(t.name)) {
        static_topics.push_back(t.name);
      }
    }
    if (static_topics.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "Bag has no static TF topic (…tf_static); cannot resolve %.*s. "
        "Provide a bag whose /tf_static contains the needed transforms.",
        static_cast<int>(purpose.size()), purpose.data());
      return false;
    }

    io::ReadFilter filter;
    filter.topics = static_topics;
    reader->set_filter(filter);

    std::unordered_map<std::string, std::unique_ptr<core::decoder::Decoder>> decoders;
    for (const auto & info : reader->topics()) {
      if (info.type != kTfMessageType || !core::is_static_tf_topic(info.name)) {
        continue;
      }
      auto open = core::decoder::open_decoder(info);
      if (!open.ok()) {
        BAGWIZ_LOG_ERROR(
          kLogger, "Could not open decoder for static TF topic '%s': %s", info.name.c_str(),
          open.error.c_str());
        return false;
      }
      decoders.emplace(info.name, std::move(open.decoder));
    }

    io::RawMessage raw;
    try {
      while (reader->next(raw)) {
        const auto it = decoders.find(raw.topic->name);
        if (it == decoders.end()) {
          continue;
        }
        const auto decoded = it->second->decode(raw.payload);
        if (!decoded.ok()) {
          BAGWIZ_LOG_ERROR(
            kLogger, "Failed to decode static TF on '%s': %s", raw.topic->name.c_str(),
            decoded.error.c_str());
          return false;
        }
        for (const auto & t : core::extract_tf_message(*decoded.value)) {
          buffer.setTransform(t, "bagwiz", true);
        }
      }
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Error reading static TF: %s", e.what());
      return false;
    }
    return true;
  }

  // Resolve T_lidar_imu (cloud frame <- imu frame) from the bag's static TF.
  // Returns false (logged) on any failure.
  bool resolve_extrinsic(core::slam::SensorTransform & out)
  {
    std::string cloud_frame;
    std::string imu_frame;
    if (!peek_frames(cloud_frame, imu_frame)) {
      return false;
    }

    // Same frame for cloud and IMU (e.g. an already-base_link IMU): the extrinsic
    // is identity regardless of whether that frame is a TF node.
    if (cloud_frame == imu_frame) {
      out = core::slam::SensorTransform{};
      return true;
    }

    tf2::BufferCore buffer{kTfBufferCacheTime};
    if (!build_static_tf_buffer(buffer, "the LiDAR<-IMU extrinsic")) {
      return false;
    }

    const auto missing = core::missing_frames(buffer, cloud_frame, imu_frame);
    if (!missing.empty()) {
      std::string names;
      for (std::size_t i = 0; i < missing.size(); ++i) {
        names += (i ? ", " : "") + missing[i];
      }
      BAGWIZ_LOG_ERROR(
        kLogger, "Frame(s) not present in the bag's static TF tree: %s", names.c_str());
      return false;
    }

    try {
      const auto ts = buffer.lookupTransform(cloud_frame, imu_frame, tf2::TimePointZero);
      out = to_sensor_transform(ts);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(
        kLogger, "No static TF chain from '%s' to '%s': %s", cloud_frame.c_str(), imu_frame.c_str(),
        e.what());
      return false;
    }

    return true;
  }

  // First decodable header.frame_id of the cloud and GNSS topics in one bounded
  // pass. Returns false only on a reopen failure; a topic that never decodes
  // leaves its frame empty for the caller to handle.
  bool peek_cloud_and_gnss_frames(std::string & cloud_frame, std::string & gnss_frame)
  {
    std::unique_ptr<io::BagReader> reader;
    try {
      reader = io::open_read(args_.input_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_WARN(
        kLogger, "Could not reopen %s to peek the GNSS frame: %s", args_.input_path.c_str(),
        e.what());
      return false;
    }
    io::ReadFilter filter;
    filter.topics.push_back(args_.cloud_topic);
    filter.topics.push_back(args_.gnss_topic);
    reader->set_filter(filter);

    io::RawMessage raw;
    while ((cloud_frame.empty() || gnss_frame.empty()) && reader->next(raw)) {
      if (cloud_frame.empty() && raw.topic->name == args_.cloud_topic) {
        const auto parsed = core::pointcloud::parse_pointcloud2(raw.payload);
        if (parsed.ok()) {
          cloud_frame = parsed.cloud->frame_id;
        }
      } else if (gnss_frame.empty() && raw.topic->name == args_.gnss_topic) {
        const auto parsed = core::slam::parse_navsatfix(raw.payload);
        if (parsed.ok()) {
          gnss_frame = parsed.sample->frame_id;
        }
      }
    }
    return true;
  }

  // True if the bag carries at least one static TF topic (…tf_static). Lets the
  // GNSS lever-arm resolution skip build_static_tf_buffer's hard-error path (which
  // is fatal for the IMU extrinsic) and degrade gracefully instead.
  bool bag_has_static_tf()
  {
    std::unique_ptr<io::BagReader> reader;
    try {
      reader = io::open_read(args_.input_path);
    } catch (const std::exception &) {
      return false;
    }
    for (const auto & t : reader->topics()) {
      if (t.type == kTfMessageType && core::is_static_tf_topic(t.name)) {
        return true;
      }
    }
    return false;
  }

  // Resolve the GNSS antenna lever-arm T_cloud_gnss.translation() from the bag's
  // static TF (cloud frame <- NavSatFix frame_id). Unlike the IMU extrinsic this
  // is NON-FATAL: GNSS still works without it, just uncorrected, so every failure
  // path logs a warning and returns {0,0,0} (no correction) rather than aborting.
  std::array<double, 3> resolve_gnss_offset()
  {
    const std::array<double, 3> kZero{0.0, 0.0, 0.0};

    std::string cloud_frame;
    std::string gnss_frame;
    if (!peek_cloud_and_gnss_frames(cloud_frame, gnss_frame) || cloud_frame.empty()) {
      BAGWIZ_LOG_WARN(
        kLogger,
        "Could not read cloud/GNSS frame_ids; GNSS constraints use the raw antenna position "
        "(no lever-arm correction).");
      return kZero;
    }
    if (gnss_frame.empty()) {
      BAGWIZ_LOG_WARN(
        kLogger,
        "NavSatFix on '%s' has an empty header.frame_id; cannot resolve the antenna lever-arm "
        "from static TF — GNSS constraints use the raw antenna position.",
        args_.gnss_topic.c_str());
      return kZero;
    }
    if (cloud_frame == gnss_frame) {
      BAGWIZ_LOG_INFO(
        kLogger, "GNSS and cloud share frame '%s'; antenna lever-arm is zero", cloud_frame.c_str());
      return kZero;
    }
    if (!bag_has_static_tf()) {
      BAGWIZ_LOG_WARN(
        kLogger,
        "Bag has no static TF (…tf_static) to resolve the GNSS antenna lever-arm ('%s' <- '%s'); "
        "GNSS constraints use the raw antenna position.",
        cloud_frame.c_str(), gnss_frame.c_str());
      return kZero;
    }

    tf2::BufferCore buffer{kTfBufferCacheTime};
    if (!build_static_tf_buffer(buffer, "the GNSS antenna lever-arm")) {
      BAGWIZ_LOG_WARN(
        kLogger,
        "Could not read static TF for the GNSS antenna lever-arm; GNSS constraints use the raw "
        "antenna position.");
      return kZero;
    }
    const auto missing = core::missing_frames(buffer, cloud_frame, gnss_frame);
    if (!missing.empty()) {
      std::string names;
      for (std::size_t i = 0; i < missing.size(); ++i) {
        names += (i ? ", " : "") + missing[i];
      }
      BAGWIZ_LOG_WARN(
        kLogger,
        "GNSS frame(s) absent from the bag's static TF tree: %s; GNSS constraints use the raw "
        "antenna position.",
        names.c_str());
      return kZero;
    }

    try {
      const auto ts = buffer.lookupTransform(cloud_frame, gnss_frame, tf2::TimePointZero);
      const auto t = to_sensor_transform(ts);
      BAGWIZ_LOG_INFO(
        kLogger,
        "Resolved GNSS antenna lever-arm from static TF ('%s' <- '%s'): "
        "(%.3f, %.3f, %.3f) m",
        cloud_frame.c_str(), gnss_frame.c_str(), t.translation[0], t.translation[1],
        t.translation[2]);
      return t.translation;
    } catch (const std::exception & e) {
      BAGWIZ_LOG_WARN(
        kLogger,
        "No static TF chain from '%s' to '%s' (%s); GNSS constraints use the raw antenna position.",
        cloud_frame.c_str(), gnss_frame.c_str(), e.what());
      return kZero;
    }
  }

  // Read the cloud (and, in IMU mode, IMU) topic in log order, dispatching each
  // message by type. Returns false on a fatal read error or when no scan decoded
  // (both logged); otherwise fills the counters.
  template <typename ScanFn, typename ImuFn, typename GnssFn>
  bool process_messages(
    io::BagReader & reader, ScanFn && on_scan, ImuFn && on_imu, GnssFn && on_gnss,
    core::slam::ScanProgress & progress, std::int64_t & scans, std::int64_t & skipped,
    std::int64_t & imu_count, std::int64_t & gnss_count)
  {
    io::ReadFilter filter;
    filter.topics.push_back(args_.cloud_topic);
    if (!args_.imu_topic.empty()) {
      filter.topics.push_back(args_.imu_topic);
    }
    if (!args_.gnss_topic.empty()) {
      filter.topics.push_back(args_.gnss_topic);
    }
    reader.set_filter(filter);

    io::RawMessage raw;
    // Every next() returns a message on one of the filtered topics, so this
    // counter tracks bag-read progress against the summed per-topic total.
    std::int64_t processed = 0;
    try {
      while (reader.next(raw)) {
        ++processed;
        if (raw.topic->name == args_.cloud_topic) {
          const auto parsed = core::pointcloud::parse_pointcloud2(raw.payload);
          if (!parsed.ok()) {
            ++skipped;
            continue;
          }
          const auto scan = core::slam::to_lidar_scan(*parsed.cloud);
          if (!scan.ok()) {
            ++skipped;
            continue;
          }
          on_scan(*scan.scan);
          ++scans;
        } else if (!args_.imu_topic.empty() && raw.topic->name == args_.imu_topic) {
          const auto parsed = core::slam::parse_imu(raw.payload);
          if (!parsed.ok()) {
            continue;  // a malformed IMU sample is dropped, not fatal
          }
          on_imu(*parsed.sample);
          ++imu_count;
        } else if (!args_.gnss_topic.empty() && raw.topic->name == args_.gnss_topic) {
          const auto parsed = core::slam::parse_navsatfix(raw.payload);
          if (!parsed.ok()) {
            continue;  // a malformed GNSS sample is dropped, not fatal
          }
          // Drop fixes with no satellite lock — they carry no usable position.
          if (parsed.sample->status == core::slam::kNavSatStatusNoFix) {
            continue;
          }
          on_gnss(*parsed.sample);
          ++gnss_count;
        }
        progress.update(processed, scans);
      }
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(
        kLogger, "read error after %s scans: %s", std::to_string(scans).c_str(), e.what());
      return false;
    }
    if (scans == 0) {
      BAGWIZ_LOG_ERROR(
        kLogger, "No decodable PointCloud2 messages on '%s'", args_.cloud_topic.c_str());
      return false;
    }
    return true;
  }

  // Convert a TrajectoryPose to the geometry_msgs Pose representation used by
  // core::compose_trajectory_pose.
  static geometry_msgs::msg::Pose to_geometry_pose(const core::TrajectoryPose & p)
  {
    geometry_msgs::msg::Pose out;
    out.position.x = p.tx;
    out.position.y = p.ty;
    out.position.z = p.tz;
    out.orientation.x = p.qx;
    out.orientation.y = p.qy;
    out.orientation.z = p.qz;
    out.orientation.w = p.qw;
    return out;
  }

  // Convert a geometry_msgs Pose back to a TrajectoryPose, preserving the stamp.
  static core::TrajectoryPose to_trajectory_pose(
    const geometry_msgs::msg::Pose & p, std::int64_t stamp_ns)
  {
    core::TrajectoryPose out;
    out.timestamp_ns = stamp_ns;
    out.tx = p.position.x;
    out.ty = p.position.y;
    out.tz = p.position.z;
    out.qx = p.orientation.x;
    out.qy = p.orientation.y;
    out.qz = p.orientation.z;
    out.qw = p.orientation.w;
    return out;
  }

  // Apply a cloud-frame -> output-frame static transform to every pose. The
  // incoming poses are T_world_cloud; right-multiplying by T_cloud_output yields
  // T_world_output, i.e. the requested frame's pose in the SLAM world.
  void transform_trajectory_to_frame(
    std::vector<core::TrajectoryPose> & poses, const geometry_msgs::msg::Transform & t_cloud_output)
  {
    for (auto & p : poses) {
      const auto remapped =
        core::compose_trajectory_pose(std::nullopt, to_geometry_pose(p), t_cloud_output);
      p = to_trajectory_pose(remapped, p.timestamp_ns);
    }
  }

  // Resolve the optional --frame remapping. Returns true when no remapping is
  // requested or when the transform was found. On failure logs and returns false.
  // On success, `cloud_frame` holds the PointCloud2 frame_id and `body_to` is
  // set to the cloud-frame -> output-frame transform when remapping is needed.
  bool resolve_output_transform(
    std::string & cloud_frame, std::optional<geometry_msgs::msg::Transform> & body_to)
  {
    body_to = std::nullopt;
    if (args_.output_frame.empty()) {
      return true;
    }

    if (!peek_cloud_frame(cloud_frame)) {
      return false;
    }

    if (args_.output_frame == cloud_frame) {
      BAGWIZ_LOG_INFO(
        kLogger, "Output frame '%s' matches the cloud frame; trajectory is not remapped.",
        args_.output_frame.c_str());
      return true;
    }

    tf2::BufferCore buffer{kTfBufferCacheTime};
    if (!build_static_tf_buffer(buffer, "the output-frame remap")) {
      return false;
    }

    const auto missing = core::missing_frames(buffer, cloud_frame, args_.output_frame);
    if (!missing.empty()) {
      std::string names;
      for (std::size_t i = 0; i < missing.size(); ++i) {
        names += (i ? ", " : "") + missing[i];
      }
      BAGWIZ_LOG_ERROR(
        kLogger, "Frame(s) not present in the bag's static TF tree: %s", names.c_str());
      return false;
    }

    try {
      const auto ts = buffer.lookupTransform(cloud_frame, args_.output_frame, tf2::TimePointZero);
      body_to = ts.transform;
      BAGWIZ_LOG_INFO(
        kLogger, "Remapping trajectory from cloud frame '%s' to output frame '%s' using static TF.",
        cloud_frame.c_str(), args_.output_frame.c_str());
      return true;
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(
        kLogger, "No static TF chain from '%s' to '%s': %s", cloud_frame.c_str(),
        args_.output_frame.c_str(), e.what());
      return false;
    }
  }

  // Resolve --backend plus a CUDA device probe into use_gpu_. Returns false
  // (logged) only when 'cuda' was forced but is unavailable; 'auto' silently uses
  // CPU when GPU is unavailable (announcing the fallback when a CUDA build merely
  // lacks a usable device).
  bool resolve_backend()
  {
    const std::string & backend = args_.backend;
    const auto cuda = core::slam::query_cuda_device();
    const bool gpu_runnable = cuda.has_cuda_build && cuda.error.empty() && cuda.device_count > 0;

    if (backend == "cpu") {
      use_gpu_ = false;
      return true;
    }
    if (backend == "cuda") {
      if (!cuda.has_cuda_build) {
        BAGWIZ_LOG_ERROR(
          kLogger,
          "--backend cuda requested but this binary was built without CUDA; rebuild with "
          "-DBAGWIZ_WITH_SLAM_CUDA=ON (pixi run -e humble-cuda build-full), or use --backend "
          "auto/cpu.");
        return false;
      }
      if (!cuda.error.empty()) {
        BAGWIZ_LOG_ERROR(
          kLogger, "--backend cuda: CUDA device query failed: %s", cuda.error.c_str());
        return false;
      }
      if (cuda.device_count <= 0) {
        BAGWIZ_LOG_ERROR(kLogger, "--backend cuda requested but no CUDA device is available.");
        return false;
      }
      use_gpu_ = true;
      BAGWIZ_LOG_INFO(kLogger, "Backend: GPU (CUDA).");
      return true;
    }
    // auto (the default): prefer GPU when runnable, else CPU.
    use_gpu_ = gpu_runnable;
    if (use_gpu_) {
      BAGWIZ_LOG_INFO(kLogger, "Backend: GPU (CUDA) — auto-selected.");
    } else if (cuda.has_cuda_build) {
      // CUDA build but no usable device: announce the CPU fallback (a non-CUDA
      // build under 'auto' is silently CPU, the normal case).
      const std::string cuda_error_suffix =
        cuda.error.empty() ? std::string() : std::string(": ") + cuda.error;
      BAGWIZ_LOG_INFO(
        kLogger, "Backend: CPU — auto (no usable CUDA device%s).", cuda_error_suffix.c_str());
    }
    return true;
  }

  // Optimized mapping path -> optimized TUM trajectory + binary PCD map.
  int run_mapping(
    io::BagReader & reader, const std::optional<core::slam::SensorTransform> & t_lidar_imu)
  {
    // Resolve the antenna lever-arm (T_cloud_gnss) from static TF so the GNSS prior
    // constrains the sensor origin, not the antenna. Non-fatal: a missing TF leaves
    // the offset zero (raw-antenna behavior) with a warning.
    std::array<double, 3> gnss_antenna_offset{0.0, 0.0, 0.0};
    if (!args_.gnss_topic.empty()) {
      gnss_antenna_offset = resolve_gnss_offset();
    }
    const core::slam::CloudMapperConfig config =
      build_mapper_config(args_, t_lidar_imu, use_gpu_, gnss_antenna_offset);

    // Resolve the optional --frame remapping up front. The trajectory is expressed
    // in the PointCloud2 frame_id by default; a requested --frame is resolved
    // through the bag's static TF and applied after optimization. Resolving here
    // avoids running the full SLAM pipeline only to fail on an invalid frame.
    std::string cloud_frame;
    std::optional<geometry_msgs::msg::Transform> output_body_to;
    if (!resolve_output_transform(cloud_frame, output_body_to)) {
      return 1;
    }

    core::slam::CloudMapper mapper(config);

    // Projects each NavSatFix to a local ENU frame (origin = first fix) before
    // handing it to the mapper as a metric GnssPoint. A no-op when GNSS is off
    // (process_messages won't read the topic), but cheap to always construct.
    core::slam::GnssProjector projector;
    auto on_gnss = [&](const core::slam::GnssSample & g) {
      const std::array<double, 3> enu = projector.project(g.latitude, g.longitude, g.altitude);
      core::slam::GnssPoint point{g.stamp_ns, enu};
      // NavSatFix covariance is ENU at the antenna; the local ENU projection
      // preserves those axes over the trajectory area, so carry it unchanged for
      // per-prior weighting in the mapper.
      point.covariance = g.position_covariance;
      point.covariance_type = g.position_covariance_type;
      mapper.insert_gnss(point);
    };

    const auto progress_setup =
      resolve_scan_progress(reader, args_, ::isatty(STDERR_FILENO) != 0, kLogger);
    const bool progress_on = progress_setup.enabled;
    core::slam::ScanProgress progress(progress_setup.total_msgs, progress_on);

    std::int64_t scans = 0;
    std::int64_t skipped = 0;
    std::int64_t imu_count = 0;
    std::int64_t gnss_count = 0;
    if (!process_messages(
          reader, [&](const core::slam::LidarScan & s) { mapper.insert(s); },
          [&](const core::slam::ImuSample & i) { mapper.insert_imu(i); }, on_gnss, progress, scans,
          skipped, imu_count, gnss_count)) {
      return 1;
    }
    progress.done();

    auto finalized = finalize_with_spinner(mapper, progress_on, kLogger);
    core::slam::CloudMap map = std::move(finalized.map);

    if (map.trajectory.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "SLAM produced no trajectory poses from %s scans", std::to_string(scans).c_str());
      return 1;
    }

    // The dynamic-point removal ran inside finish() (on the per-scan frames,
    // before the export merge); surface its outcome the way the outlier
    // removal below logs its own.
    if (args_.remove_dynamic) {
      BAGWIZ_LOG_INFO(
        kLogger,
        "Dynamic-point removal dropped %zu of %zu scan point(s) in %.1fs (voxel %.2f m, "
        "d_s %.2f m, d_p %d)",
        map.dynamic_removed_point_count, map.dynamic_input_point_count, map.dynamic_removal_seconds,
        args_.dynamic_resolution, args_.dynamic_sensor_offset, args_.dynamic_neighborhood);
    }

    // Radius outlier removal BEFORE colorization, so only the surviving
    // points are colorized and exported (the colorizer builds its kd-tree
    // over the filtered cloud).
    if (args_.remove_outliers) {
      const std::size_t before = map.points.size();
      std::size_t removed = 0;
      {
        core::slam::FinalizeSpinner spinner("Removing isolated points", progress_on);
        removed = remove_isolated_map_points(
          map, args_.outlier_radius, args_.outlier_min_neighbors,
          resolve_threads(args_.num_threads));
      }
      BAGWIZ_LOG_INFO(
        kLogger,
        "Outlier removal dropped %zu isolated point(s) (%zu -> %zu): fewer than %d neighbors "
        "within %.2f m",
        removed, before, map.points.size(), args_.outlier_min_neighbors, args_.outlier_radius);
    }

    // Colorize the map from the camera images BEFORE the optional --frame
    // remap: the colorizer interpolates camera poses from the trajectory,
    // which at this point still expresses the cloud frame the camera
    // extrinsic was resolved against.
    std::vector<std::array<std::uint8_t, 3>> map_colors;
    if (!args_.image_topics.empty()) {
      core::slam::FinalizeSpinner spinner("Colorizing map", progress_on);
      colorize_map(map, map_colors, use_gpu_);
    }

    // Apply the optional --frame remapping before writing.
    if (output_body_to.has_value()) {
      transform_trajectory_to_frame(map.trajectory, *output_body_to);
    }

    // Write the trajectory and the map; the map stream is flushed and closed
    // inside so the --viewer serve below reads a complete file.
    if (!write_map_outputs(
          output_path_, map_path_, map.trajectory, map.points, map.intensities, map_colors,
          kLogger)) {
      return 1;
    }

    log_mapping_summary(
      map, args_, scans, skipped, imu_count, gnss_count, output_path_, map_path_, kLogger);

    // --viewer: serve the map.pcd just written and open the browser. This blocks
    // until the user interrupts the viewer.
    if (args_.viewer) {
#ifdef BAGWIZ_WITH_MAP_VIEWER
      return core::slam::serve_map_viewer(map_path_);
#else
      BAGWIZ_LOG_ERROR(
        kLogger, "--viewer is unavailable: this binary was built without the map viewer");
      return 1;
#endif
    }
    return 0;
  }

  // Colorize the optimized map from the --cam image topics: stream every
  // camera's images through its own MapColorizer in a single bag pass,
  // gain-align the per-camera results and blend them by observation weight,
  // then fill the points no camera observed from their nearest observed
  // neighbor (unless --no-color-propagate) and move the result into `colors`
  // (parallel to map.points). NON-FATAL by design — the map geometry is valid
  // without colors, so every failure path warns and leaves `colors` empty
  // (map.pcd is then written without an rgb field) rather than discarding a
  // finished SLAM run.
  void colorize_map(
    const core::slam::CloudMap & map, std::vector<std::array<std::uint8_t, 3>> & colors,
    bool use_gpu)
  {
    const std::size_t cam_count = args_.image_topics.size();
    const int threads = colorize_thread_count(args_.num_threads);
    // With several cameras the per-image work (decode + add_image) is
    // parallelized across one worker thread per camera below, so each camera's
    // MapColorizer gets only its share of the thread budget for its internal
    // sweeps. One camera keeps the whole budget on the serial path.
    const bool parallel = cam_count > 1;
    const int sweep_threads =
      parallel ? std::max(1, threads / static_cast<int>(cam_count)) : threads;

    // The geometry pre-pass (kd-tree, normals, spacings) is camera
    // independent and the kd-tree build is the expensive part, so build it
    // once and share it between every camera's MapColorizer.
    const auto geometry = build_shared_colorize_geometry(map.points, threads);
    auto colorizers = build_camera_colorizers(
      camera_infos_, t_cloud_cams_, args_.range_max, sweep_threads, use_gpu, geometry, map.points,
      map.trajectory);

    std::unique_ptr<io::BagReader> reader;
    try {
      reader = io::open_read(args_.input_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_WARN(
        kLogger, "Could not reopen %s for colorization (%s); map.pcd is written without colors.",
        args_.input_path.c_str(), e.what());
      return;
    }
    io::ReadFilter filter;
    filter.topics = args_.image_topics;
    // The SLAM LiDAR stream rides along: each scan is the scene's occluder
    // geometry at its own time, so the colorizer can reject map points that
    // sit behind vehicles and pedestrians which left nothing in the
    // accumulated map (see MapColorizer::add_image's dynamic_points).
    filter.topics.push_back(args_.cloud_topic);
    reader->set_filter(filter);

    const auto colorize_start = std::chrono::steady_clock::now();
    std::vector<std::int64_t> decode_failures(cam_count, 0);
    // Images are paired with their temporally NEAREST scan, not the latest
    // one: the scan is the visibility oracle for moving traffic, so the
    // pairing must be tight (see ScanImagePairer for the queuing rule).
    core::slam::ScanImagePairer pairer;

    // Camera-parallel pipeline: the reader loop below stays single-threaded
    // and hands each decided image to its camera's worker via a bounded
    // queue. A MapColorizer is touched only by its own worker, and the worker
    // consumes the queue in FIFO order, so each camera's add_image sequence —
    // and therefore the colorize result — is identical to the serial run.
    constexpr std::size_t kWorkQueueCapacity = 4;
    std::vector<std::unique_ptr<ColorizeWorkQueue>> work_queues;
    std::vector<std::thread> workers;
    if (parallel) {
      work_queues.reserve(cam_count);
      workers.reserve(cam_count);
      for (std::size_t cam = 0; cam < cam_count; ++cam) {
        work_queues.push_back(std::make_unique<ColorizeWorkQueue>(kWorkQueueCapacity));
      }
      for (std::size_t cam = 0; cam < cam_count; ++cam) {
        workers.emplace_back([&, cam]() {
          std::int64_t failures = 0;
          ColorizeWorkItem item;
          while (work_queues[cam]->pop(item)) {
            try {
              const auto decoded = core::image::to_packed_raster(item.type, item.payload);
              if (!decoded.ok()) {
                ++failures;
                continue;
              }
              const auto & raster = *decoded.raster;
              // Prefer the capture stamp; fall back to the queue stamp (the
              // bag record time) when the publisher left header.stamp unset.
              const std::int64_t stamp =
                raster.header_stamp_ns != 0 ? raster.header_stamp_ns : item.stamp_ns;
              colorizers[cam]->add_image(
                stamp, raster.bgr, raster.width, raster.height, item.dynamic_points);
            } catch (const std::exception &) {
              ++failures;
            }
          }
          decode_failures[cam] = failures;
        });
      }
    }

    auto feed_image = [&](
                        core::slam::ScanImagePairer::PendingImage & img,
                        std::span<const std::array<float, 3>> dynamic) {
      if (parallel) {
        work_queues[img.cam]->push(
          ColorizeWorkItem{
            img.stamp_ns, std::move(img.type), std::move(img.payload),
            std::vector<std::array<float, 3>>(dynamic.begin(), dynamic.end())});
        return;
      }
      const auto decoded = core::image::to_packed_raster(img.type, img.payload);
      if (!decoded.ok()) {
        ++decode_failures[img.cam];
        return;
      }
      const auto & raster = *decoded.raster;
      // Prefer the capture stamp; fall back to the queue stamp (the bag
      // record time) when the publisher left header.stamp unset.
      const std::int64_t stamp =
        raster.header_stamp_ns != 0 ? raster.header_stamp_ns : img.stamp_ns;
      colorizers[img.cam]->add_image(stamp, raster.bgr, raster.width, raster.height, dynamic);
    };
    // Feed every pending image whose nearest scan is known.
    auto drain_pairer = [&]() {
      while (pairer.has_decidable()) {
        auto decision = pairer.decide_front();
        feed_image(decision.image, decision.dynamic_points);
      }
    };

    io::RawMessage raw;
    try {
      while (reader->next(raw)) {
        if (raw.topic->name == args_.cloud_topic) {
          const auto parsed = core::pointcloud::parse_pointcloud2(raw.payload);
          if (!parsed.ok()) {
            continue;
          }
          const auto extracted = core::slam::to_lidar_scan(*parsed.cloud);
          if (!extracted.ok()) {
            continue;
          }
          auto world = core::slam::scan_to_world_points(*extracted.scan, map.trajectory);
          if (!world.has_value()) {
            continue;
          }
          pairer.push_scan(extracted.scan->stamp_ns, std::move(*world));
          drain_pairer();
          continue;
        }
        // One filtered pass over the bag; each message belongs to exactly one
        // camera, dispatched by topic name.
        std::size_t cam = cam_count;
        for (std::size_t i = 0; i < cam_count; ++i) {
          if (raw.topic->name == args_.image_topics[i]) {
            cam = i;
            break;
          }
        }
        if (cam == cam_count) {
          continue;
        }
        pairer.push_image(
          core::slam::ScanImagePairer::PendingImage{
            cam,
            core::image::image_capture_stamp_ns(raw.topic->type, raw.payload, raw.timestamp_ns),
            raw.topic->type, std::vector<std::byte>(raw.payload.begin(), raw.payload.end())});
        drain_pairer();
      }
      // End of stream: no closer scan can still arrive; flush the rest.
      pairer.finish();
      drain_pairer();
    } catch (const std::exception & e) {
      BAGWIZ_LOG_WARN(
        kLogger,
        "Error reading the --cam topic(s) for colorization (%s); continuing with the images "
        "read so far.",
        e.what());
    }

    // Stop accepting work and let the camera workers drain their queues.
    for (auto & queue : work_queues) {
      queue->close();
    }
    for (auto & worker : workers) {
      worker.join();
    }

    std::vector<core::slam::MapColorizeResult> results;
    results.reserve(cam_count);
    for (std::size_t cam = 0; cam < cam_count; ++cam) {
      results.push_back(colorizers[cam]->finish());
      const auto & result = results.back();
      BAGWIZ_LOG_INFO(
        kLogger,
        "Colorize: %zu of %zu map points observed via '%s' (%zu image(s) used, %zu outside "
        "the trajectory span, %" PRId64 " failed to decode)",
        result.colored_points, map.points.size(), args_.image_topics[cam].c_str(),
        result.images_used, result.images_skipped, decode_failures[cam]);
    }

    auto merged = core::slam::merge_colorize_results(results);
    const double colorize_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - colorize_start).count();
    if (merged.images_used == 0) {
      BAGWIZ_LOG_WARN(
        kLogger,
        "No usable image on any --cam topic for colorization; map.pcd is written without "
        "colors.");
      return;
    }

    // Fill the points no camera observed with the color of the nearest
    // observed neighbor. The radius follows the data — 4x the median local
    // point spacing, clamped to [0.05, 5] m — so it tracks the map density
    // instead of an absolute guess. Degenerate inputs (empty spacings, a
    // non-finite or zero median) skip the propagation.
    std::size_t propagated = 0;
    if (args_.color_propagate && merged.colored_points > 0) {
      const auto radius = core::slam::propagation_radius_from_spacings(geometry->spacings);
      if (radius.has_value()) {
        propagated = core::pointcloud::propagate_uncolored(
          map.points, geometry->tree, merged.colors, merged.observed, *radius, threads);
        BAGWIZ_LOG_INFO(
          kLogger, "Propagated colors to %zu unobserved map points (radius %.3f m)", propagated,
          *radius);
      }
    }

    colors = std::move(merged.colors);
    if (args_.color_propagate) {
      const std::size_t uncolored = map.points.size() - merged.colored_points - propagated;
      BAGWIZ_LOG_INFO(
        kLogger,
        "Colorized %zu of %zu map points from %zu camera topic(s) in %.1fs (%zu propagated "
        "from observed neighbors, %zu left uncolored)",
        merged.colored_points, map.points.size(), cam_count, colorize_seconds, propagated,
        uncolored);
    } else {
      BAGWIZ_LOG_INFO(
        kLogger,
        "Colorized %zu of %zu map points from %zu camera topic(s) in %.1fs (%zu left uncolored)",
        merged.colored_points, map.points.size(), cam_count, colorize_seconds,
        map.points.size() - merged.colored_points);
    }
  }

  const MapSlamArgs & args_;
  std::filesystem::path output_path_;  // <output_root>/traj.tum
  std::filesystem::path map_path_;     // <output_root>/map.pcd (mapping mode only)
  // Effective backend resolved by resolve_backend() from --backend.
  bool use_gpu_ = false;
  // --cam state, parallel to args_.image_topics (listing order; the first
  // topic is the gain-alignment reference when the per-camera results are
  // blended), filled by validate_camera_inputs / resolve_camera_extrinsics.
  std::vector<std::string> camera_info_topics_;            // resolved CameraInfo topics
  std::vector<core::image::CameraInfo> camera_infos_;      // first CameraInfo message on each
  std::vector<core::slam::SensorTransform> t_cloud_cams_;  // cloud <- camera optical frames
};

}  // namespace

int run_map_slam(const MapSlamArgs & args)
{
  return MapSlamRunner(args).run();
}

}  // namespace bagwiz::commands
