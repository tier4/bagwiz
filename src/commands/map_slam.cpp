// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/map_slam.hpp"

#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/logging.hpp"
#include "bagwiz/core/output_path.hpp"
#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/core/slam/cloud_mapper.hpp"
#include "bagwiz/core/slam/cuda_device.hpp"
#include "bagwiz/core/slam/gnss_projector.hpp"
#include "bagwiz/core/slam/gnss_sample.hpp"
#include "bagwiz/core/slam/imu_sample.hpp"
#include "bagwiz/core/slam/lidar_scan.hpp"
#include "bagwiz/core/slam/map_viewer.hpp"
#include "bagwiz/core/slam/point_cloud_io.hpp"
#include "bagwiz/core/slam/progress_bar.hpp"
#include "bagwiz/core/slam/sensor_transform.hpp"
#include "bagwiz/core/tf_chain.hpp"
#include "bagwiz/core/tf_value_extract.hpp"
#include "bagwiz/core/trajectory.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <tf2/buffer_core.hpp>
#include <tf2/time.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <fmt/core.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <memory>
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
constexpr std::string_view kTfStaticSuffix = "tf_static";
// Static transforms are timeless; a year-long cache dwarfs any bag and matches
// `tf walk` / `tf static calc` buffer sizing.
constexpr std::chrono::hours kTfBufferCacheTime{24 * 365};

bool is_static_tf_topic(std::string_view topic_name)
{
  if (topic_name.size() < kTfStaticSuffix.size()) {
    return false;
  }
  return topic_name.compare(
           topic_name.size() - kTfStaticSuffix.size(), kTfStaticSuffix.size(), kTfStaticSuffix) ==
         0;
}

// Clamp an explicit --threads value to the host's hardware concurrency so the
// user cannot oversubscribe the machine. A value <= 0 or a concurrency that
// cannot be queried leaves the argument unchanged (the caller applies defaults).
int cap_threads_at_hardware_limit(int num_threads)
{
  if (num_threads <= 0) {
    return num_threads;
  }
  const unsigned int hardware = std::thread::hardware_concurrency();
  if (hardware == 0) {
    return num_threads;
  }
  return std::min(num_threads, static_cast<int>(hardware));
}

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
    // already CLI-checked > 0 and --recovery-min-inliers is range-checked to
    // [0, 1]; here we enforce the relations between them.
    if (!(args_.range_min < args_.range_max)) {
      BAGWIZ_LOG_ERROR(
        kLogger, "--min-range (%g) must be strictly less than --max-range (%g).", args_.range_min,
        args_.range_max);
      return 1;
    }
    if (!(args_.recovery_min_inlier_fraction > 0.0)) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "--recovery-min-inliers must be > 0 (got %g); 0 would disable the recovery acceptance gate "
        "entirely.",
        args_.recovery_min_inlier_fraction);
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

    std::unique_ptr<io::BagReader> reader;
    try {
      reader = io::open_read(args_.input_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", args_.input_path.c_str(), e.what());
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
      if (t.type == kTfMessageType && is_static_tf_topic(t.name)) {
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
      if (info.type != kTfMessageType || !is_static_tf_topic(info.name)) {
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
      if (t.type == kTfMessageType && is_static_tf_topic(t.name)) {
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

  // Write `poses` as TUM to output_path_. Returns false on failure (logged).
  bool write_trajectory(const std::vector<core::TrajectoryPose> & poses)
  {
    std::ofstream out(output_path_, std::ios::binary);
    if (!out) {
      BAGWIZ_LOG_ERROR(kLogger, "could not open %s for writing", output_path_.c_str());
      return false;
    }
    core::write_tum(out, poses);
    if (!out.good()) {
      BAGWIZ_LOG_ERROR(kLogger, "write failed: %s", output_path_.c_str());
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
    core::slam::CloudMapperConfig config;
    config.input_resolution = args_.input_resolution;
    config.range_min = args_.range_min;
    config.range_max = args_.range_max;
    config.recovery_min_inlier_fraction = args_.recovery_min_inlier_fraction;
    config.submap_max_keyframes = args_.submap_max_keyframes;
    config.t_lidar_imu = t_lidar_imu;
    config.num_threads = cap_threads_at_hardware_limit(args_.num_threads);
    config.enable_gnss = !args_.gnss_topic.empty();
    config.use_gpu = use_gpu_;
    // Recovery scan-matches the window scans against the optimized map, so it runs
    // in LiDAR-only mode too; --imu only adds the IMU init/fallback path inside the
    // mapper. Gated solely on the recover toggles, not on the IMU topic.
    config.recover_start = args_.recover_start;
    config.recover_end = args_.recover_end;
    // Resolve the antenna lever-arm (T_cloud_gnss) from static TF so the GNSS prior
    // constrains the sensor origin, not the antenna. Non-fatal: a missing TF leaves
    // the offset zero (raw-antenna behavior) with a warning.
    if (config.enable_gnss) {
      config.gnss_antenna_offset = resolve_gnss_offset();
    }

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

    // Live progress bar (stderr) for the long read+feed phase. Auto-suppressed
    // off a TTY / under NO_COLOR / with --no-progress (progress_enabled), so it
    // never spams a pipe or log. The total is the number of messages the read
    // loop will stream; a stats failure only forfeits the determinate bar.
    const bool progress_on = core::slam::progress_enabled(
      ::isatty(STDERR_FILENO) != 0, std::getenv("NO_COLOR") != nullptr, args_.no_progress);
    std::int64_t progress_total_msgs = 0;
    if (progress_on) {
      std::vector<std::string> progress_topics{args_.cloud_topic};
      if (!args_.imu_topic.empty()) {
        progress_topics.push_back(args_.imu_topic);
      }
      if (!args_.gnss_topic.empty()) {
        progress_topics.push_back(args_.gnss_topic);
      }
      try {
        const auto topic_counts = reader.compute_topic_counts(progress_topics);
        progress_total_msgs = core::slam::progress_total(topic_counts, progress_topics);
      } catch (const std::exception & e) {
        BAGWIZ_LOG_WARN(
          kLogger, "Could not read bag stats for the progress bar (%s); using an indeterminate bar",
          e.what());
      }
    }
    core::slam::ScanProgress progress(progress_total_msgs, progress_on);

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

    // finish() runs the blocking finalization (global optimization + endpoint
    // recovery + map export) with no per-step progress; animate an indeterminate
    // spinner on a worker thread until it returns.
    core::slam::CloudMap map;
    const auto finalize_start = std::chrono::steady_clock::now();
    {
      core::slam::FinalizeSpinner spinner("Finalizing map", progress_on);
      map = mapper.finish();
    }
    const double finalize_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - finalize_start).count();
    // Log the breakdown, not just the total: endpoint recovery (up to a full
    // odometry smoother window of scan registrations), not the iSAM2 update,
    // dominates finalization on LiDAR-only runs, and a bare total reads as
    // "the optimizer is slow".
    BAGWIZ_LOG_INFO(
      kLogger,
      "Finalization took %.1fs (global optimization %.1fs, endpoint recovery %.1fs, "
      "map export %.1fs)",
      finalize_seconds, map.optimize_seconds, map.recovery_seconds, map.export_seconds);

    if (map.trajectory.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "SLAM produced no trajectory poses from %s scans", std::to_string(scans).c_str());
      return 1;
    }

    // Apply the optional --frame remapping before writing.
    if (output_body_to.has_value()) {
      transform_trajectory_to_frame(map.trajectory, *output_body_to);
    }

    // Open the map stream before committing the trajectory so an unwritable
    // --map path fails before either file is touched (rather than leaving an
    // orphaned trajectory behind).
    std::ofstream map_out(map_path_, std::ios::binary);
    if (!map_out) {
      BAGWIZ_LOG_ERROR(kLogger, "could not open %s for writing", map_path_.c_str());
      return 1;
    }
    if (!write_trajectory(map.trajectory)) {
      return 1;
    }
    core::slam::write_pcd(map_out, map.points, map.intensities);
    // Flush and close before the good() check and before --viewer serves the file.
    // An open ofstream keeps the final partial (<8 KiB) block in its user-space
    // buffer, so until the stream is destroyed the on-disk file is short of its
    // own header's vertex count. serve_map_viewer() (below) blocks while map_out
    // is still in scope, so without this close it would read a too-small
    // file_size, send a truncated body, and the browser's PCD loader would fail
    // with "Offset is outside the bounds of the DataView". close() also surfaces
    // a flush failure (e.g. disk full) through good() below, which the prior
    // mid-write good() check could not see.
    map_out.close();
    if (!map_out.good()) {
      BAGWIZ_LOG_ERROR(kLogger, "write failed: %s", map_path_.c_str());
      return 1;
    }

    BAGWIZ_LOG_INFO(
      kLogger,
      "Wrote %zu optimized trajectory poses and a %zu-point map from %zu scans%s (%zu skipped) "
      "to %s and %s",
      map.trajectory.size(), map.points.size(), scans, imu_suffix(imu_count).c_str(), skipped,
      output_path_.string().c_str(), map_path_.string().c_str());

    if (args_.recover_start) {
      if (map.recovered_start_pose_count > 0) {
        BAGWIZ_LOG_INFO(
          kLogger, "Recovered %zu initialization-window pose(s) by scan-matching",
          map.recovered_start_pose_count);
      } else if (map.warmup_overflowed) {
        BAGWIZ_LOG_INFO(
          kLogger,
          "Initialization-window recovery abandoned: the pre-init scan buffer overflowed before "
          "odometry converged (a very long static/slow start)");
      } else {
        BAGWIZ_LOG_INFO(
          kLogger,
          "No initialization-window poses recovered (odometry started immediately, or no "
          "pre-init scans)");
      }
    }

    if (args_.recover_end) {
      if (map.recovered_end_pose_count > 0) {
        BAGWIZ_LOG_INFO(
          kLogger, "Recovered %zu cooldown-window pose(s) by scan-matching",
          map.recovered_end_pose_count);
      } else {
        BAGWIZ_LOG_INFO(
          kLogger,
          "No cooldown-window poses recovered (no trailing scans past the last estimated "
          "frame)");
      }
    }

    if (!args_.gnss_topic.empty()) {
      if (map.gnss_factor_count > 0) {
        BAGWIZ_LOG_INFO(
          kLogger, "Applied %zu GNSS constraint(s) from %" PRId64 " fix(es) on '%s'",
          map.gnss_factor_count, gnss_count, args_.gnss_topic.c_str());
      } else {
        // GNSS was requested but the alignment could not initialize: the map is
        // still valid, just unconstrained by GNSS. Warn rather than fail.
        BAGWIZ_LOG_WARN(
          kLogger,
          "GNSS topic '%s' yielded no constraints (%s fix(es) read); the global optimization ran "
          "without GNSS. Likely too little motion (baseline) or no temporal overlap between GNSS "
          "and the submaps.",
          args_.gnss_topic.c_str(), std::to_string(gnss_count).c_str());
      }
    }

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

  // " + N IMU samples" when IMU mode ran, otherwise empty.
  std::string imu_suffix(std::int64_t imu_count) const
  {
    if (args_.imu_topic.empty()) {
      return "";
    }
    return fmt::format(" + {} IMU samples", imu_count);
  }

  const MapSlamArgs & args_;
  std::filesystem::path output_path_;  // <output_root>/traj.tum
  std::filesystem::path map_path_;     // <output_root>/map.pcd (mapping mode only)
  // Effective backend resolved by resolve_backend() from --backend.
  bool use_gpu_ = false;
};

}  // namespace

int run_map_slam(const MapSlamArgs & args)
{
  return MapSlamRunner(args).run();
}

}  // namespace bagwiz::commands
