// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/slam_run.hpp"

#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/logging.hpp"
#include "bagwiz/core/output_path.hpp"
#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/core/slam/cloud_mapper.hpp"
#include "bagwiz/core/slam/cloud_odometry.hpp"
#include "bagwiz/core/slam/imu_sample.hpp"
#include "bagwiz/core/slam/lidar_scan.hpp"
#include "bagwiz/core/slam/point_cloud_io.hpp"
#include "bagwiz/core/slam/sensor_transform.hpp"
#include "bagwiz/core/tf_chain.hpp"
#include "bagwiz/core/tf_value_extract.hpp"
#include "bagwiz/core/trajectory.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <tf2/buffer_core.hpp>
#include <tf2/time.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <fmt/core.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bagwiz::commands
{
namespace
{
constexpr const char * kLogger = "bagwiz.cmd.slam";
constexpr const char * kPointCloud2Type = "sensor_msgs/msg/PointCloud2";
constexpr const char * kImuType = "sensor_msgs/msg/Imu";
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

// Drives a single `bagwiz slam run` invocation. Holds the parsed arguments plus
// the output paths derived from output_root, and ports the bag reading + GLIM
// feeding that used to live on the SlamCommand. One instance per run().
class SlamRunner
{
public:
  explicit SlamRunner(const SlamRunArgs & args) : args_(args) {}

  int run()
  {
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

    // Global mapping is the default; derive the map path unless the user asked to
    // skip it.
    if (!args_.without_global_optim) {
      map_path_ = args_.output_root / "map.ply";
      const auto prepared_map = core::prepare_output_path(map_path_, args_.overwrite);
      if (!prepared_map.ok) {
        BAGWIZ_LOG_ERROR(kLogger, "%s", prepared_map.error.c_str());
        return 1;
      }
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

    // `--without-global-optim` swaps the optimized mapping pipeline for the raw
    // odometry path; everything up to here (open / validate / prepare outputs /
    // extrinsic) is shared.
    return args_.without_global_optim ? run_odometry(*reader, t_lidar_imu)
                                      : run_mapping(*reader, t_lidar_imu);
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
  bool build_static_tf_buffer(tf2::BufferCore & buffer)
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
        "Bag has no static TF topic (…tf_static); cannot resolve the LiDAR<-IMU extrinsic. "
        "Provide a bag whose /tf_static connects the cloud and IMU frames.");
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
      fmt::print(
        stdout, "IMU and cloud share frame '{}'; using identity LiDAR<-IMU extrinsic\n",
        cloud_frame);
      return true;
    }

    tf2::BufferCore buffer{kTfBufferCacheTime};
    if (!build_static_tf_buffer(buffer)) {
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

    fmt::print(
      stdout, "Resolved LiDAR<-IMU extrinsic from static TF ('{}' <- '{}')\n", cloud_frame,
      imu_frame);
    return true;
  }

  // Read the cloud (and, in IMU mode, IMU) topic in log order, dispatching each
  // message by type. Returns false on a fatal read error or when no scan decoded
  // (both logged); otherwise fills the counters.
  template <typename ScanFn, typename ImuFn>
  bool process_messages(
    io::BagReader & reader, ScanFn && on_scan, ImuFn && on_imu, std::int64_t & scans,
    std::int64_t & skipped, std::int64_t & imu_count)
  {
    io::ReadFilter filter;
    filter.topics.push_back(args_.cloud_topic);
    if (!args_.imu_topic.empty()) {
      filter.topics.push_back(args_.imu_topic);
    }
    reader.set_filter(filter);

    io::RawMessage raw;
    try {
      while (reader.next(raw)) {
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
        }
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

  // Odometry-only path -> TUM trajectory.
  int run_odometry(
    io::BagReader & reader, const std::optional<core::slam::SensorTransform> & t_lidar_imu)
  {
    core::slam::CloudOdometry odometry(t_lidar_imu);
    std::int64_t scans = 0;
    std::int64_t skipped = 0;
    std::int64_t imu_count = 0;
    if (!process_messages(
          reader, [&](const core::slam::LidarScan & s) { odometry.insert(s); },
          [&](const core::slam::ImuSample & i) { odometry.insert_imu(i); }, scans, skipped,
          imu_count)) {
      return 1;
    }

    const std::vector<core::TrajectoryPose> poses = odometry.finish();
    if (poses.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "SLAM produced no trajectory poses from %s scans", std::to_string(scans).c_str());
      return 1;
    }
    if (!write_trajectory(poses)) {
      return 1;
    }

    fmt::print(
      stdout, "Wrote {} trajectory poses from {} scans{} ({} skipped) to {}\n", poses.size(), scans,
      imu_suffix(imu_count), skipped, output_path_.string());
    return 0;
  }

  // Optimized mapping path -> optimized TUM trajectory + binary PLY map.
  int run_mapping(
    io::BagReader & reader, const std::optional<core::slam::SensorTransform> & t_lidar_imu)
  {
    core::slam::CloudMapperConfig config;
    config.map_resolution = args_.map_resolution;
    config.t_lidar_imu = t_lidar_imu;
    core::slam::CloudMapper mapper(config);
    std::int64_t scans = 0;
    std::int64_t skipped = 0;
    std::int64_t imu_count = 0;
    if (!process_messages(
          reader, [&](const core::slam::LidarScan & s) { mapper.insert(s); },
          [&](const core::slam::ImuSample & i) { mapper.insert_imu(i); }, scans, skipped,
          imu_count)) {
      return 1;
    }

    const core::slam::CloudMap map = mapper.finish();
    if (map.trajectory.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "SLAM produced no trajectory poses from %s scans", std::to_string(scans).c_str());
      return 1;
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
    core::slam::write_ply(map_out, map.points, map.intensities);
    if (!map_out.good()) {
      BAGWIZ_LOG_ERROR(kLogger, "write failed: %s", map_path_.c_str());
      return 1;
    }

    fmt::print(
      stdout,
      "Wrote {} optimized trajectory poses and a {}-point map from {} scans{} ({} skipped) "
      "to {} and {}\n",
      map.trajectory.size(), map.points.size(), scans, imu_suffix(imu_count), skipped,
      output_path_.string(), map_path_.string());
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

  const SlamRunArgs & args_;
  std::filesystem::path output_path_;  // <output_root>/traj.tum
  std::filesystem::path map_path_;     // <output_root>/map.ply (mapping mode only)
};

}  // namespace

int run_slam_run(const SlamRunArgs & args)
{
  return SlamRunner(args).run();
}

}  // namespace bagwiz::commands
