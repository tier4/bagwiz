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
#include "bagwiz/core/slam/gnss_projector.hpp"
#include "bagwiz/core/slam/gnss_sample.hpp"
#include "bagwiz/core/slam/imu_sample.hpp"
#include "bagwiz/core/slam/lidar_scan.hpp"
#include "bagwiz/core/slam/map_viewer.hpp"
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

#include <array>
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
    if (!args_.gnss_topic.empty()) {
      // GNSS constraints are added only to the global factor graph, so they are
      // meaningless without global mapping. Reject the contradictory combination
      // early rather than silently dropping the topic.
      if (args_.without_global_optim) {
        BAGWIZ_LOG_ERROR(
          kLogger,
          "--gnss requires global mapping but --without-global-optim was given; GNSS "
          "constraints only apply during global optimization");
        return 1;
      }
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

    // Global mapping is the default; derive the map path unless the user asked to
    // skip it.
    if (!args_.without_global_optim) {
      map_path_ = args_.output_root / "map.pcd";
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
      fmt::print(
        stdout, "GNSS and cloud share frame '{}'; antenna lever-arm is zero\n", cloud_frame);
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
    if (!build_static_tf_buffer(buffer)) {
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
      fmt::print(
        stdout,
        "Resolved GNSS antenna lever-arm from static TF ('{}' <- '{}'): "
        "({:.3f}, {:.3f}, {:.3f}) m\n",
        cloud_frame, gnss_frame, t.translation[0], t.translation[1], t.translation[2]);
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
    std::int64_t & scans, std::int64_t & skipped, std::int64_t & imu_count,
    std::int64_t & gnss_count)
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
    // GNSS is rejected together with --without-global-optim, so this path never
    // sees a GNSS topic; the handler is a no-op and the count stays 0.
    std::int64_t gnss_count = 0;
    if (!process_messages(
          reader, [&](const core::slam::LidarScan & s) { odometry.insert(s); },
          [&](const core::slam::ImuSample & i) { odometry.insert_imu(i); },
          [](const core::slam::GnssSample &) {}, scans, skipped, imu_count, gnss_count)) {
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

  // Optimized mapping path -> optimized TUM trajectory + binary PCD map.
  int run_mapping(
    io::BagReader & reader, const std::optional<core::slam::SensorTransform> & t_lidar_imu)
  {
    core::slam::CloudMapperConfig config;
    config.map_resolution = args_.map_resolution;
    config.t_lidar_imu = t_lidar_imu;
    config.enable_gnss = !args_.gnss_topic.empty();
    // Resolve the antenna lever-arm (T_cloud_gnss) from static TF so the GNSS prior
    // constrains the sensor origin, not the antenna. Non-fatal: a missing TF leaves
    // the offset zero (raw-antenna behavior) with a warning.
    if (config.enable_gnss) {
      config.gnss_antenna_offset = resolve_gnss_offset();
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

    std::int64_t scans = 0;
    std::int64_t skipped = 0;
    std::int64_t imu_count = 0;
    std::int64_t gnss_count = 0;
    if (!process_messages(
          reader, [&](const core::slam::LidarScan & s) { mapper.insert(s); },
          [&](const core::slam::ImuSample & i) { mapper.insert_imu(i); }, on_gnss, scans, skipped,
          imu_count, gnss_count)) {
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
    core::slam::write_pcd(map_out, map.points, map.intensities);
    // Flush and close before the good() check and before --vis serves the file.
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

    fmt::print(
      stdout,
      "Wrote {} optimized trajectory poses and a {}-point map from {} scans{} ({} skipped) "
      "to {} and {}\n",
      map.trajectory.size(), map.points.size(), scans, imu_suffix(imu_count), skipped,
      output_path_.string(), map_path_.string());

    if (!args_.gnss_topic.empty()) {
      if (map.gnss_factor_count > 0) {
        fmt::print(
          stdout, "Applied {} GNSS constraint(s) from {} fix(es) on '{}'\n", map.gnss_factor_count,
          gnss_count, args_.gnss_topic);
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

    // --vis: serve the map.pcd just written and open the browser. This blocks
    // until the user interrupts the viewer. CLI parsing already rejects
    // --vis together with --without-global-optim, so a map always exists here.
    if (args_.vis) {
#ifdef BAGWIZ_WITH_MAP_VIEWER
      return core::slam::serve_map_viewer(map_path_);
#else
      BAGWIZ_LOG_ERROR(
        kLogger, "--vis is unavailable: this binary was built without the map viewer");
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

  const SlamRunArgs & args_;
  std::filesystem::path output_path_;  // <output_root>/traj.tum
  std::filesystem::path map_path_;     // <output_root>/map.pcd (mapping mode only)
};

}  // namespace

int run_slam_run(const SlamRunArgs & args)
{
  return SlamRunner(args).run();
}

}  // namespace bagwiz::commands
