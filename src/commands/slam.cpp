// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/core/logging.hpp"
#include "bagwiz/core/output_path.hpp"
#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/core/slam/cloud_mapper.hpp"
#include "bagwiz/core/slam/cloud_odometry.hpp"
#include "bagwiz/core/slam/lidar_scan.hpp"
#include "bagwiz/core/slam/point_cloud_io.hpp"
#include "bagwiz/core/trajectory.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <fmt/core.h>

#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace bagwiz::commands
{
namespace
{
constexpr const char * kLogger = "bagwiz.cmd.slam";
constexpr const char * kPointCloud2Type = "sensor_msgs/msg/PointCloud2";
}  // namespace

// `bagwiz slam <input> <cloud_topic>` runs LiDAR-only SLAM (GLIM's
// OdometryEstimationCT) over a single PointCloud2 topic, entirely in-process:
// bagwiz reads + decodes the bag and feeds GLIM's modules directly, with no ROS
// node / pub-sub. The estimated 6-DoF trajectory is written in the TUM format.
//
// With `--map <out.ply>` the marginalized frames additionally flow through
// GLIM's SubMapping -> GlobalMapping (CloudMapper) so the output is the
// globally-optimized trajectory plus an optimized world-frame point-cloud map
// (binary PLY). Without it, only odometry runs and the trajectory is written.
//
// Clouds without a per-point time field are treated as already
// motion-undistorted (all points simultaneous). IMU / camera fusion and the GPU
// backend are later milestones.
class SlamCommand : public Command
{
public:
  [[nodiscard]] std::string_view name() const override { return "slam"; }
  [[nodiscard]] std::string_view description() const override
  {
    return "Estimate a trajectory from a LiDAR PointCloud2 topic (GLIM, in-process)";
  }

  void configure(CLI::App & app) override
  {
    app.add_option("input", input_path_, "Bag path (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    app.add_option("topic", cloud_topic_, "PointCloud2 topic to run SLAM on")->required();
    app.add_option("-o,--output", output_path_, "Output trajectory path (TUM format)")->required();
    app.add_option(
      "--map", map_path_,
      "Also build and write the globally-optimized point-cloud map (binary PLY); "
      "the trajectory written to -o is then the optimized one");
    app
      .add_option(
        "--map-resolution", map_resolution_,
        "Exported map voxel size in meters (smaller = denser; default 0.2). Controls "
        "only the exported map's density, never the optimization or trajectory. The "
        "LiDAR preprocessor's ~0.15 m input voxel bounds the real resolution. Only used "
        "with --map.")
      ->check(CLI::PositiveNumber);
    app.add_flag("-w,--overwrite", overwrite_, "Overwrite the output(s) if they already exist");
  }

  int run() override
  {
    std::unique_ptr<io::BagReader> reader;
    try {
      reader = io::open_read(input_path_);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", input_path_.c_str(), e.what());
      return 1;
    }

    const io::TopicInfo * topic_info = nullptr;
    for (const auto & t : reader->topics()) {
      if (t.name == cloud_topic_) {
        topic_info = &t;
        break;
      }
    }
    if (topic_info == nullptr) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Topic '%s' is not present in %s", cloud_topic_.c_str(), input_path_.c_str());
      return 1;
    }
    if (topic_info->type != kPointCloud2Type) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Topic '%s' is %s, expected %s", cloud_topic_.c_str(), topic_info->type.c_str(),
        kPointCloud2Type);
      return 1;
    }

    const auto prepared = core::prepare_output_path(output_path_, overwrite_);
    if (!prepared.ok) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", prepared.error.c_str());
      return 1;
    }
    if (!map_path_.empty()) {
      const auto prepared_map = core::prepare_output_path(map_path_, overwrite_);
      if (!prepared_map.ok) {
        BAGWIZ_LOG_ERROR(kLogger, "%s", prepared_map.error.c_str());
        return 1;
      }
    }

    io::ReadFilter filter;
    filter.topics.push_back(cloud_topic_);
    reader->set_filter(filter);

    // `--map` swaps the odometry-only path (M1) for the optimized mapping
    // pipeline; everything up to here (open / validate / prepare outputs) is
    // shared.
    return map_path_.empty() ? run_odometry(*reader) : run_mapping(*reader);
  }

private:
  // Read every decodable scan on the topic in arrival order, feeding each to
  // `consume`. Returns false on a fatal read error or when no scan decoded
  // (both already logged); otherwise fills `scans` / `skipped`.
  template <typename Consumer>
  bool read_scans(
    io::BagReader & reader, Consumer && consume, std::int64_t & scans, std::int64_t & skipped)
  {
    io::RawMessage raw;
    try {
      while (reader.next(raw)) {
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
        consume(*scan.scan);
        ++scans;
      }
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(
        kLogger, "read error after %s scans: %s", std::to_string(scans).c_str(), e.what());
      return false;
    }
    if (scans == 0) {
      BAGWIZ_LOG_ERROR(kLogger, "No decodable PointCloud2 messages on '%s'", cloud_topic_.c_str());
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

  // M1 path: odometry only -> TUM trajectory.
  int run_odometry(io::BagReader & reader)
  {
    core::slam::CloudOdometry odometry;
    std::int64_t scans = 0;
    std::int64_t skipped = 0;
    if (!read_scans(
          reader, [&](const core::slam::LidarScan & s) { odometry.insert(s); }, scans, skipped)) {
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
      stdout, "Wrote {} trajectory poses from {} scans ({} skipped) to {}\n", poses.size(), scans,
      skipped, output_path_.string());
    return 0;
  }

  // M2 path: optimized mapping -> optimized TUM trajectory + binary PLY map.
  int run_mapping(io::BagReader & reader)
  {
    core::slam::CloudMapperConfig config;
    config.map_resolution = map_resolution_;
    core::slam::CloudMapper mapper(config);
    std::int64_t scans = 0;
    std::int64_t skipped = 0;
    if (!read_scans(
          reader, [&](const core::slam::LidarScan & s) { mapper.insert(s); }, scans, skipped)) {
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
      "Wrote {} optimized trajectory poses and a {}-point map from {} scans ({} skipped) "
      "to {} and {}\n",
      map.trajectory.size(), map.points.size(), scans, skipped, output_path_.string(),
      map_path_.string());
    return 0;
  }

  std::filesystem::path input_path_;
  std::string cloud_topic_;
  std::filesystem::path output_path_;
  std::filesystem::path map_path_;
  double map_resolution_ = 0.2;  // exported-map voxel size [m]; see --map-resolution
  bool overwrite_ = false;
};

BAGWIZ_REGISTER_COMMAND(SlamCommand)

}  // namespace bagwiz::commands
