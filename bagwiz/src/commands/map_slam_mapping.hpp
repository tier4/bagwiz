// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__MAP_SLAM_MAPPING_HPP_
#define COMMANDS__MAP_SLAM_MAPPING_HPP_

#include "bagwiz/commands/map_slam.hpp"
#include "bagwiz/core/slam/cloud_mapper.hpp"
#include "bagwiz/core/slam/sensor_transform.hpp"
#include "bagwiz/core/tf/trajectory.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>

// Internals of `map slam`'s mapping run, split out of map_slam.cpp so the
// config fill, the progress setup, the output writing, and the run summary
// can be unit-tested without driving a SLAM run. CLI-internal: this header
// lives with the command sources and is not installed.
namespace bagwiz::commands
{

// Fill the CloudMapperConfig from the parsed CLI arguments.
// `gnss_antenna_offset` is the antenna lever-arm (T_cloud_gnss.translation)
// the caller resolved from the bag's static TF so the GNSS prior constrains
// the sensor origin, not the antenna; a zero offset (GNSS off, or the TF
// absent) reproduces the raw-antenna behavior.
// `visual_cameras` is the --cam extrinsic table (cloud frame <- camera optical
// frame) in --cam listing order, so a row index is the
// VisualObservation::camera_id the visual frontends stamp; empty (no --cam)
// leaves the visual constraints off.
[[nodiscard]] core::slam::CloudMapperConfig build_mapper_config(
  const MapSlamArgs & args, const std::optional<core::slam::SensorTransform> & t_lidar_imu,
  bool use_gpu, const std::array<double, 3> & gnss_antenna_offset,
  std::span<const core::slam::SensorTransform> visual_cameras);

// Inputs for the ScanProgress construction: whether the live bar renders at
// all, and the total message count it runs against (0 = indeterminate).
struct ScanProgressSetup
{
  bool enabled = false;
  std::int64_t total_msgs = 0;
};

// Resolve the read+feed phase's progress setup. The bar renders only on an
// interactive stderr, with NO_COLOR unset and --no-progress not passed. The
// total is the number of messages the read loop will stream; a bag-stats
// failure only forfeits the determinate bar (warned).
[[nodiscard]] ScanProgressSetup resolve_scan_progress(
  io::BagReader & reader, const MapSlamArgs & args, bool stderr_is_tty, const char * logger);

// Result of finalize_with_spinner(): the finished map plus finish()'s
// wall-clock time in seconds.
struct FinalizeResult
{
  core::slam::CloudMap map;
  double seconds = 0.0;
};

// Run the blocking finalization (global optimization + endpoint window fill +
// map export), which exposes no per-step progress, under an indeterminate
// spinner, then log the timing breakdown.
[[nodiscard]] FinalizeResult finalize_with_spinner(
  core::slam::CloudMapper & mapper, bool progress_on, const char * logger);

// Remove isolated points from the finished map in place (radius outlier
// removal): a point survives when at least `min_neighbors` OTHER map points
// lie within `radius` meters. map.points and, when present, the parallel
// map.intensities are compacted together with their order preserved. Returns
// the number of removed points; radius <= 0 or min_neighbors <= 0 removes
// nothing (the CLI enforces positive values, this is only the last line of
// defense).
[[nodiscard]] std::size_t remove_isolated_map_points(
  core::slam::CloudMap & map, double radius, int min_neighbors, int num_threads);

// Write traj.tum (`trajectory`) and map.pcd (`points` + `intensities` +
// `colors`) to their paths. The map stream is opened BEFORE the trajectory
// file is committed so an unwritable map path fails before either file is
// touched (rather than leaving an orphaned trajectory behind), and it is
// flushed and closed before this returns — an open ofstream keeps the final
// partial block in its user-space buffer, so a later --viewer serve must not
// see the file still open. Returns false on any failure (logged).
[[nodiscard]] bool write_map_outputs(
  const std::filesystem::path & trajectory_path, const std::filesystem::path & map_path,
  std::span<const core::TrajectoryPose> trajectory, std::span<const std::array<float, 3>> points,
  std::span<const float> intensities, std::span<const std::array<std::uint8_t, 3>> colors,
  const char * logger);

// Log the end-of-run summary: the written pose/point counts, then the
// start/end fill outcomes and the GNSS constraint application (only for the
// features the run actually requested).
void log_mapping_summary(
  const core::slam::CloudMap & map, const MapSlamArgs & args, std::int64_t scans,
  std::int64_t skipped, std::int64_t imu_count, std::int64_t gnss_count,
  const std::filesystem::path & trajectory_path, const std::filesystem::path & map_path,
  const char * logger);

}  // namespace bagwiz::commands

#endif  // COMMANDS__MAP_SLAM_MAPPING_HPP_
