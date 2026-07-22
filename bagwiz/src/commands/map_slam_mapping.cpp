// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "map_slam_mapping.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/slam/point_cloud_io.hpp"
#include "bagwiz/core/slam/progress_bar.hpp"
#include "map_slam_threads.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <fmt/core.h>

#include <chrono>
#include <cinttypes>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <string>
#include <vector>

namespace bagwiz::commands
{
namespace
{

// " + N IMU samples" when IMU mode ran, otherwise empty.
std::string imu_suffix(const MapSlamArgs & args, std::int64_t imu_count)
{
  if (args.imu_topic.empty()) {
    return "";
  }
  return fmt::format(" + {} IMU samples", imu_count);
}

}  // namespace

core::slam::CloudMapperConfig build_mapper_config(
  const MapSlamArgs & args, const std::optional<core::slam::SensorTransform> & t_lidar_imu,
  bool use_gpu, const std::array<double, 3> & gnss_antenna_offset)
{
  core::slam::CloudMapperConfig config;
  config.input_resolution = args.input_resolution;
  config.range_min = args.range_min;
  config.range_max = args.range_max;
  config.fill_min_inlier_fraction = args.fill_min_inlier_fraction;
  config.submap_max_keyframes = args.submap_max_keyframes;
  config.t_lidar_imu = t_lidar_imu;
  config.num_threads = resolve_threads(args.num_threads);
  config.enable_gnss = !args.gnss_topic.empty();
  config.use_gpu = use_gpu;
  // The fill scan-matches the window scans against the optimized map, so it runs
  // in LiDAR-only mode too; --imu only adds the IMU init/fallback path inside the
  // mapper. Gated solely on the fill toggles, not on the IMU topic.
  config.fill_start = args.fill_start;
  config.fill_end = args.fill_end;
  config.gnss_antenna_offset = gnss_antenna_offset;
  return config;
}

ScanProgressSetup resolve_scan_progress(
  io::BagReader & reader, const MapSlamArgs & args, bool stderr_is_tty, const char * logger)
{
  // Live progress bar (stderr) for the long read+feed phase. Auto-suppressed
  // off a TTY / under NO_COLOR / with --no-progress (progress_enabled), so it
  // never spams a pipe or log. The total is the number of messages the read
  // loop will stream; a stats failure only forfeits the determinate bar.
  ScanProgressSetup setup;
  setup.enabled = core::slam::progress_enabled(
    stderr_is_tty, std::getenv("NO_COLOR") != nullptr, args.no_progress);
  if (setup.enabled) {
    std::vector<std::string> progress_topics{args.cloud_topic};
    if (!args.imu_topic.empty()) {
      progress_topics.push_back(args.imu_topic);
    }
    if (!args.gnss_topic.empty()) {
      progress_topics.push_back(args.gnss_topic);
    }
    try {
      const auto topic_counts = reader.compute_topic_counts(progress_topics);
      setup.total_msgs = core::slam::progress_total(topic_counts, progress_topics);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_WARN(
        logger, "Could not read bag stats for the progress bar (%s); using an indeterminate bar",
        e.what());
    }
  }
  return setup;
}

FinalizeResult finalize_with_spinner(
  core::slam::CloudMapper & mapper, bool progress_on, const char * logger)
{
  // finish() runs the blocking finalization (global optimization + endpoint
  // window fill + map export) with no per-step progress; animate an indeterminate
  // spinner on a worker thread until it returns.
  FinalizeResult result;
  const auto finalize_start = std::chrono::steady_clock::now();
  {
    core::slam::FinalizeSpinner spinner("Finalizing map", progress_on);
    result.map = mapper.finish();
  }
  result.seconds =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - finalize_start).count();
  // Log the breakdown, not just the total: the endpoint fill (up to a full
  // odometry smoother window of scan registrations), not the iSAM2 update,
  // dominates finalization on LiDAR-only runs, and a bare total reads as
  // "the optimizer is slow".
  BAGWIZ_LOG_INFO(
    logger,
    "Finalization took %.1fs (global optimization %.1fs, endpoint fill %.1fs, "
    "map export %.1fs)",
    result.seconds, result.map.optimize_seconds, result.map.window_fill_seconds,
    result.map.export_seconds);
  return result;
}

bool write_map_outputs(
  const std::filesystem::path & trajectory_path, const std::filesystem::path & map_path,
  std::span<const core::TrajectoryPose> trajectory, std::span<const std::array<float, 3>> points,
  std::span<const float> intensities, std::span<const std::array<std::uint8_t, 3>> colors,
  const char * logger)
{
  // Open the map stream before committing the trajectory so an unwritable
  // --map path fails before either file is touched (rather than leaving an
  // orphaned trajectory behind).
  std::ofstream map_out(map_path, std::ios::binary);
  if (!map_out) {
    BAGWIZ_LOG_ERROR(logger, "could not open %s for writing", map_path.c_str());
    return false;
  }

  std::ofstream traj_out(trajectory_path, std::ios::binary);
  if (!traj_out) {
    BAGWIZ_LOG_ERROR(logger, "could not open %s for writing", trajectory_path.c_str());
    return false;
  }
  core::write_tum(traj_out, trajectory);
  if (!traj_out.good()) {
    BAGWIZ_LOG_ERROR(logger, "write failed: %s", trajectory_path.c_str());
    return false;
  }

  core::slam::write_pcd(map_out, points, intensities, colors);
  // Flush and close before the good() check and before --viewer serves the file.
  // An open ofstream keeps the final partial (<8 KiB) block in its user-space
  // buffer, so until the stream is destroyed the on-disk file is short of its
  // own header's vertex count. serve_map_viewer() (called by the caller, below)
  // blocks while map_out is still in scope, so without this close it would read
  // a too-small file_size, send a truncated body, and the browser's PCD loader
  // would fail with "Offset is outside the bounds of the DataView". close()
  // also surfaces a flush failure (e.g. disk full) through good() below, which
  // the prior mid-write good() check could not see.
  map_out.close();
  if (!map_out.good()) {
    BAGWIZ_LOG_ERROR(logger, "write failed: %s", map_path.c_str());
    return false;
  }
  return true;
}

void log_mapping_summary(
  const core::slam::CloudMap & map, const MapSlamArgs & args, std::int64_t scans,
  std::int64_t skipped, std::int64_t imu_count, std::int64_t gnss_count,
  const std::filesystem::path & trajectory_path, const std::filesystem::path & map_path,
  const char * logger)
{
  BAGWIZ_LOG_INFO(
    logger,
    "Wrote %zu optimized trajectory poses and a %zu-point map from %zu scans%s (%zu skipped) "
    "to %s and %s",
    map.trajectory.size(), map.points.size(), scans, imu_suffix(args, imu_count).c_str(), skipped,
    trajectory_path.string().c_str(), map_path.string().c_str());

  if (args.fill_start) {
    if (map.filled_start_pose_count > 0) {
      BAGWIZ_LOG_INFO(
        logger, "Filled %zu initialization-window pose(s) by scan-matching",
        map.filled_start_pose_count);
    } else if (map.warmup_overflowed) {
      BAGWIZ_LOG_INFO(
        logger,
        "Initialization-window fill abandoned: the pre-init scan buffer overflowed before "
        "odometry converged (a very long static/slow start)");
    } else {
      BAGWIZ_LOG_INFO(
        logger,
        "No initialization-window poses filled (odometry started immediately, or no "
        "pre-init scans)");
    }
  }

  if (args.fill_end) {
    if (map.filled_end_pose_count > 0) {
      BAGWIZ_LOG_INFO(
        logger, "Filled %zu cooldown-window pose(s) by scan-matching", map.filled_end_pose_count);
    } else {
      BAGWIZ_LOG_INFO(
        logger,
        "No cooldown-window poses filled (no trailing scans past the last estimated "
        "frame)");
    }
  }

  if (!args.gnss_topic.empty()) {
    if (map.gnss_factor_count > 0) {
      BAGWIZ_LOG_INFO(
        logger, "Applied %zu GNSS constraint(s) from %" PRId64 " fix(es) on '%s'",
        map.gnss_factor_count, gnss_count, args.gnss_topic.c_str());
    } else {
      // GNSS was requested but the alignment could not initialize: the map is
      // still valid, just unconstrained by GNSS. Warn rather than fail.
      BAGWIZ_LOG_WARN(
        logger,
        "GNSS topic '%s' yielded no constraints (%s fix(es) read); the global optimization ran "
        "without GNSS. Likely too little motion (baseline) or no temporal overlap between GNSS "
        "and the submaps.",
        args.gnss_topic.c_str(), std::to_string(gnss_count).c_str());
    }
  }
}

}  // namespace bagwiz::commands
