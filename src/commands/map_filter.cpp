// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/map_filter.hpp"

#include "bagwiz/core/logging.hpp"
#include "bagwiz/core/output_path.hpp"
#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/core/slam/cloud_filters.hpp"
#include "bagwiz/core/slam/lidar_scan.hpp"
#include "bagwiz/core/slam/point_cloud_io.hpp"
#include "bagwiz/core/slam/progress_bar.hpp"
#include "bagwiz/core/trajectory.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <fmt/core.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace bagwiz::commands
{
namespace
{
constexpr const char * kLogger = "bagwiz.cmd.map";
constexpr const char * kPointCloud2Type = "sensor_msgs/msg/PointCloud2";
constexpr const char * kMapFileName = "map.pcd";

// Resolve a path that may be either a .pcd file or a directory containing
// map.pcd, mirroring the viewer's convention.
std::filesystem::path resolve_map_path(const std::filesystem::path & path)
{
  std::error_code ec;
  if (std::filesystem::is_directory(path, ec)) {
    return path / kMapFileName;
  }
  return path;
}

// True if `topic` exists in the bag and has the expected type.
bool topic_present_with_type(
  const io::BagReader & reader, const std::string & topic, const char * expected_type,
  const std::filesystem::path & bag_path)
{
  const io::TopicInfo * info = nullptr;
  for (const auto & t : reader.topics()) {
    if (t.name == topic) {
      info = &t;
      break;
    }
  }
  if (info == nullptr) {
    BAGWIZ_LOG_ERROR(kLogger, "Topic '%s' is not present in %s", topic.c_str(), bag_path.c_str());
    return false;
  }
  if (info->type != expected_type) {
    BAGWIZ_LOG_ERROR(
      kLogger, "Topic '%s' is %s, expected %s", topic.c_str(), info->type.c_str(), expected_type);
    return false;
  }
  return true;
}

// Rotate vector v by unit quaternion q = (qx, qy, qz, qw).
std::array<double, 3> rotate_vector(
  double qx, double qy, double qz, double qw, double vx, double vy, double vz)
{
  // t = 2 * cross(q_vec, v)
  const double tx = 2.0 * (qy * vz - qz * vy);
  const double ty = 2.0 * (qz * vx - qx * vz);
  const double tz = 2.0 * (qx * vy - qy * vx);
  // v' = v + qw * t + cross(q_vec, t)
  return {
    vx + qw * tx + (qy * tz - qz * ty),
    vy + qw * ty + (qz * tx - qx * tz),
    vz + qw * tz + (qx * ty - qy * tx),
  };
}

}  // namespace

int run_map_filter_removert(const MapFilterRemovertArgs & args)
{
  // Resolve and read the map to be filtered.
  const auto map_file = resolve_map_path(args.map_path);
  {
    std::error_code ec;
    if (!std::filesystem::is_regular_file(map_file, ec)) {
      BAGWIZ_LOG_ERROR(kLogger, "Map file not found: %s", map_file.c_str());
      return 1;
    }
  }

  core::slam::PcdCloud map;
  {
    std::ifstream map_in(map_file, std::ios::binary);
    if (!map_in) {
      BAGWIZ_LOG_ERROR(kLogger, "Could not open map %s", map_file.c_str());
      return 1;
    }
    const auto result = core::slam::read_pcd(map_in);
    if (!result.ok) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Failed to read map %s: %s", map_file.c_str(), result.error.c_str());
      return 1;
    }
    map = std::move(result.cloud);
  }
  if (map.points.empty()) {
    BAGWIZ_LOG_ERROR(kLogger, "Map is empty: %s", map_file.c_str());
    return 1;
  }

  // Validate the bag + topic before any heavy work.
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(args.input_path);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", args.input_path.c_str(), e.what());
    return 1;
  }
  if (!topic_present_with_type(*reader, args.cloud_topic, kPointCloud2Type, args.input_path)) {
    return 1;
  }

  // Read the optimized trajectory.
  std::vector<core::TrajectoryPose> traj;
  {
    std::ifstream traj_in(args.traj_path, std::ios::binary);
    if (!traj_in) {
      BAGWIZ_LOG_ERROR(kLogger, "Could not open trajectory %s", args.traj_path.c_str());
      return 1;
    }
    const auto result = core::read_tum(traj_in);
    traj = std::move(result.poses);
    if (result.skipped_lines > 0) {
      BAGWIZ_LOG_WARN(
        kLogger,
        "Trajectory %s skipped %s malformed line(s); only successfully parsed poses are used.",
        args.traj_path.c_str(), std::to_string(result.skipped_lines).c_str());
    }
    if (traj.empty()) {
      BAGWIZ_LOG_ERROR(kLogger, "Trajectory is empty: %s", args.traj_path.c_str());
      return 1;
    }
    std::sort(
      traj.begin(), traj.end(), [](const core::TrajectoryPose & a, const core::TrajectoryPose & b) {
        return a.timestamp_ns < b.timestamp_ns;
      });
  }

  // Resolve output path: a directory receives map.pcd inside it.
  std::filesystem::path out_file = args.output_path;
  {
    std::error_code ec;
    if (std::filesystem::is_directory(args.output_path, ec)) {
      out_file = args.output_path / kMapFileName;
    } else {
      // The path may not exist yet; if it looks like a directory (no extension),
      // treat it as one so a bare output directory is created automatically.
      if (!out_file.has_extension()) {
        std::filesystem::create_directories(out_file, ec);
        out_file /= kMapFileName;
      } else if (out_file.has_parent_path()) {
        std::filesystem::create_directories(out_file.parent_path(), ec);
      }
    }
  }
  const auto prepared = core::prepare_output_path(out_file, args.overwrite);
  if (!prepared.ok) {
    BAGWIZ_LOG_ERROR(kLogger, "%s", prepared.error.c_str());
    return 1;
  }

  // Open the output stream early so an unwritable path fails before filtering.
  std::ofstream out_stream(out_file, std::ios::binary);
  if (!out_stream) {
    BAGWIZ_LOG_ERROR(kLogger, "Could not open %s for writing", out_file.c_str());
    return 1;
  }

  // Progress bar over the bag-read phase.
  const bool progress_on = core::slam::progress_enabled(
    ::isatty(STDERR_FILENO) != 0, std::getenv("NO_COLOR") != nullptr, args.no_progress);
  std::int64_t progress_total_msgs = 0;
  if (progress_on) {
    try {
      const std::vector<std::string> progress_topics{args.cloud_topic};
      const auto topic_counts = reader->compute_topic_counts(progress_topics);
      progress_total_msgs = core::slam::progress_total(topic_counts, progress_topics);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_WARN(
        kLogger, "Could not read bag stats for the progress bar (%s); using an indeterminate bar",
        e.what());
    }
  }
  core::slam::ScanProgress progress(progress_total_msgs, progress_on);

  // Configure and construct the filter.
  core::slam::RemovertConfig config;
  config.vertical_fov_deg = args.vertical_fov_deg;
  config.horizontal_fov_deg = args.horizontal_fov_deg;
  config.remove_resolutions = args.remove_resolutions;
  config.revert_resolutions = args.revert_resolutions;
  config.adaptive_coeff = args.adaptive_coeff;
  config.valid_diff_upper_bound = args.valid_diff_upper_bound;
  config.enable_revert = args.enable_revert;
  core::slam::RemovertFilter filter(config, map.points);

  // Replay the selected PointCloud2 topic, transform each scan into the world
  // frame using the optimized trajectory, and feed it to the filter.
  io::ReadFilter read_filter;
  read_filter.topics.push_back(args.cloud_topic);
  reader->set_filter(read_filter);

  std::int64_t processed = 0;
  std::int64_t scans = 0;
  std::int64_t skipped = 0;
  std::int64_t no_pose = 0;
  io::RawMessage raw;
  try {
    while (reader->next(raw)) {
      ++processed;
      const auto parsed = core::pointcloud::parse_pointcloud2(raw.payload);
      if (!parsed.ok()) {
        ++skipped;
        progress.update(processed, scans);
        continue;
      }
      const auto scan = core::slam::to_lidar_scan(*parsed.cloud);
      if (!scan.ok()) {
        ++skipped;
        progress.update(processed, scans);
        continue;
      }
      const auto pose = core::lookup_pose(scan.scan->stamp_ns, traj);
      if (!pose) {
        ++no_pose;
        progress.update(processed, scans);
        continue;
      }

      const auto & p = *pose;
      const std::array<double, 3> origin{p.tx, p.ty, p.tz};
      std::vector<std::array<float, 3>> world_points;
      world_points.reserve(scan.scan->points.size());
      for (const auto & pt : scan.scan->points) {
        const auto r = rotate_vector(p.qx, p.qy, p.qz, p.qw, pt[0], pt[1], pt[2]);
        world_points.push_back(
          {static_cast<float>(r[0] + p.tx), static_cast<float>(r[1] + p.ty),
           static_cast<float>(r[2] + p.tz)});
      }
      filter.add_scan(origin, std::move(world_points));
      ++scans;
      progress.update(processed, scans);
    }
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(
      kLogger, "read error after %s scans: %s", std::to_string(scans).c_str(), e.what());
    return 1;
  }
  progress.done();

  if (scans == 0) {
    BAGWIZ_LOG_ERROR(kLogger, "No decodable scans on '%s'", args.cloud_topic.c_str());
    return 1;
  }
  if (no_pose > 0) {
    BAGWIZ_LOG_WARN(
      kLogger, "%s scan(s) had no matching trajectory pose and were skipped",
      std::to_string(no_pose).c_str());
  }

  // Run the filter.
  const auto filter_start = std::chrono::steady_clock::now();
  std::vector<char> keep;
  {
    core::slam::FinalizeSpinner spinner("Removing dynamic points", progress_on);
    keep = filter.filter();
  }
  const double filter_seconds =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - filter_start).count();
  const std::size_t removed_count = filter.removed_count();
  const std::size_t reverted_count = filter.reverted_count();

  std::vector<std::array<float, 3>> kept_points;
  std::vector<float> kept_intensities;
  kept_points.reserve(map.points.size());
  const bool with_intensity =
    !map.intensities.empty() && map.intensities.size() == map.points.size();
  if (with_intensity) {
    kept_intensities.reserve(map.intensities.size());
  }
  for (std::size_t i = 0; i < map.points.size(); ++i) {
    if (keep[i] != 0) {
      kept_points.push_back(map.points[i]);
      if (with_intensity) {
        kept_intensities.push_back(map.intensities[i]);
      }
    }
  }

  core::slam::write_pcd(
    out_stream, kept_points,
    with_intensity ? std::span<const float>(kept_intensities.data(), kept_intensities.size())
                   : std::span<const float>{});
  out_stream.close();
  if (!out_stream.good()) {
    BAGWIZ_LOG_ERROR(kLogger, "write failed: %s", out_file.c_str());
    return 1;
  }

  BAGWIZ_LOG_INFO(
    kLogger,
    "Filtered map: kept %zu of %zu points (removed %zu; reverted %zu) from %zu scans in %.1fs -> "
    "%s",
    kept_points.size(), map.points.size(), removed_count, reverted_count, scans, filter_seconds,
    out_file.string().c_str());
  return 0;
}

}  // namespace bagwiz::commands
