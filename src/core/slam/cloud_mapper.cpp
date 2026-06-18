// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/cloud_mapper.hpp"

#include "bagwiz/core/slam/lidar_scan.hpp"
#include "bagwiz/core/trajectory.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glim/mapping/global_mapping.hpp>
#include <glim/mapping/sub_map.hpp>
#include <glim/mapping/sub_mapping.hpp>
#include <glim/odometry/estimation_frame.hpp>
#include <glim/odometry/odometry_estimation_ct.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/raw_points.hpp>
#include <glim/util/time_keeper.hpp>
#include <gtsam_points/types/point_cloud.hpp>

#include <spdlog/spdlog.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <utility>
#include <vector>

namespace bagwiz::core::slam
{
namespace
{
core::TrajectoryPose to_pose(double stamp, const Eigen::Isometry3d & transform)
{
  const Eigen::Vector3d translation = transform.translation();
  Eigen::Quaterniond rotation(transform.rotation());
  rotation.normalize();

  core::TrajectoryPose pose;
  pose.timestamp_ns = static_cast<std::int64_t>(std::llround(stamp * 1e9));
  pose.tx = translation.x();
  pose.ty = translation.y();
  pose.tz = translation.z();
  pose.qx = rotation.x();
  pose.qy = rotation.y();
  pose.qz = rotation.z();
  pose.qw = rotation.w();
  return pose;
}

// We feed LiDAR only (OdometryEstimationCT is LiDAR-only CT-GICP) and never call
// insert_imu, so disable IMU in both mapping stages — the GLIM defaults enable
// it, which would make insert_submap build IMU factors from uninitialised
// bias/velocity and warn that frames are not IMU-framed.
glim::SubMappingParams make_sub_mapping_params()
{
  glim::SubMappingParams params;
  params.enable_imu = false;
  return params;
}
glim::GlobalMappingParams make_global_mapping_params()
{
  glim::GlobalMappingParams params;
  params.enable_imu = false;
  return params;
}
}  // namespace

struct CloudMapper::Impl
{
  // One frame of a submap, captured BEFORE the submap is inserted into global
  // mapping (which overwrites SubMap::T_world_origin with the chained global
  // estimate). T_origin_frame is the frame's pose relative to the submap origin
  // and is invariant under global optimization; the optimized world pose is
  // recovered later as (optimized T_world_origin) * T_origin_frame.
  struct FrameRef
  {
    double stamp = 0.0;
    Eigen::Isometry3d T_origin_frame = Eigen::Isometry3d::Identity();
  };
  struct SubMapEntry
  {
    glim::SubMap::Ptr submap;  // kept so its T_world_origin can be read post-optimize
    std::vector<FrameRef> frames;
  };

  glim::TimeKeeper time_keeper;
  glim::CloudPreprocessor preprocessor;
  glim::OdometryEstimationCT odometry;
  std::unique_ptr<glim::SubMapping> sub_mapping;
  std::unique_ptr<glim::GlobalMapping> global_mapping;
  std::vector<SubMapEntry> entries;

  Impl()
  : sub_mapping(std::make_unique<glim::SubMapping>(make_sub_mapping_params())),
    global_mapping(std::make_unique<glim::GlobalMapping>(make_global_mapping_params()))
  {
  }

  // Capture each frame's submap-local relative pose, then hand the submap to
  // global mapping. Order matters: insert_submap rewrites T_world_origin and
  // drops the per-frame point clouds, so the relative poses are read first.
  void capture_and_insert(const glim::SubMap::Ptr & submap)
  {
    SubMapEntry entry;
    entry.submap = submap;
    const Eigen::Isometry3d T_origin_world = submap->T_world_origin.inverse();
    entry.frames.reserve(submap->frames.size());
    for (const auto & frame : submap->frames) {
      if (!frame) {
        continue;
      }
      FrameRef ref;
      ref.stamp = frame->stamp;
      ref.T_origin_frame = T_origin_world * frame->T_world_lidar;
      entry.frames.push_back(ref);
    }
    entries.push_back(std::move(entry));
    global_mapping->insert_submap(submap);
  }

  void drain_submaps()
  {
    for (const auto & submap : sub_mapping->get_submaps()) {
      if (submap) {
        capture_and_insert(submap);
      }
    }
  }
};

CloudMapper::CloudMapper()
{
  // GLIM logs ~50 lines of "config file not found / using default value" while
  // its modules read parameters at construction (we drive GLIM with no config
  // directory on purpose — its built-in defaults are what we want). Silence that
  // one-time startup chatter; genuine runtime warnings still surface afterwards.
  // The level is restored via RAII so a throwing GLIM constructor cannot leave
  // the shared logger muted for the rest of the process.
  const auto logger = glim::get_default_logger();
  struct LevelGuard
  {
    spdlog::logger & logger;
    spdlog::level::level_enum level;
    ~LevelGuard() { logger.set_level(level); }
  } guard{*logger, logger->level()};
  logger->set_level(spdlog::level::off);
  impl_ = std::make_unique<Impl>();
}
CloudMapper::~CloudMapper() = default;
CloudMapper::CloudMapper(CloudMapper &&) noexcept = default;
CloudMapper & CloudMapper::operator=(CloudMapper &&) noexcept = default;

void CloudMapper::insert(const LidarScan & scan)
{
  auto raw = std::make_shared<glim::RawPoints>();
  raw->stamp = static_cast<double>(scan.stamp_ns) * 1e-9;

  const std::size_t num_points = scan.points.size();
  raw->points.reserve(num_points);
  for (const auto & point : scan.points) {
    raw->points.emplace_back(point[0], point[1], point[2], 1.0);
  }
  if (!scan.intensities.empty()) {
    raw->intensities = scan.intensities;
  }

  // A time-less cloud is fed explicit zero per-point times (already
  // motion-undistorted), NOT an empty vector — that would make glim::TimeKeeper
  // synthesize order-based pseudo times and wrongly "deskew" a concatenated
  // cloud. (Mirrors CloudOdometry::insert.)
  if (scan.has_per_point_time && scan.times.size() == num_points) {
    raw->times = scan.times;
  } else {
    raw->times.assign(num_points, 0.0);
  }

  if (!impl_->time_keeper.process(raw)) {
    return;
  }

  const auto preprocessed = impl_->preprocessor.preprocess(raw);
  std::vector<glim::EstimationFrame::ConstPtr> marginalized;
  impl_->odometry.insert_frame(preprocessed, marginalized);

  // The marginalized odometry frames are this mapper's input to sub mapping.
  for (const auto & frame : marginalized) {
    if (frame) {
      impl_->sub_mapping->insert_frame(frame);
    }
  }
  impl_->drain_submaps();
}

CloudMap CloudMapper::finish()
{
  // Flush the odometry smoother window — the remaining frames are marginalized
  // exactly as glim's async pipeline does at end of sequence — into sub mapping,
  // then force out the final submap.
  for (const auto & frame : impl_->odometry.get_remaining_frames()) {
    if (frame) {
      impl_->sub_mapping->insert_frame(frame);
    }
  }
  // This drain only sees submaps the flushed remaining frames newly completed —
  // get_submaps() destructively swaps its queue, so the submaps drained during
  // insert() are already gone. submit_end_of_sequence() then forces a final
  // submap out of whatever odometry frames remain; it builds a fresh submap
  // rather than pulling from that queue, so there is no overlap.
  impl_->drain_submaps();
  for (const auto & submap : impl_->sub_mapping->submit_end_of_sequence()) {
    if (submap) {
      impl_->capture_and_insert(submap);
    }
  }

  // Heavy step: global matching-based iSAM2 optimization. Updates each held
  // submap's T_world_origin in place (GlobalMapping::update_submaps).
  impl_->global_mapping->optimize();

  CloudMap result;

  // Trajectory: optimized world pose per frame = T_world_origin * T_origin_frame.
  // Keyed by timestamp so the poses come out time-ordered and a duplicate stamp
  // (submap boundaries share no frames, but stay defensive) keeps one entry.
  std::map<std::int64_t, core::TrajectoryPose> poses;
  for (const auto & entry : impl_->entries) {
    if (!entry.submap) {
      continue;
    }
    const Eigen::Isometry3d & T_world_origin = entry.submap->T_world_origin;
    for (const auto & ref : entry.frames) {
      const Eigen::Isometry3d T_world_frame = T_world_origin * ref.T_origin_frame;
      const auto pose = to_pose(ref.stamp, T_world_frame);
      poses[pose.timestamp_ns] = pose;
    }
  }
  result.trajectory.reserve(poses.size());
  for (const auto & entry : poses) {
    result.trajectory.push_back(entry.second);
  }

  // Map: GlobalMapping::export_points concatenates every submap's frame points
  // transformed by its optimized T_world_origin, i.e. the world-frame map.
  const auto cloud = impl_->global_mapping->export_points();
  if (cloud && cloud->size() > 0) {
    const std::size_t n = cloud->size();
    const bool has_intensities = cloud->has_intensities();
    result.points.reserve(n);
    if (has_intensities) {
      result.intensities.reserve(n);
    }
    for (std::size_t i = 0; i < n; ++i) {
      const Eigen::Vector4d & p = cloud->points[i];
      result.points.push_back(
        {static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z())});
      if (has_intensities) {
        result.intensities.push_back(static_cast<float>(cloud->intensities[i]));
      }
    }
  }

  return result;
}

}  // namespace bagwiz::core::slam
