// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef SLAM__VISUAL_FACTORS_HPP_
#define SLAM__VISUAL_FACTORS_HPP_

#include "bagwiz/core/slam/visual_observation.hpp"

#include <Eigen/Geometry>
#include <gtsam_points/types/point_cloud.hpp>

#include <gtsam/nonlinear/NonlinearFactor.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

// Src-local: builds gtsam smart-projection factors from VisualObservation
// tracks, associating each observation with the submap whose LiDAR trajectory
// covers its stamp. Not installed; only cloud_mapper.cpp (via bagwiz_slam_glim)
// links this TU.
namespace bagwiz::core::slam::visual
{

// A snapshot of one GLIM submap as visual factor construction needs it: its
// current world pose (the triangulation seed), the merged submap cloud for
// the LiDAR-support gate, and the per-frame LiDAR trajectory (in the submap's
// own origin frame, so it stays valid across global-mapping re-optimization).
struct SubmapView
{
  std::uint64_t id = 0;                            // gtsam key = X(id)
  Eigen::Isometry3d T_world_origin;                // current estimate (triangulation seed)
  std::vector<double> frame_stamps;                // seconds, ascending
  std::vector<Eigen::Isometry3d> T_origin_frames;  // parallel: LiDAR pose in origin frame
  const gtsam_points::PointCloud * cloud =
    nullptr;  // merged submap cloud (origin frame); may be null
};

struct Params
{
  double obs_sigma = 1.0e-3;
  int max_obs_per_track = 16;
  double gate_distance = 1.0;  // <= 0 disables the LiDAR gate
};

struct Stats
{
  std::size_t factors = 0;
  std::size_t tracks_total = 0;
  std::size_t tracks_single_submap = 0;  // dropped: constrain nothing
  std::size_t tracks_too_short = 0;      // dropped: < 3 associated observations
  std::size_t tracks_triangulation_failed = 0;
  std::size_t tracks_gated = 0;  // dropped by the LiDAR-support gate
};

// Interpolate the LiDAR pose at stamp t (seconds) inside `view`'s frame span:
// SE(3) between bracketing frames (slerp rotation, lerp translation). Returns
// nullopt when t is outside [frame_stamps.front(), frame_stamps.back()].
[[nodiscard]] std::optional<Eigen::Isometry3d> interpolate_origin_pose(
  const SubmapView & view, double t);

// Find the submap whose frame span contains t; nullopt in inter-submap gaps.
// `views` must be sorted by frame_stamps.front().
[[nodiscard]] std::optional<std::size_t> submap_for_stamp(
  std::span<const SubmapView> views, double t);

[[nodiscard]] Stats build_visual_factors(
  std::span<const VisualObservation> observations,
  std::span<const Eigen::Isometry3d> t_lidar_cams,  // by camera_id
  std::span<const SubmapView> submaps, const Params & params,
  std::vector<gtsam::NonlinearFactor::shared_ptr> & out);

}  // namespace bagwiz::core::slam::visual

#endif  // SLAM__VISUAL_FACTORS_HPP_
