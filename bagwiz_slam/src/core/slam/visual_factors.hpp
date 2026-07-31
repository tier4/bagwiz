// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef CORE__SLAM__VISUAL_FACTORS_HPP_
#define CORE__SLAM__VISUAL_FACTORS_HPP_

#include "bagwiz/core/slam/visual_observation.hpp"

#include <Eigen/Geometry>
#include <gtsam_points/types/point_cloud.hpp>

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholePose.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/slam/SmartFactorParams.h>
#include <gtsam/slam/SmartProjectionRigFactor.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <utility>
#include <vector>

// Src-local: builds gtsam smart-projection factors from VisualObservation
// tracks, associating each observation with the submap whose LiDAR trajectory
// covers its stamp. Not installed; only cloud_mapper.cpp (via bagwiz_slam_glim)
// links this TU.
namespace bagwiz::core::slam::visual
{

// The smart-factor recipe shared by the global co-visibility factors and the
// odometry-window factors (visual_odometry_window.cpp). PinholePose is
// mandatory: SmartProjectionRigFactor static-asserts the camera's pose-only
// dimension; PinholeCamera does not compile against it.
using RigCamera = gtsam::PinholePose<gtsam::Cal3_S2>;
using RigFactor = gtsam::SmartProjectionRigFactor<RigCamera>;

// Identity pinhole: observations are undistorted normalized coordinates, so
// the calibration is (fx, fy, s, cx, cy) = (1, 1, 0, 0, 0) and every
// image-plane quantity below (sigma, rank tolerance, reprojection error) is
// in normalized units.
[[nodiscard]] gtsam::Cal3_S2::shared_ptr normalized_calibration();

// The frozen SmartProjectionParams recipe (HESSIAN + ZERO_ON_DEGENERACY,
// normalized-units rank tolerance). See the definition for why each value
// is load-bearing and what must never be added.
[[nodiscard]] gtsam::SmartProjectionParams make_smart_projection_params();

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
  int max_obs_per_track = 16;  // <= 0 keeps every observation (no cap)
  double gate_distance = 1.0;  // <= 0 disables the LiDAR gate
};

// What identifies one tracked feature. VisualObservation::track_id is unique
// only WITHIN a camera — every VisualFrontend instance numbers its own tracks
// from 0 — so anything grouping observations into tracks must key on the pair.
// Keying on track_id alone silently fuses unrelated features from different
// cameras into one track, which then triangulates to garbage.
using TrackKey = std::pair<std::int32_t, std::uint64_t>;  // (camera_id, track_id)

struct TrackKeyHash
{
  std::size_t operator()(const TrackKey & key) const noexcept
  {
    // Mix rather than xor: camera ids are tiny and track ids dense, so xor
    // would pile every camera's track n into one bucket.
    const std::size_t track = std::hash<std::uint64_t>{}(key.second);
    const std::size_t camera = std::hash<std::int32_t>{}(key.first);
    return track ^ (camera + 0x9e3779b97f4a7c15ULL + (track << 6) + (track >> 2));
  }
};

[[nodiscard]] inline TrackKey track_key(const VisualObservation & obs)
{
  return {obs.camera_id, obs.track_id};
}

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

#endif  // CORE__SLAM__VISUAL_FACTORS_HPP_
