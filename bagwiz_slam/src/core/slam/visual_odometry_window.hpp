// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef CORE__SLAM__VISUAL_ODOMETRY_WINDOW_HPP_
#define CORE__SLAM__VISUAL_ODOMETRY_WINDOW_HPP_

#include "visual_odometry_grouping.hpp"  // NOLINT(build/include_subdir) src-local header

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

// Src-local, like visual_factors.hpp and visual_odometry_grouping.hpp: the
// rebuilt-window batch-LM core of the camera-only visual-inertial odometry
// (issue #376 Phase 3). Not installed; only visual_odometry.cpp links this
// TU.
namespace bagwiz::core::slam::vio
{

// Per-camera-id extrinsic table (p_imu = T * p_cam) plus the tuning knobs the
// rebuilt-window solver needs. window_span_ns must match the grouping period
// upstream (GroupingBuffer::Config::period_ns): folding predicts IMU poses
// across exactly one such window (see visual_odometry_window.cpp).
struct WindowConfig
{
  std::vector<Eigen::Isometry3d> t_imu_cams;  // per camera_id: p_imu = T * p_cam
  std::int64_t window_span_ns = 100'000'000;  // == grouping period
  double obs_sigma = 1.0e-3;                  // normalized units
  int max_keyframes = 10;
  int min_track_obs = 3;
  double bias_random_walk_sigma = 1.0e-3;
};

// One keyframe leaving the window: its solved state, plus the landmarks
// triangulated from its (and its track-mates') folded observations, colored
// by this keyframe's own view of each track.
struct MarginalizedKeyframe
{
  std::int64_t stamp_ns = 0;
  Eigen::Isometry3d T_world_imu = Eigen::Isometry3d::Identity();
  Eigen::Vector3d v_world_imu = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 6, 1> imu_bias = Eigen::Matrix<double, 6, 1>::Zero();
  std::vector<Eigen::Vector3d> landmarks_world;  // triangulated, world frame
  std::vector<std::array<std::uint8_t, 3>>
    landmark_rgb;  // parallel; from this keyframe's own observation
};

// Rebuilt-window batch smoother: keeps the last `max_keyframes` observation
// groups plus the IMU chain between them and, on every push, rebuilds the
// whole factor graph from scratch (no factor identity, no ISAM2 slots) and
// solves it with Levenberg-Marquardt. See visual_odometry_window.cpp for the
// full design rationale.
//
// All state lives behind a Pimpl. The public interface above needs nothing
// beyond Eigen and this package's own headers, and a member holding the
// per-keyframe preintegrated IMU measurements (gtsam::PreintegratedImuMeasurements,
// declared in gtsam/navigation/ImuFactor.h) would otherwise force every
// consumer of this header to pull in that GTSAM optimization header too, on
// top of glim itself.
class WindowSolver
{
public:
  // Throws std::invalid_argument if config.max_keyframes < 1,
  // config.min_track_obs < 1, or config.window_span_ns <= 0.
  explicit WindowSolver(WindowConfig config);
  ~WindowSolver();

  void insert_imu(
    double stamp, const Eigen::Vector3d & linear_acc, const Eigen::Vector3d & angular_vel);
  void initialize(
    std::int64_t stamp_ns, const Eigen::Isometry3d & T_world_imu,
    const Eigen::Vector3d & v_world_imu, const Eigen::Matrix<double, 6, 1> & imu_bias);
  [[nodiscard]] bool initialized() const;
  // Append one keyframe group, rebuild + LM-solve the window, return the
  // keyframes that left it (oldest first). initialize() must have been
  // called -- if not, this is a no-op returning {} rather than touching an
  // empty window; the first pushed group is anchored at the initialized
  // state.
  [[nodiscard]] std::vector<MarginalizedKeyframe> push_keyframe(const ObservationGroup & group);
  // IMU-propagated pose prediction from the newest keyframe (keyframe gating).
  [[nodiscard]] std::optional<Eigen::Isometry3d> predict_T_world_imu(std::int64_t stamp_ns);
  // Current in-window keyframes, oldest first (drained by get_remaining_frames).
  // Landmarks are triangulated exactly as in marginalization, so a snapshot
  // entry is a valid EstimationFrame source. Non-const: runs a full rebuild
  // + LM solve of the window (same as push_keyframe()) before triangulating,
  // so every entry is consistent even without an intervening push -- callers
  // that don't need a fresh solve (nothing changed since the last push)
  // should avoid calling this on a hot path.
  [[nodiscard]] std::vector<MarginalizedKeyframe> window_snapshot();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace bagwiz::core::slam::vio

#endif  // CORE__SLAM__VISUAL_ODOMETRY_WINDOW_HPP_
