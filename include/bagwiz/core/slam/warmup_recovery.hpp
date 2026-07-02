// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__WARMUP_RECOVERY_HPP_
#define BAGWIZ__CORE__SLAM__WARMUP_RECOVERY_HPP_

#include <Eigen/Geometry>

#include <optional>
#include <span>
#include <vector>

// Backward IMU propagation for the SLAM initialization window.
//
// GLIM's LiDAR-IMU odometry stays silent during IMU initialization (a ~1 s
// LOOSE-init window): it emits no estimation frame until the initial state has
// converged, so the scans captured before that first frame get no pose and the
// exported trajectory has no samples over its opening window. Once the first
// frame arrives, however, its converged state (world pose, world velocity, and
// IMU biases) is a fully-observed boundary condition. Integrating the buffered
// IMU stream *backward* from that boundary — with the biases now known and over
// a short (~1 s) window — recovers accurate poses for the warmup scans.
//
// The symmetric problem exists at the END of a run: the newest scans are still
// inside the odometry smoother window at end-of-sequence and never get
// marginalized into a finalized submap, so the trajectory stops one window short
// of the last input scan. There the boundary is the LAST estimated frame and the
// buffered IMU is integrated *forward* from it (forwardpropagate_imu) to recover
// the trailing "cooldown" scans. Forward and backward share the same strapdown
// kinematics, differing only in integration direction.
//
// This module is the GLIM-free, Eigen-only kinematic core of both recoveries: it
// knows nothing about GLIM, the bag, or the map. The SLAM wrapper
// (cloud_mapper.cpp) buffers the raw IMU + scan stamps, captures the boundary off
// a GLIM EstimationFrame, calls backpropagate_imu() / forwardpropagate_imu(), and
// re-anchors the result onto the globally-optimized map. Keeping the math here
// lets it be unit-tested with synthetic IMU (static / rotating / translating)
// without the GLIM stack.
namespace bagwiz::core::slam
{

// One raw IMU measurement (before bias removal), matching sensor_msgs/Imu
// convention: linear_acceleration in m/s^2 (specific force, i.e. gravity
// reaction included), angular_velocity in rad/s. Stamp in seconds.
struct BackpropImu
{
  double stamp = 0.0;
  Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
};

// The converged odometry state at the first estimated frame — the boundary the
// warmup window is propagated back from. Pose and velocity are in GLIM's world
// (odometry) frame for the IMU body; the biases are GLIM's imu_bias split into
// its accelerometer (first three) and gyroscope (last three) parts.
struct BackpropBoundary
{
  double stamp = 0.0;  // seconds; the first frame's timestamp
  Eigen::Isometry3d T_world_imu = Eigen::Isometry3d::Identity();
  Eigen::Vector3d v_world_imu = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
};

// A world-frame IMU body pose at a timestamp (seconds).
struct TimedPose
{
  double stamp = 0.0;
  Eigen::Isometry3d T_world_imu = Eigen::Isometry3d::Identity();
};

// GLIM's default gravity magnitude (gtsam PreintegrationParams::MakeSharedU,
// selected when the sensor is "upright" — GLIM's default). The world frame is
// Z-up, so gravity acts along -Z: gravity_world = {0, 0, -kGravityMagnitude}.
inline constexpr double kGravityMagnitude = 9.81;

// The gravity acceleration vector in GLIM's default (Z-up) world frame.
inline Eigen::Vector3d default_gravity_world()
{
  return {0.0, 0.0, -kGravityMagnitude};
}

// Back-propagate the IMU body pose from `boundary` backward in time over the
// IMU samples in `imu_window` that precede it. `imu_window` must be sorted by
// ascending stamp; only samples with stamp strictly less than boundary.stamp
// are integrated (later samples are ignored). Each interval uses zero-order
// hold on its earlier sample and the known biases — a first-order backward
// approximation of the strapdown kinematics. (GLIM bootstraps this window with a
// batch factor-graph estimate, not a single forward integration, so this
// reconstructs the pre-init motion rather than exactly inverting GLIM's scheme;
// over a ~1 s window with known biases the truncation error is negligible.):
//
//   a = R * (linear_acceleration - acc_bias) + gravity_world
//   R(t)   = R(t+dt) * Exp(-(gyro - gyro_bias) * dt)
//   v(t)   = v(t+dt) - a * dt
//   p(t)   = p(t+dt) - v(t) * dt - 0.5 * a * dt^2
//
// Returns the IMU-rate poses at each integrated sample stamp plus the boundary
// itself, ascending by stamp (so the last element is the boundary). With no
// preceding samples the result is just the boundary knot.
std::vector<TimedPose> backpropagate_imu(
  const BackpropBoundary & boundary, std::span<const BackpropImu> imu_window,
  const Eigen::Vector3d & gravity_world = {0.0, 0.0, -kGravityMagnitude});

// Forward-propagate the IMU body pose from `boundary` forward in time over the
// IMU samples in `imu_window` that follow it. `imu_window` must be sorted by
// ascending stamp; only samples with stamp strictly greater than boundary.stamp
// are integrated (earlier samples are ignored, so the caller may pass a trailing
// ring that still contains pre-boundary samples). Each interval uses zero-order
// hold on its arriving (later) sample and the known biases — a first-order
// forward strapdown integration, the mirror of backpropagate_imu:
//
//   a = R * (linear_acceleration - acc_bias) + gravity_world
//   p(t+dt) = p(t) + v(t) * dt + 0.5 * a * dt^2
//   v(t+dt) = v(t) + a * dt
//   R(t+dt) = R(t) * Exp((gyro - gyro_bias) * dt)
//
// Returns the IMU-rate poses at the boundary plus each integrated sample stamp,
// ascending by stamp (so the FIRST element is the boundary). With no following
// samples the result is just the boundary knot.
std::vector<TimedPose> forwardpropagate_imu(
  const BackpropBoundary & boundary, std::span<const BackpropImu> imu_window,
  const Eigen::Vector3d & gravity_world = {0.0, 0.0, -kGravityMagnitude});

// Interpolate an IMU pose at `stamp` from `knots` (ascending by stamp): position
// linearly, orientation by shortest-path SLERP. Returns std::nullopt when
// `knots` is empty or `stamp` lies outside [front.stamp, back.stamp] — the
// warmup window is never extrapolated beyond the integrated span.
std::optional<Eigen::Isometry3d> interpolate_pose(std::span<const TimedPose> knots, double stamp);

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__WARMUP_RECOVERY_HPP_
