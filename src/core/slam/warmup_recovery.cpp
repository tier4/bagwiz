// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/warmup_recovery.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <span>
#include <vector>

namespace bagwiz::core::slam
{
namespace
{

// SO(3) exponential of a rotation vector (axis * angle). Below this angle the
// rotation is treated as identity, avoiding a divide-by-near-zero in the
// axis normalization (the first-order term is already sub-micro-radian there).
constexpr double kMinRotationAngle = 1e-12;

Eigen::Matrix3d so3_exp(const Eigen::Vector3d & phi)
{
  const double angle = phi.norm();
  if (angle < kMinRotationAngle) {
    return Eigen::Matrix3d::Identity();
  }
  return Eigen::AngleAxisd(angle, phi / angle).toRotationMatrix();
}

}  // namespace

std::vector<TimedPose> backpropagate_imu(
  const BackpropBoundary & boundary, std::span<const BackpropImu> imu_window,
  const Eigen::Vector3d & gravity_world)
{
  // State at the boundary (the integration's initial condition), stepped
  // backward one IMU interval at a time.
  Eigen::Matrix3d R = boundary.T_world_imu.rotation();
  Eigen::Vector3d p = boundary.T_world_imu.translation();
  Eigen::Vector3d v = boundary.v_world_imu;
  double t_cur = boundary.stamp;

  // Built newest-first (boundary, then each earlier sample), reversed to
  // ascending before returning so the boundary is the last knot.
  std::vector<TimedPose> knots;
  knots.reserve(imu_window.size() + 1);
  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = R;
    pose.translation() = p;
    knots.push_back({boundary.stamp, pose});
  }

  // Highest index of a sample strictly before the boundary. imu_window is
  // ascending, so this is a suffix-trim. The caller buffers IMU until GLIM
  // marginalizes its first frame (~5 s past the boundary), so this routinely
  // drops a large post-boundary tail whose scans GLIM already estimated.
  std::size_t count = imu_window.size();
  while (count > 0 && imu_window[count - 1].stamp >= boundary.stamp) {
    --count;
  }

  for (std::size_t k = count; k > 0; --k) {
    const BackpropImu & s = imu_window[k - 1];
    const double dt = t_cur - s.stamp;
    if (dt <= 0.0) {
      // Non-increasing stamps (duplicate or out-of-order) yield no interval;
      // skip defensively rather than integrate a zero/negative step.
      continue;
    }

    // Zero-order hold on this (earlier) sample governs the interval [s.stamp,
    // t_cur]. Rotation integrates independently of acceleration, so recover the
    // earlier orientation first, then evaluate the world acceleration with it.
    const Eigen::Vector3d omega = s.angular_velocity - boundary.gyro_bias;
    const Eigen::Matrix3d R_prev = R * so3_exp(-omega * dt);
    const Eigen::Vector3d a = R_prev * (s.linear_acceleration - boundary.acc_bias) + gravity_world;

    const Eigen::Vector3d v_prev = v - a * dt;
    const Eigen::Vector3d p_prev = p - v_prev * dt - 0.5 * a * dt * dt;

    R = R_prev;
    v = v_prev;
    p = p_prev;
    t_cur = s.stamp;

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = R;
    pose.translation() = p;
    knots.push_back({s.stamp, pose});
  }

  std::reverse(knots.begin(), knots.end());
  return knots;
}

std::vector<TimedPose> forwardpropagate_imu(
  const BackpropBoundary & boundary, std::span<const BackpropImu> imu_window,
  const Eigen::Vector3d & gravity_world)
{
  // State at the boundary (the integration's initial condition), stepped
  // forward one IMU interval at a time. The mirror of backpropagate_imu.
  Eigen::Matrix3d R = boundary.T_world_imu.rotation();
  Eigen::Vector3d p = boundary.T_world_imu.translation();
  Eigen::Vector3d v = boundary.v_world_imu;
  double t_cur = boundary.stamp;

  // Built ascending from the boundary, so the boundary is the first knot and no
  // final reverse is needed (unlike backpropagate_imu).
  std::vector<TimedPose> knots;
  knots.reserve(imu_window.size() + 1);
  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = R;
    pose.translation() = p;
    knots.push_back({boundary.stamp, pose});
  }

  // Lowest index of a sample strictly after the boundary. imu_window is
  // ascending, so this is a prefix-trim. The caller buffers a trailing ring that
  // still holds pre-boundary samples (the boundary is only known at end of
  // sequence), so this routinely drops a pre-boundary head.
  std::size_t start = 0;
  while (start < imu_window.size() && imu_window[start].stamp <= boundary.stamp) {
    ++start;
  }

  for (std::size_t k = start; k < imu_window.size(); ++k) {
    const BackpropImu & s = imu_window[k];
    const double dt = s.stamp - t_cur;
    if (dt <= 0.0) {
      // Non-increasing stamps (duplicate or out-of-order) yield no interval;
      // skip defensively rather than integrate a zero/negative step.
      continue;
    }

    // Zero-order hold on this (arriving) sample governs the interval [t_cur,
    // s.stamp]. Evaluate the world acceleration with the current (start-of-
    // interval) orientation, advance position/velocity, then advance the
    // orientation — the exact forward mirror of the backward step.
    const Eigen::Vector3d omega = s.angular_velocity - boundary.gyro_bias;
    const Eigen::Vector3d a = R * (s.linear_acceleration - boundary.acc_bias) + gravity_world;

    const Eigen::Vector3d p_next = p + v * dt + 0.5 * a * dt * dt;
    const Eigen::Vector3d v_next = v + a * dt;
    const Eigen::Matrix3d R_next = R * so3_exp(omega * dt);

    R = R_next;
    v = v_next;
    p = p_next;
    t_cur = s.stamp;

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = R;
    pose.translation() = p;
    knots.push_back({s.stamp, pose});
  }

  return knots;
}

std::optional<Eigen::Isometry3d> interpolate_pose(std::span<const TimedPose> knots, double stamp)
{
  if (knots.empty()) {
    return std::nullopt;
  }
  const double t_front = knots.front().stamp;
  const double t_back = knots.back().stamp;
  if (stamp < t_front || stamp > t_back) {
    return std::nullopt;  // never extrapolate outside the integrated span
  }

  // First knot at or after `stamp` (lower_bound over the ascending stamps).
  const auto upper = std::lower_bound(
    knots.begin(), knots.end(), stamp,
    [](const TimedPose & knot, double value) { return knot.stamp < value; });

  if (upper == knots.begin() || upper->stamp == stamp) {
    return upper->T_world_imu;  // exact hit (incl. the front knot)
  }

  const TimedPose & lo = *(upper - 1);
  const TimedPose & hi = *upper;
  const double span = hi.stamp - lo.stamp;
  const double alpha = span > 0.0 ? (stamp - lo.stamp) / span : 0.0;

  const Eigen::Quaterniond q_lo(lo.T_world_imu.rotation());
  const Eigen::Quaterniond q_hi(hi.T_world_imu.rotation());
  const Eigen::Vector3d t_lo = lo.T_world_imu.translation();
  const Eigen::Vector3d t_hi = hi.T_world_imu.translation();

  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.linear() = q_lo.slerp(alpha, q_hi).normalized().toRotationMatrix();
  out.translation() = t_lo + alpha * (t_hi - t_lo);
  return out;
}

}  // namespace bagwiz::core::slam
