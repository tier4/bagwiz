// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/warmup_recovery.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace bagwiz::core::slam
{
namespace
{

// Gravity reaction (specific force) a stationary, gravity-aligned IMU reads:
// acc_meas = R^T (a_world - g_world), with a_world = 0 and R = I gives
// -g_world = +{0,0,g}. Used to build synthetic windows whose true world
// acceleration is zero (static / constant-velocity / z-rotation cases).
Eigen::Vector3d gravity_reaction()
{
  return {0.0, 0.0, kGravityMagnitude};
}

// Build an ascending IMU window of `n` samples spaced `dt`, ending just before
// `boundary_stamp` (last sample at boundary_stamp - dt), all carrying the same
// acc/gyro. Sample i sits at boundary_stamp - (n - i) * dt.
std::vector<BackpropImu> make_window(
  double boundary_stamp, int n, double dt, const Eigen::Vector3d & acc,
  const Eigen::Vector3d & gyro)
{
  std::vector<BackpropImu> window;
  window.reserve(n);
  for (int i = 0; i < n; ++i) {
    BackpropImu s;
    s.stamp = boundary_stamp - static_cast<double>(n - i) * dt;
    s.linear_acceleration = acc;
    s.angular_velocity = gyro;
    window.push_back(s);
  }
  return window;
}

TEST(WarmupRecovery, StaticBoundaryStaysPut)
{
  BackpropBoundary boundary;
  boundary.stamp = 10.0;  // T_world_imu = I, v = 0

  const auto window = make_window(10.0, 200, 0.005, gravity_reaction(), Eigen::Vector3d::Zero());
  const auto knots = backpropagate_imu(boundary, window, default_gravity_world());

  ASSERT_EQ(knots.size(), window.size() + 1);
  // A stationary IMU stays at the origin with identity orientation over the
  // whole (1 s) window.
  for (const auto & knot : knots) {
    EXPECT_LT(knot.T_world_imu.translation().norm(), 1e-9);
    const Eigen::AngleAxisd aa(knot.T_world_imu.rotation());
    EXPECT_LT(aa.angle(), 1e-9);
  }
  // Ascending, boundary last.
  EXPECT_DOUBLE_EQ(knots.back().stamp, 10.0);
  EXPECT_LT(knots.front().stamp, knots.back().stamp);
}

TEST(WarmupRecovery, ConstantVelocityTranslatesBack)
{
  BackpropBoundary boundary;
  boundary.stamp = 5.0;
  boundary.v_world_imu = Eigen::Vector3d(2.0, 0.0, 0.0);  // 2 m/s along +x, no rotation

  // a_world = 0 (constant velocity) => acc reads the gravity reaction, gyro 0.
  const auto window = make_window(5.0, 100, 0.01, gravity_reaction(), Eigen::Vector3d::Zero());
  const auto knots = backpropagate_imu(boundary, window, default_gravity_world());

  // Earliest knot is 1 s before the boundary; position should be v*Δt behind:
  // p(t) = p0 - v0 * (boundary - t) = (-2.0, 0, 0).
  const auto & first = knots.front();
  const double dt_back = boundary.stamp - first.stamp;
  EXPECT_NEAR(dt_back, 1.0, 1e-9);
  EXPECT_NEAR(first.T_world_imu.translation().x(), -2.0, 1e-6);
  EXPECT_NEAR(first.T_world_imu.translation().y(), 0.0, 1e-9);
  EXPECT_NEAR(first.T_world_imu.translation().z(), 0.0, 1e-9);
  const Eigen::AngleAxisd aa(first.T_world_imu.rotation());
  EXPECT_LT(aa.angle(), 1e-9);
}

TEST(WarmupRecovery, ConstantYawRateRotatesBack)
{
  const double wz = 0.5;  // rad/s about +z
  BackpropBoundary boundary;
  boundary.stamp = 3.0;  // R = I, v = 0

  // Rotation about the gravity (z) axis keeps the IMU gravity-aligned, so the
  // accelerometer still reads the pure gravity reaction and world accel is 0.
  const auto window =
    make_window(3.0, 100, 0.01, gravity_reaction(), Eigen::Vector3d(0.0, 0.0, wz));
  const auto knots = backpropagate_imu(boundary, window, default_gravity_world());

  // 1 s before the boundary the orientation is Exp(-wz * 1 s) about +z.
  const auto & first = knots.front();
  const double dt_back = boundary.stamp - first.stamp;
  EXPECT_NEAR(dt_back, 1.0, 1e-9);
  const Eigen::AngleAxisd aa(first.T_world_imu.rotation());
  EXPECT_NEAR(aa.angle(), wz * dt_back, 1e-6);
  // Axis is -z (angle is positive, so the axis carries the sign).
  EXPECT_NEAR(aa.axis().z(), -1.0, 1e-6);
  EXPECT_LT(first.T_world_imu.translation().norm(), 1e-6);
}

TEST(WarmupRecovery, EmptyWindowReturnsBoundaryOnly)
{
  BackpropBoundary boundary;
  boundary.stamp = 1.0;
  boundary.T_world_imu.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);

  const auto knots = backpropagate_imu(boundary, {}, default_gravity_world());
  ASSERT_EQ(knots.size(), 1u);
  EXPECT_DOUBLE_EQ(knots.front().stamp, 1.0);
  EXPECT_NEAR(
    (knots.front().T_world_imu.translation() - Eigen::Vector3d(1, 2, 3)).norm(), 0.0, 1e-12);
}

TEST(WarmupRecovery, SamplesAtOrAfterBoundaryAreIgnored)
{
  BackpropBoundary boundary;
  boundary.stamp = 2.0;

  // Two samples before, two at/after the boundary — only the earlier two count.
  std::vector<BackpropImu> window;
  for (double t : {1.98, 1.99, 2.00, 2.01}) {
    BackpropImu s;
    s.stamp = t;
    s.linear_acceleration = gravity_reaction();
    window.push_back(s);
  }
  const auto knots = backpropagate_imu(boundary, window, default_gravity_world());
  ASSERT_EQ(knots.size(), 3u);  // boundary + 2 preceding
  EXPECT_DOUBLE_EQ(knots.back().stamp, 2.0);
}

TEST(WarmupRecovery, InterpolatePoseWithinSpan)
{
  std::vector<TimedPose> knots(3);
  knots[0].stamp = 0.0;
  knots[1].stamp = 1.0;
  knots[1].T_world_imu.translation() = Eigen::Vector3d(2.0, 0.0, 0.0);
  knots[2].stamp = 2.0;
  knots[2].T_world_imu.translation() = Eigen::Vector3d(4.0, 0.0, 0.0);

  const auto mid = interpolate_pose(knots, 0.5);
  ASSERT_TRUE(mid.has_value());
  EXPECT_NEAR(mid->translation().x(), 1.0, 1e-9);

  const auto exact = interpolate_pose(knots, 1.0);
  ASSERT_TRUE(exact.has_value());
  EXPECT_NEAR(exact->translation().x(), 2.0, 1e-9);

  const auto at_front = interpolate_pose(knots, 0.0);
  ASSERT_TRUE(at_front.has_value());
  EXPECT_NEAR(at_front->translation().x(), 0.0, 1e-9);
}

TEST(WarmupRecovery, InterpolatePoseOutsideSpanIsNullopt)
{
  std::vector<TimedPose> knots(2);
  knots[0].stamp = 0.0;
  knots[1].stamp = 1.0;

  EXPECT_FALSE(interpolate_pose(knots, -0.1).has_value());
  EXPECT_FALSE(interpolate_pose(knots, 1.1).has_value());
  EXPECT_FALSE(interpolate_pose({}, 0.5).has_value());
}

}  // namespace
}  // namespace bagwiz::core::slam
