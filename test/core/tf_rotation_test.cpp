// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tf_rotation.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace
{

using bagwiz::core::euler_deg_to_euler_rad;
using bagwiz::core::euler_rad_to_euler_deg;
using bagwiz::core::euler_rad_to_quat;
using bagwiz::core::EulerAngles;
using bagwiz::core::quat_to_euler_rad;

constexpr double kEps = 1e-9;

TEST(TfRotation, IdentityQuaternionIsZeroEuler)
{
  const auto rpy = quat_to_euler_rad(0.0, 0.0, 0.0, 1.0);
  EXPECT_NEAR(rpy.roll, 0.0, kEps);
  EXPECT_NEAR(rpy.pitch, 0.0, kEps);
  EXPECT_NEAR(rpy.yaw, 0.0, kEps);
}

TEST(TfRotation, NinetyDegYawQuaternionMatchesTaitBryan)
{
  // Quaternion for a +90 deg rotation about Z: (0, 0, sin(45 deg), cos(45 deg))
  const double s = std::sin(M_PI / 4.0);
  const double c = std::cos(M_PI / 4.0);
  const auto rpy = quat_to_euler_rad(0.0, 0.0, s, c);
  EXPECT_NEAR(rpy.roll, 0.0, kEps);
  EXPECT_NEAR(rpy.pitch, 0.0, kEps);
  EXPECT_NEAR(rpy.yaw, M_PI / 2.0, kEps);
}

TEST(TfRotation, NinetyDegRollQuaternionMatchesTaitBryan)
{
  // +90 deg about X: (sin(45 deg), 0, 0, cos(45 deg))
  const double s = std::sin(M_PI / 4.0);
  const double c = std::cos(M_PI / 4.0);
  const auto rpy = quat_to_euler_rad(s, 0.0, 0.0, c);
  EXPECT_NEAR(rpy.roll, M_PI / 2.0, kEps);
  EXPECT_NEAR(rpy.pitch, 0.0, kEps);
  EXPECT_NEAR(rpy.yaw, 0.0, kEps);
}

TEST(TfRotation, RadToDegLandmarkValues)
{
  const auto deg = euler_rad_to_euler_deg({0.0, M_PI / 2.0, M_PI});
  EXPECT_NEAR(deg.roll, 0.0, kEps);
  EXPECT_NEAR(deg.pitch, 90.0, kEps);
  EXPECT_NEAR(deg.yaw, 180.0, kEps);
}

TEST(TfRotation, DegToRadLandmarkValues)
{
  const auto rad = euler_deg_to_euler_rad({0.0, 90.0, 180.0});
  EXPECT_NEAR(rad.roll, 0.0, kEps);
  EXPECT_NEAR(rad.pitch, M_PI / 2.0, kEps);
  EXPECT_NEAR(rad.yaw, M_PI, kEps);
}

TEST(TfRotation, RadDegRoundtripPreservesValue)
{
  const EulerAngles rad{0.123, -0.456, 1.5708};
  const auto back = euler_deg_to_euler_rad(euler_rad_to_euler_deg(rad));
  EXPECT_NEAR(back.roll, rad.roll, kEps);
  EXPECT_NEAR(back.pitch, rad.pitch, kEps);
  EXPECT_NEAR(back.yaw, rad.yaw, kEps);
}

TEST(TfRotation, QuatEulerQuatRoundtripPreservesQuaternion)
{
  // Pick non-trivial RPY clear of gimbal lock (|pitch| != pi/2).
  const EulerAngles rpy{0.3, -0.4, 1.1};
  const auto q0 = euler_rad_to_quat(rpy);
  const auto rpy_back = quat_to_euler_rad(q0[0], q0[1], q0[2], q0[3]);
  const auto q1 = euler_rad_to_quat(rpy_back);
  // Compare quaternions component-wise (the canonical form returned by
  // setRPY is unique, so no need to handle the q vs -q ambiguity here).
  EXPECT_NEAR(q1[0], q0[0], kEps);
  EXPECT_NEAR(q1[1], q0[1], kEps);
  EXPECT_NEAR(q1[2], q0[2], kEps);
  EXPECT_NEAR(q1[3], q0[3], kEps);
}

TEST(TfRotation, QuatRadDegRadQuatRoundtrip)
{
  // Mirrors what `bagwiz tf walk` does internally when the interactive `r`
  // key cycles to euler_deg: quaternion -> rad -> deg, then back via
  // deg -> rad -> quat.
  const EulerAngles rpy{0.2, 0.5, -0.7};
  const auto q0 = euler_rad_to_quat(rpy);
  const auto rad = quat_to_euler_rad(q0[0], q0[1], q0[2], q0[3]);
  const auto deg = euler_rad_to_euler_deg(rad);
  const auto rad_back = euler_deg_to_euler_rad(deg);
  const auto q1 = euler_rad_to_quat(rad_back);
  EXPECT_NEAR(q1[0], q0[0], kEps);
  EXPECT_NEAR(q1[1], q0[1], kEps);
  EXPECT_NEAR(q1[2], q0[2], kEps);
  EXPECT_NEAR(q1[3], q0[3], kEps);
}

}  // namespace
