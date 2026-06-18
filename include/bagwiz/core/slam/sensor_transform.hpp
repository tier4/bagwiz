// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__SENSOR_TRANSFORM_HPP_
#define BAGWIZ__CORE__SLAM__SENSOR_TRANSFORM_HPP_

#include <array>

// A GLIM/Eigen-free rigid transform used to carry a sensor extrinsic across the
// pimpl seam into the SLAM odometry/mapping wrappers. The command layer resolves
// the LiDAR↔IMU extrinsic from the bag's static TF (tf2) and hands it over as
// this plain POD; cloud_odometry.cpp / cloud_mapper.cpp convert it to an
// Eigen::Isometry3d internally, keeping the public headers free of GLIM/Eigen.
//
// Convention matches GLIM's `T_lidar_imu` (p_lidar = T_lidar_imu * p_imu): the
// transform maps a point expressed in the IMU frame into the LiDAR/cloud frame.
namespace bagwiz::core::slam
{

struct SensorTransform
{
  std::array<double, 3> translation{0.0, 0.0, 0.0};         // (x, y, z) meters
  std::array<double, 4> rotation_xyzw{0.0, 0.0, 0.0, 1.0};  // quaternion (x, y, z, w)
};

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__SENSOR_TRANSFORM_HPP_
