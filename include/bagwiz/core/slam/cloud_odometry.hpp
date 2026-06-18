// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__CLOUD_ODOMETRY_HPP_
#define BAGWIZ__CORE__SLAM__CLOUD_ODOMETRY_HPP_

#include "bagwiz/core/slam/imu_sample.hpp"
#include "bagwiz/core/slam/lidar_scan.hpp"
#include "bagwiz/core/slam/sensor_transform.hpp"
#include "bagwiz/core/trajectory.hpp"

#include <memory>
#include <optional>
#include <vector>

// Odometry over a sequence of scans, wrapping a GLIM estimator. With no IMU it
// uses the LiDAR-only OdometryEstimationCT; when constructed with a LiDAR↔IMU
// extrinsic it uses the LiDAR-IMU OdometryEstimationCPU and accepts IMU samples
// via insert_imu(). The GLIM / Eigen / GTSAM types are hidden behind a pimpl so
// this header — and therefore the `slam` command that drives it — stays free of
// GLIM includes; only cloud_odometry.cpp pulls GLIM in. The whole translation
// unit is compiled only when BAGWIZ_WITH_SLAM is on.
//
// Usage: feed scans (and, in IMU mode, IMU samples) in timestamp order with
// insert()/insert_imu(), then call finish() once to flush the remaining frames
// and obtain the full estimated trajectory. No ROS node / pub-sub is involved —
// GLIM's modules are called directly.
namespace bagwiz::core::slam
{

class CloudOdometry
{
public:
  // `t_lidar_imu` selects the backend: nullopt → LiDAR-only CT odometry (no
  // insert_imu); a value → LiDAR-IMU CPU odometry with that extrinsic (feed IMU
  // via insert_imu). The convention is GLIM's T_lidar_imu (p_lidar =
  // T_lidar_imu * p_imu).
  explicit CloudOdometry(std::optional<SensorTransform> t_lidar_imu = std::nullopt);
  ~CloudOdometry();

  CloudOdometry(const CloudOdometry &) = delete;
  CloudOdometry & operator=(const CloudOdometry &) = delete;
  CloudOdometry(CloudOdometry &&) noexcept;
  CloudOdometry & operator=(CloudOdometry &&) noexcept;

  // Feed one IMU sample (LiDAR-IMU mode only; a no-op in LiDAR-only mode).
  // Samples must arrive in non-decreasing timestamp order, interleaved with
  // scans so that the IMU covering each scan's interval arrives before it.
  void insert_imu(const ImuSample & imu);

  // Feed one scan. Scans must arrive in non-decreasing timestamp order. A scan
  // with no per-point time is fed with explicit zero per-point times (treated
  // as already motion-undistorted), bypassing GLIM's pseudo-time synthesis.
  void insert(const LidarScan & scan);

  // Flush the remaining in-flight frames and return the full estimated
  // trajectory (LiDAR pose in the world frame), sorted by timestamp.
  [[nodiscard]] std::vector<core::TrajectoryPose> finish();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__CLOUD_ODOMETRY_HPP_
