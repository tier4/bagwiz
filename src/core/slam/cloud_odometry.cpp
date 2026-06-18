// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/cloud_odometry.hpp"

#include "bagwiz/core/slam/glim_estimator.hpp"
#include "bagwiz/core/slam/lidar_scan.hpp"
#include "bagwiz/core/trajectory.hpp"

#include <Eigen/Geometry>
#include <glim/odometry/estimation_frame.hpp>
#include <glim/odometry/odometry_estimation_base.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/util/raw_points.hpp>
#include <glim/util/time_keeper.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <vector>

namespace bagwiz::core::slam
{
namespace
{
core::TrajectoryPose to_pose(const glim::EstimationFrame & frame)
{
  const Eigen::Isometry3d transform = frame.T_world_lidar;
  const Eigen::Vector3d translation = transform.translation();
  Eigen::Quaterniond rotation(transform.rotation());
  rotation.normalize();

  core::TrajectoryPose pose;
  pose.timestamp_ns = static_cast<std::int64_t>(std::llround(frame.stamp * 1e9));
  pose.tx = translation.x();
  pose.ty = translation.y();
  pose.tz = translation.z();
  pose.qx = rotation.x();
  pose.qy = rotation.y();
  pose.qz = rotation.z();
  pose.qw = rotation.w();
  return pose;
}
}  // namespace

struct CloudOdometry::Impl
{
  glim::TimeKeeper time_keeper;
  glim::CloudPreprocessor preprocessor;
  // CT (LiDAR-only) or CPU (LiDAR-IMU) behind the common base interface.
  std::unique_ptr<glim::OdometryEstimationBase> odometry;
  // Keyed by timestamp so every scan contributes exactly one pose and a later
  // finalized (marginalized) pose overwrites the earlier provisional estimate.
  // std::map also keeps the trajectory ordered by time for free.
  std::map<std::int64_t, core::TrajectoryPose> poses;
};

CloudOdometry::CloudOdometry(std::optional<SensorTransform> t_lidar_imu)
{
  // Silence GLIM's one-time construction chatter (it logs ~50 "config not found /
  // using default" lines while reading params; we drive it with no config dir on
  // purpose). RAII-restored so a throwing GLIM constructor cannot leave the
  // shared logger muted for the rest of the process; genuine runtime warnings
  // still surface afterwards.
  const detail::ScopedLoggerSilence silence;
  impl_ = std::make_unique<Impl>();
  impl_->odometry = detail::make_odometry_estimator(t_lidar_imu);
}
CloudOdometry::~CloudOdometry() = default;
CloudOdometry::CloudOdometry(CloudOdometry &&) noexcept = default;
CloudOdometry & CloudOdometry::operator=(CloudOdometry &&) noexcept = default;

void CloudOdometry::insert_imu(const ImuSample & imu)
{
  const double stamp = static_cast<double>(imu.stamp_ns) * 1e-9;
  const Eigen::Vector3d linear_acc(
    imu.linear_acceleration[0], imu.linear_acceleration[1], imu.linear_acceleration[2]);
  const Eigen::Vector3d angular_vel(
    imu.angular_velocity[0], imu.angular_velocity[1], imu.angular_velocity[2]);
  // A no-op on the CT base (LiDAR-only mode); the CPU estimator buffers it.
  impl_->odometry->insert_imu(stamp, linear_acc, angular_vel);
}

void CloudOdometry::insert(const LidarScan & scan)
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

  // A time-less cloud is fed explicit zero per-point times (treated as already
  // motion-undistorted), NOT an empty vector — that would make glim::TimeKeeper
  // synthesize order-based pseudo times and wrongly "deskew" a concatenated
  // cloud.
  if (scan.has_per_point_time && scan.times.size() == num_points) {
    raw->times = scan.times;
  } else {
    raw->times.assign(num_points, 0.0);
  }

  // Resolve the per-point time convention (relative/absolute, unit) and validate
  // ordering. A rewind / invalid frame returns false; skip it.
  if (!impl_->time_keeper.process(raw)) {
    return;
  }

  const auto preprocessed = impl_->preprocessor.preprocess(raw);
  std::vector<glim::EstimationFrame::ConstPtr> marginalized;
  // insert_frame returns this scan's (provisional) estimate; `marginalized`
  // collects the finalized poses of frames leaving the fixed-lag window. Record
  // the returned frame so every scan contributes a pose (including the last
  // smoother-window's worth that never marginalize), then let any finalized
  // pose for the same timestamp overwrite it.
  const glim::EstimationFrame::ConstPtr active =
    impl_->odometry->insert_frame(preprocessed, marginalized);
  if (active) {
    const auto pose = to_pose(*active);
    impl_->poses[pose.timestamp_ns] = pose;
  }
  for (const auto & frame : marginalized) {
    if (frame) {
      const auto pose = to_pose(*frame);
      impl_->poses[pose.timestamp_ns] = pose;
    }
  }
}

std::vector<core::TrajectoryPose> CloudOdometry::finish()
{
  for (const auto & frame : impl_->odometry->get_remaining_frames()) {
    if (frame) {
      const auto pose = to_pose(*frame);
      impl_->poses[pose.timestamp_ns] = pose;
    }
  }
  std::vector<core::TrajectoryPose> trajectory;
  trajectory.reserve(impl_->poses.size());
  for (const auto & entry : impl_->poses) {
    trajectory.push_back(entry.second);
  }
  return trajectory;  // std::map already orders the poses by timestamp
}

}  // namespace bagwiz::core::slam
