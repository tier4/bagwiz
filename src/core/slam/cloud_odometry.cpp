// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/cloud_odometry.hpp"

#include "bagwiz/core/slam/lidar_scan.hpp"
#include "bagwiz/core/trajectory.hpp"

#include <Eigen/Geometry>
#include <glim/odometry/estimation_frame.hpp>
#include <glim/odometry/odometry_estimation_ct.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/util/raw_points.hpp>
#include <glim/util/time_keeper.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
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
  glim::OdometryEstimationCT odometry;
  std::vector<core::TrajectoryPose> poses;
};

CloudOdometry::CloudOdometry() : impl_(std::make_unique<Impl>())
{
}
CloudOdometry::~CloudOdometry() = default;
CloudOdometry::CloudOdometry(CloudOdometry &&) noexcept = default;
CloudOdometry & CloudOdometry::operator=(CloudOdometry &&) noexcept = default;

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
  impl_->odometry.insert_frame(preprocessed, marginalized);
  for (const auto & frame : marginalized) {
    if (frame) {
      impl_->poses.push_back(to_pose(*frame));
    }
  }
}

std::vector<core::TrajectoryPose> CloudOdometry::finish()
{
  for (const auto & frame : impl_->odometry.get_remaining_frames()) {
    if (frame) {
      impl_->poses.push_back(to_pose(*frame));
    }
  }
  std::sort(
    impl_->poses.begin(), impl_->poses.end(),
    [](const core::TrajectoryPose & lhs, const core::TrajectoryPose & rhs) {
      return lhs.timestamp_ns < rhs.timestamp_ns;
    });
  return impl_->poses;
}

}  // namespace bagwiz::core::slam
