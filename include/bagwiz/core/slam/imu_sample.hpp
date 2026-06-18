// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__IMU_SAMPLE_HPP_
#define BAGWIZ__CORE__SLAM__IMU_SAMPLE_HPP_

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string>

// SLAM-facing extraction layer that turns a sensor_msgs/msg/Imu CDR payload into
// a GLIM-free plain-data sample, mirroring how lidar_scan / parse_pointcloud2
// keep the GLIM dependency out of the ingest layer. Only the linear acceleration
// and angular velocity (plus the header stamp / frame_id) are extracted — GLIM's
// LiDAR-IMU odometry preintegrates raw IMU and estimates bias itself, so the
// orientation and the covariance arrays are not needed and are skipped.
//
// Lives in bagwiz_core so it compiles in every build and is unit-tested without
// the GLIM stack; the IMU-coupled odometry that consumes these samples is still
// confined to the BAGWIZ_WITH_SLAM translation units.
namespace bagwiz::core::slam
{

// One extracted IMU sample. Units are whatever the bag carries (ROS convention:
// linear_acceleration in m/s^2 including gravity, angular_velocity in rad/s) —
// fed verbatim to GLIM, which expects raw IMU.
struct ImuSample
{
  std::int64_t stamp_ns = 0;  // header.stamp as nanoseconds since epoch
  std::string frame_id;       // header.frame_id (the IMU's sensor frame)

  std::array<double, 3> linear_acceleration{};  // m/s^2 (x, y, z)
  std::array<double, 3> angular_velocity{};     // rad/s (x, y, z)
};

// Outcome of parse_imu(). On success `sample` holds the data and `error` is
// empty; on failure `sample` is reset and `error` carries the reason.
struct ImuSampleResult
{
  std::optional<ImuSample> sample;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return sample.has_value() && error.empty(); }
};

// Parse a sensor_msgs/msg/Imu CDR payload. Endianness is handled by the CDR
// reader; a malformed / truncated payload yields a non-ok result with `error`
// set rather than throwing.
[[nodiscard]] ImuSampleResult parse_imu(std::span<const std::byte> payload);

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__IMU_SAMPLE_HPP_
