// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/imu_sample.hpp"

#include "bagwiz/core/cdr_walker/cdr_reader.hpp"

#include <cstdint>
#include <exception>
#include <span>
#include <string>
#include <utility>

namespace bagwiz::core::slam
{
namespace
{
// sensor_msgs/msg/Imu CDR layout (CDR-1):
//
//   std_msgs/Header header
//     builtin_interfaces/Time stamp { int32 sec; uint32 nanosec; }
//     string frame_id
//   geometry_msgs/Quaternion orientation { float64 x, y, z, w; }
//   float64[9] orientation_covariance
//   geometry_msgs/Vector3 angular_velocity { float64 x, y, z; }
//   float64[9] angular_velocity_covariance
//   geometry_msgs/Vector3 linear_acceleration { float64 x, y, z; }
//   float64[9] linear_acceleration_covariance
//
// The covariance fields are FIXED-SIZE arrays (float64[9]) — serialized inline
// with no length prefix. We extract the stamp, frame_id, angular_velocity and
// linear_acceleration; orientation and all covariances are skipped (GLIM
// preintegrates raw IMU and estimates bias itself). read_f64 handles the 8-byte
// alignment after the variable-length frame_id string.
void skip_f64(cdr_walker::CdrReader & reader, int count)
{
  for (int i = 0; i < count; ++i) {
    reader.read_f64();
  }
}
}  // namespace

ImuSampleResult parse_imu(std::span<const std::byte> payload)
{
  ImuSampleResult result;
  try {
    cdr_walker::CdrReader reader(payload);

    // Build into a local sample and only publish it once every field has been
    // read — a mid-parse underflow then leaves no partially-filled sample
    // visible to a caller that forgets to check ok().
    ImuSample sample;
    const std::int32_t sec = reader.read_i32();
    const std::uint32_t nanosec = reader.read_u32();
    sample.stamp_ns = static_cast<std::int64_t>(sec) * 1'000'000'000LL + nanosec;
    sample.frame_id = reader.read_string();

    skip_f64(reader, 4);  // orientation (x, y, z, w)
    skip_f64(reader, 9);  // orientation_covariance

    sample.angular_velocity[0] = reader.read_f64();
    sample.angular_velocity[1] = reader.read_f64();
    sample.angular_velocity[2] = reader.read_f64();

    skip_f64(reader, 9);  // angular_velocity_covariance

    sample.linear_acceleration[0] = reader.read_f64();
    sample.linear_acceleration[1] = reader.read_f64();
    sample.linear_acceleration[2] = reader.read_f64();
    // linear_acceleration_covariance is not read.

    result.sample = std::move(sample);
  } catch (const std::exception & e) {
    result.error = std::string("failed to parse sensor_msgs/msg/Imu payload: ") + e.what();
  }
  return result;
}

}  // namespace bagwiz::core::slam
