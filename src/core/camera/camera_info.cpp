// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/camera/camera_info.hpp"

#include "bagwiz/core/cdr_walker/cdr_reader.hpp"

#include <cstdint>
#include <exception>
#include <span>
#include <string>
#include <utility>

namespace bagwiz::core::camera
{

// sensor_msgs/msg/CameraInfo CDR layout (CDR-1, as written by Fast/Cyclone DDS):
//
//   std_msgs/Header header
//     builtin_interfaces/Time stamp { int32 sec; uint32 nanosec; }
//     string frame_id
//   uint32 height
//   uint32 width
//   string distortion_model
//   float64[9] K
//   float64[] D
//   float64[12] P
//   uint32 binning_x
//   uint32 binning_y
//   sensor_msgs/RegionOfInterest roi
//     uint32 x_offset
//     uint32 y_offset
//     uint32 height
//     uint32 width
//     bool   do_rectify
//
// Only the fields up to P are required for the overlay pipeline; the rest are
// ignored.
CameraInfoResult extract_camera_info(std::span<const std::byte> payload)
{
  CameraInfoResult result;
  try {
    cdr_walker::CdrReader reader(payload);

    (void)reader.read_i32();     // header.stamp.sec
    (void)reader.read_u32();     // header.stamp.nanosec
    result.info.frame_id = reader.read_string();

    result.info.height = reader.read_u32();
    result.info.width = reader.read_u32();
    result.info.distortion_model = reader.read_string();

    for (std::size_t i = 0; i < result.info.K.size(); ++i) {
      result.info.K[i] = reader.read_f64();
    }

    const std::uint32_t d_len = reader.read_sequence_length();
    result.info.D.resize(d_len);
    for (std::uint32_t i = 0; i < d_len; ++i) {
      result.info.D[i] = reader.read_f64();
    }

    for (std::size_t i = 0; i < result.info.P.size(); ++i) {
      result.info.P[i] = reader.read_f64();
    }

    // binning_x, binning_y and roi are not needed.
  } catch (const std::exception & e) {
    result.ok = false;
    result.error = std::string("failed to parse sensor_msgs/msg/CameraInfo payload: ") + e.what();
    result.info = CameraInfo{};
    return result;
  }

  if (result.info.K[0] <= 0.0 || result.info.K[4] <= 0.0) {
    result.ok = false;
    result.error = "invalid CameraInfo focal lengths: K[0] and K[4] must be positive";
    result.info = CameraInfo{};
    return result;
  }

  result.ok = true;
  return result;
}

}  // namespace bagwiz::core::camera
