// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/image/camera_info.hpp"

#include "bagwiz/core/cdr_walker/cdr_reader.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace bagwiz::core::image
{

// sensor_msgs/msg/CameraInfo CDR layout (CDR-1, as written by Fast/Cyclone DDS):
//
//   std_msgs/Header header
//     builtin_interfaces/Time stamp { int32 sec; uint32 nanosec; }
//     string frame_id
//   uint32 height
//   uint32 width
//   string distortion_model
//   float64[] d               // length-prefixed sequence
//   float64[9] k              // fixed array
//   float64[9] r              // fixed array
//   float64[12] p             // fixed array
//   uint32 binning_x
//   uint32 binning_y
//   sensor_msgs/RegionOfInterest roi
//     uint32 x_offset
//     uint32 y_offset
//     uint32 width
//     uint32 height
//     bool do_rectify
//
// Only header, height, width, distortion_model, d, k, r, and p are needed for
// undistortion; the rest are read and discarded so the parser stays positioned
// correctly.
CameraInfoResult extract_camera_info(std::span<const std::byte> payload)
{
  CameraInfoResult result;
  try {
    cdr_walker::CdrReader reader(payload);

    // std_msgs/Header
    (void)reader.read_i32();     // header.stamp.sec
    (void)reader.read_u32();     // header.stamp.nanosec
    (void)reader.read_string();  // header.frame_id

    CameraInfo info;
    info.height = reader.read_u32();
    info.width = reader.read_u32();
    info.distortion_model = reader.read_string();

    // float64[] d
    {
      const std::uint32_t d_len = reader.read_sequence_length();
      info.d.resize(d_len);
      for (std::uint32_t i = 0; i < d_len; ++i) {
        info.d[i] = reader.read_f64();
      }
    }

    // float64[9] k
    for (std::size_t i = 0; i < 9; ++i) {
      info.k[i] = reader.read_f64();
    }

    // float64[9] r
    for (std::size_t i = 0; i < 9; ++i) {
      info.r[i] = reader.read_f64();
    }

    // float64[12] p
    for (std::size_t i = 0; i < 12; ++i) {
      info.p[i] = reader.read_f64();
    }

    // binning_x, binning_y, roi — not needed, but consume them so the reader
    // reaches the end of a well-formed message.
    (void)reader.read_u32();   // binning_x
    (void)reader.read_u32();   // binning_y
    (void)reader.read_u32();   // roi.x_offset
    (void)reader.read_u32();   // roi.y_offset
    (void)reader.read_u32();   // roi.width
    (void)reader.read_u32();   // roi.height
    (void)reader.read_bool();  // roi.do_rectify

    result.info = std::move(info);
  } catch (const std::exception & e) {
    result.info.reset();
    result.error = std::string("failed to parse sensor_msgs/msg/CameraInfo payload: ") + e.what();
  }
  return result;
}

}  // namespace bagwiz::core::image
