// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/image/raw_image.hpp"

#include "bagwiz/core/cdr_walker/cdr_reader.hpp"

#include <cstdint>
#include <exception>
#include <span>
#include <string>
#include <utility>

namespace bagwiz::core::image
{

// sensor_msgs/msg/Image CDR layout (CDR-1, as written by Fast/Cyclone DDS):
//
//   std_msgs/Header header
//     builtin_interfaces/Time stamp { int32 sec; uint32 nanosec; }
//     string frame_id
//   uint32 height
//   uint32 width
//   string encoding
//   uint8  is_bigendian
//   uint32 step
//   uint8[] data            // length-prefixed; step * height bytes
//
// The pixel `data` is read as a zero-copy span via CdrReader::read_bytes, so a
// multi-megabyte frame is never materialised element-by-element.
RawImageResult extract_raw_image(std::span<const std::byte> payload)
{
  RawImageResult result;
  try {
    cdr_walker::CdrReader reader(payload);

    const std::int32_t stamp_sec = reader.read_i32();       // header.stamp.sec
    const std::uint32_t stamp_nanosec = reader.read_u32();  // header.stamp.nanosec
    (void)reader.read_string();                             // header.frame_id

    const std::uint32_t height = reader.read_u32();
    const std::uint32_t width = reader.read_u32();
    std::string encoding = reader.read_string();
    (void)reader.read_u8();  // is_bigendian
    const std::uint32_t step = reader.read_u32();

    const std::uint32_t data_len = reader.read_sequence_length();
    const std::span<const std::byte> data = reader.read_bytes(data_len);

    RawImageView view;
    view.width = width;
    view.height = height;
    view.step = step;
    view.encoding = std::move(encoding);
    view.data = data;
    view.header_stamp_ns = static_cast<std::int64_t>(stamp_sec) * 1'000'000'000LL + stamp_nanosec;
    result.image = std::move(view);
  } catch (const std::exception & e) {
    result.image.reset();
    result.error = std::string("failed to parse sensor_msgs/msg/Image payload: ") + e.what();
  }
  return result;
}

}  // namespace bagwiz::core::image
