// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/image/compressed_image.hpp"

#include "bagwiz/core/cdr_walker/cdr_reader.hpp"

#include <cstdint>
#include <exception>
#include <span>
#include <string>
#include <utility>

namespace bagwiz::core::image
{

// sensor_msgs/msg/CompressedImage CDR layout (CDR-1, as written by Fast/Cyclone
// DDS):
//
//   std_msgs/Header header
//     builtin_interfaces/Time stamp { int32 sec; uint32 nanosec; }
//     string frame_id
//   string format            // e.g. "jpeg", "png", "rgb8; jpeg compressed bgr8"
//   uint8[] data             // length-prefixed; the compressed bitstream
//
// The compressed `data` is read as a zero-copy span via CdrReader::read_bytes,
// so a multi-megabyte frame is never materialised element-by-element.
CompressedImageResult extract_compressed_image(std::span<const std::byte> payload)
{
  CompressedImageResult result;
  try {
    cdr_walker::CdrReader reader(payload);

    (void)reader.read_i32();     // header.stamp.sec
    (void)reader.read_u32();     // header.stamp.nanosec
    (void)reader.read_string();  // header.frame_id

    std::string format = reader.read_string();

    const std::uint32_t data_len = reader.read_sequence_length();
    const std::span<const std::byte> data = reader.read_bytes(data_len);

    CompressedImageView view;
    view.format = std::move(format);
    view.data = data;
    result.image = std::move(view);
  } catch (const std::exception & e) {
    result.image.reset();
    result.error =
      std::string("failed to parse sensor_msgs/msg/CompressedImage payload: ") + e.what();
  }
  return result;
}

}  // namespace bagwiz::core::image
