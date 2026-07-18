// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/gnss_sample.hpp"

#include "bagwiz/core/cdr_walker/cdr_reader.hpp"

#include <cstdint>
#include <exception>
#include <span>
#include <string>
#include <utility>

namespace bagwiz::core::slam
{

// sensor_msgs/msg/NavSatFix CDR layout (CDR-1):
//
//   std_msgs/Header header
//     builtin_interfaces/Time stamp { int32 sec; uint32 nanosec; }
//     string frame_id
//   sensor_msgs/NavSatStatus status { int8 status; uint16 service; }
//   float64 latitude
//   float64 longitude
//   float64 altitude
//   float64[9] position_covariance        // FIXED-SIZE array, inline, no length prefix
//   uint8 position_covariance_type
//
// We extract the stamp, frame_id, status.status, lat/lon/alt, the 9-element
// position_covariance and its type; status.service is skipped. read_u16 aligns
// past the pad byte after the int8 status, and read_f64 aligns past the
// variable-length frame_id string and the status fields before latitude. The
// covariance is a FIXED-SIZE float64[9] (inline, no length prefix), so it is read
// as nine consecutive read_f64() calls; read_u8 then takes the trailing type byte.
GnssSampleResult parse_navsatfix(std::span<const std::byte> payload)
{
  GnssSampleResult result;
  try {
    cdr_walker::CdrReader reader(payload);

    // Build into a local sample and only publish it once every field has been
    // read — a mid-parse underflow then leaves no partially-filled sample
    // visible to a caller that forgets to check ok().
    GnssSample sample;
    const std::int32_t sec = reader.read_i32();
    const std::uint32_t nanosec = reader.read_u32();
    sample.stamp_ns = static_cast<std::int64_t>(sec) * 1'000'000'000LL + nanosec;
    sample.frame_id = reader.read_string();

    sample.status = reader.read_i8();  // NavSatStatus.status
    reader.read_u16();                 // NavSatStatus.service (skipped)

    sample.latitude = reader.read_f64();
    sample.longitude = reader.read_f64();
    sample.altitude = reader.read_f64();

    for (auto & c : sample.position_covariance) {
      c = reader.read_f64();
    }
    sample.position_covariance_type = reader.read_u8();

    result.sample = std::move(sample);
  } catch (const std::exception & e) {
    result.error = std::string("failed to parse sensor_msgs/msg/NavSatFix payload: ") + e.what();
  }
  return result;
}

}  // namespace bagwiz::core::slam
