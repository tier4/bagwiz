// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__GNSS_SAMPLE_HPP_
#define BAGWIZ__CORE__SLAM__GNSS_SAMPLE_HPP_

#include <cstdint>
#include <optional>
#include <span>
#include <string>

// SLAM-facing extraction layer that turns a sensor_msgs/msg/NavSatFix CDR
// payload into a GLIM-free plain-data sample, mirroring how imu_sample /
// parse_imu keep the GLIM dependency out of the ingest layer. Only the stamp,
// frame_id, fix status and WGS84 latitude/longitude/altitude are extracted —
// the GNSS global constraint (a horizontal translation prior on submap poses)
// needs the position, and the status lets the caller drop NO_FIX samples; the
// position covariance is skipped (the ported glim_ext constraint ignores it).
//
// Lives in bagwiz_core so it compiles in every build and is unit-tested without
// the GLIM stack; the projection to a local metric frame (gnss_projector) and
// the GLIM-coupled mapping that consumes the projected points stay separate.
namespace bagwiz::core::slam
{

// One extracted GNSS fix. Geographic coordinates are WGS84 as carried by
// NavSatFix (latitude/longitude in degrees, altitude in meters above the
// ellipsoid); they are projected to a local metric frame downstream.
struct GnssSample
{
  std::int64_t stamp_ns = 0;  // header.stamp as nanoseconds since epoch
  std::string frame_id;       // header.frame_id (the GNSS antenna's frame)

  // sensor_msgs/NavSatStatus.status: -1 NO_FIX, 0 FIX, 1 SBAS_FIX, 2 GBAS_FIX.
  std::int8_t status = 0;

  double latitude = 0.0;   // degrees
  double longitude = 0.0;  // degrees
  double altitude = 0.0;   // meters (WGS84 ellipsoid)
};

// sensor_msgs/NavSatStatus.STATUS_NO_FIX: no satellite fix; such samples carry
// no usable position and should be dropped by the caller.
inline constexpr std::int8_t kNavSatStatusNoFix = -1;

// Outcome of parse_navsatfix(). On success `sample` holds the data and `error`
// is empty; on failure `sample` is reset and `error` carries the reason.
struct GnssSampleResult
{
  std::optional<GnssSample> sample;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return sample.has_value() && error.empty(); }
};

// Parse a sensor_msgs/msg/NavSatFix CDR payload. Endianness is handled by the
// CDR reader; a malformed / truncated payload yields a non-ok result with
// `error` set rather than throwing.
[[nodiscard]] GnssSampleResult parse_navsatfix(std::span<const std::byte> payload);

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__GNSS_SAMPLE_HPP_
