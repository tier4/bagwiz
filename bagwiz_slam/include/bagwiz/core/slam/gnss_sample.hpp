// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__GNSS_SAMPLE_HPP_
#define BAGWIZ__CORE__SLAM__GNSS_SAMPLE_HPP_

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string>

// SLAM-facing extraction layer that turns a sensor_msgs/msg/NavSatFix CDR
// payload into a GLIM-free plain-data sample, mirroring how imu_sample /
// parse_imu keep the GLIM dependency out of the ingest layer. Only the stamp,
// frame_id, fix status, WGS84 latitude/longitude/altitude and the position
// covariance are extracted — the GNSS global constraint (a horizontal translation
// prior on submap poses) needs the position, the status lets the caller drop
// NO_FIX samples, and the covariance lets the mapper weight each prior by the
// fix's reported accuracy instead of a single fixed precision.
//
// Lives in the plain bagwiz_slam library, which builds in every
// configuration, so it is unit-tested without
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

  // sensor_msgs/NavSatFix.position_covariance: 3x3, row-major, in m^2, expressed
  // in the local ENU tangent plane at the antenna {East, North, Up}. Meaningful
  // only when position_covariance_type != COVARIANCE_TYPE_UNKNOWN.
  std::array<double, 9> position_covariance{};

  // sensor_msgs/NavSatFix.position_covariance_type: 0 UNKNOWN, 1 APPROXIMATED,
  // 2 DIAGONAL_KNOWN, 3 KNOWN.
  std::uint8_t position_covariance_type = 0;
};

// sensor_msgs/NavSatStatus.STATUS_NO_FIX: no satellite fix; such samples carry
// no usable position and should be dropped by the caller.
inline constexpr std::int8_t kNavSatStatusNoFix = -1;

// sensor_msgs/NavSatFix.COVARIANCE_TYPE_UNKNOWN: the position_covariance carries
// no usable accuracy information; the caller must not weight a prior by it.
inline constexpr std::uint8_t kNavSatCovarianceTypeUnknown = 0;

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
