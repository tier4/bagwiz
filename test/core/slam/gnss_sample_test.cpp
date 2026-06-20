// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/gnss_sample.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

// Unit test for parse_navsatfix against a hand-built sensor_msgs/msg/NavSatFix
// CDR-1 (LE) payload. Exercises stamp/frame/status/lat-lon-alt extraction plus the
// position_covariance[9] + type block that follows them — if the layout/alignment
// is wrong, lat/lon/alt or the covariance values shift and the assertions fail. No
// GLIM is involved; this runs in the default build.
namespace
{
namespace slam = bagwiz::core::slam;

// Minimal CDR-1 little-endian writer with the same alignment rule the reader
// uses (boundaries relative to the body, i.e. excluding the 4-byte
// encapsulation header). Mirrors imu_sample_test's CdrWriter.
class CdrWriter
{
public:
  CdrWriter()
  {
    // Encapsulation header: rep_id {0x00, 0x01 = PLAIN_CDR_LE}, options {0,0}.
    bytes_ = {std::byte{0x00}, std::byte{0x01}, std::byte{0x00}, std::byte{0x00}};
  }

  void i8(std::int8_t v) { bytes_.push_back(static_cast<std::byte>(v)); }

  void u8(std::uint8_t v) { bytes_.push_back(static_cast<std::byte>(v)); }

  void u16(std::uint16_t v)
  {
    align(2);
    bytes_.push_back(static_cast<std::byte>(v & 0xFF));
    bytes_.push_back(static_cast<std::byte>((v >> 8) & 0xFF));
  }

  void i32(std::int32_t v) { u32(static_cast<std::uint32_t>(v)); }

  void u32(std::uint32_t v)
  {
    align(4);
    for (int i = 0; i < 4; ++i) {
      bytes_.push_back(static_cast<std::byte>((v >> (8 * i)) & 0xFF));
    }
  }

  void f64(double v)
  {
    align(8);
    std::uint64_t bits = 0;
    std::memcpy(&bits, &v, sizeof(bits));
    for (int i = 0; i < 8; ++i) {
      bytes_.push_back(static_cast<std::byte>((bits >> (8 * i)) & 0xFF));
    }
  }

  void str(const std::string & s)
  {
    u32(static_cast<std::uint32_t>(s.size() + 1));  // length includes trailing NUL
    for (char c : s) {
      bytes_.push_back(static_cast<std::byte>(c));
    }
    bytes_.push_back(std::byte{0});
  }

  [[nodiscard]] const std::vector<std::byte> & bytes() const { return bytes_; }

private:
  void align(std::size_t n)
  {
    // Body offset excludes the 4-byte encapsulation header.
    while ((bytes_.size() - 4) % n != 0) {
      bytes_.push_back(std::byte{0});
    }
  }

  std::vector<std::byte> bytes_;
};

// Build a full sensor_msgs/msg/NavSatFix payload. position_covariance is written
// with the `sentinel` value so the parser must skip it; position_covariance_type
// is present but never read.
std::vector<std::byte> make_navsatfix_payload(
  std::int32_t sec, std::uint32_t nanosec, const std::string & frame_id, std::int8_t status,
  std::uint16_t service, double latitude, double longitude, double altitude,
  const std::array<double, 9> & covariance = {}, std::uint8_t covariance_type = 0)
{
  CdrWriter w;
  w.i32(sec);
  w.u32(nanosec);
  w.str(frame_id);
  // sensor_msgs/NavSatStatus { int8 status; uint16 service }
  w.i8(status);
  w.u16(service);
  w.f64(latitude);
  w.f64(longitude);
  w.f64(altitude);
  // position_covariance[9] — fixed-size float64 array (inline, no length prefix).
  for (double c : covariance) {
    w.f64(c);
  }
  w.u8(covariance_type);  // position_covariance_type
  return w.bytes();
}

TEST(ParseNavSatFix, ExtractsStampFrameStatusAndLatLonAlt)
{
  const auto payload = make_navsatfix_payload(
    1'700'000'000, 250'000'000, "gnss_link", /*status=*/0, /*service=*/1, 35.6586, 139.7454, 40.5);

  const auto result = slam::parse_navsatfix(payload);
  ASSERT_TRUE(result.ok()) << result.error;
  const slam::GnssSample & s = *result.sample;

  EXPECT_EQ(s.stamp_ns, 1'700'000'000LL * 1'000'000'000LL + 250'000'000LL);
  EXPECT_EQ(s.frame_id, "gnss_link");
  EXPECT_EQ(s.status, 0);
  EXPECT_DOUBLE_EQ(s.latitude, 35.6586);
  EXPECT_DOUBLE_EQ(s.longitude, 139.7454);
  EXPECT_DOUBLE_EQ(s.altitude, 40.5);
}

TEST(ParseNavSatFix, ExtractsPositionCovarianceAndType)
{
  // A full row-major 3x3 covariance (ENU), with off-diagonal terms, plus a KNOWN
  // type. The parser must read all nine values in order and the trailing type
  // byte; getting the float64[9] layout wrong would shift these or the type.
  const std::array<double, 9> cov = {0.58, -0.57, 0.38, -0.57, 1.43, -0.29, 0.38, -0.29, 3.06};
  const auto payload = make_navsatfix_payload(
    1'700'000'000, 0, "gnss_link", /*status=*/1, /*service=*/15, 35.0, 139.0, 40.0, cov,
    /*covariance_type=*/3);

  const auto result = slam::parse_navsatfix(payload);
  ASSERT_TRUE(result.ok()) << result.error;
  const slam::GnssSample & s = *result.sample;

  for (std::size_t i = 0; i < cov.size(); ++i) {
    EXPECT_DOUBLE_EQ(s.position_covariance[i], cov[i]) << "covariance[" << i << "]";
  }
  EXPECT_EQ(s.position_covariance_type, 3);
  // lat/lon/alt must remain uncorrupted by the covariance block that follows them.
  EXPECT_DOUBLE_EQ(s.latitude, 35.0);
  EXPECT_DOUBLE_EQ(s.altitude, 40.0);
}

TEST(ParseNavSatFix, PreservesNoFixStatus)
{
  // status = -1 (STATUS_NO_FIX). The parser must report it faithfully so callers
  // can drop the sample; it does not filter on its own.
  const auto payload =
    make_navsatfix_payload(1, 0, "", /*status=*/-1, /*service=*/0, 0.0, 0.0, 0.0);

  const auto result = slam::parse_navsatfix(payload);
  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_EQ(result.sample->status, -1);
  EXPECT_EQ(result.sample->frame_id, "");
}

TEST(ParseNavSatFix, RejectsTruncatedPayload)
{
  // Only the encapsulation header + a partial stamp: parsing must underflow and
  // return a non-ok result rather than throwing.
  const std::vector<std::byte> payload = {
    std::byte{0x00}, std::byte{0x01}, std::byte{0x00}, std::byte{0x00}, std::byte{0x2A}};

  const auto result = slam::parse_navsatfix(payload);
  EXPECT_FALSE(result.ok());
  EXPECT_FALSE(result.error.empty());
}

}  // namespace
