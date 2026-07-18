// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/imu_sample.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

// Unit test for parse_imu against a hand-built sensor_msgs/msg/Imu CDR-1 (LE)
// payload. The covariance arrays and orientation are filled with sentinel values
// the parser must SKIP — if the skip offsets are wrong, the extracted
// angular_velocity / linear_acceleration would pick up the sentinels and the
// assertions fail. No GLIM is involved; this runs in the default build.
namespace
{
namespace slam = bagwiz::core::slam;

// Minimal CDR-1 little-endian writer with the same alignment rule the reader
// uses (boundaries relative to the body, i.e. excluding the 4-byte
// encapsulation header).
class CdrWriter
{
public:
  CdrWriter()
  {
    // Encapsulation header: rep_id {0x00, 0x01 = PLAIN_CDR_LE}, options {0,0}.
    bytes_ = {std::byte{0x00}, std::byte{0x01}, std::byte{0x00}, std::byte{0x00}};
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

// Build a full sensor_msgs/msg/Imu payload. orientation + the three covariance
// arrays are written with the `sentinel` value so the parser must skip them.
std::vector<std::byte> make_imu_payload(
  std::int32_t sec, std::uint32_t nanosec, const std::string & frame_id,
  const std::array<double, 3> & angular_velocity, const std::array<double, 3> & linear_acceleration,
  double sentinel = -999.0)
{
  CdrWriter w;
  w.i32(sec);
  w.u32(nanosec);
  w.str(frame_id);
  // orientation (x, y, z, w)
  for (int i = 0; i < 4; ++i) {
    w.f64(sentinel);
  }
  // orientation_covariance[9]
  for (int i = 0; i < 9; ++i) {
    w.f64(sentinel);
  }
  // angular_velocity (x, y, z)
  for (double v : angular_velocity) {
    w.f64(v);
  }
  // angular_velocity_covariance[9]
  for (int i = 0; i < 9; ++i) {
    w.f64(sentinel);
  }
  // linear_acceleration (x, y, z)
  for (double v : linear_acceleration) {
    w.f64(v);
  }
  // linear_acceleration_covariance[9] — present but never read by parse_imu.
  for (int i = 0; i < 9; ++i) {
    w.f64(sentinel);
  }
  return w.bytes();
}

TEST(ParseImu, ExtractsStampFrameAccelAndGyro)
{
  const auto payload = make_imu_payload(
    1'700'000'000, 250'000'000, "imu_link", {0.01, -0.02, 0.03}, {0.5, -0.6, 9.81});

  const auto result = slam::parse_imu(payload);
  ASSERT_TRUE(result.ok()) << result.error;
  const slam::ImuSample & s = *result.sample;

  EXPECT_EQ(s.stamp_ns, 1'700'000'000LL * 1'000'000'000LL + 250'000'000LL);
  EXPECT_EQ(s.frame_id, "imu_link");
  EXPECT_DOUBLE_EQ(s.angular_velocity[0], 0.01);
  EXPECT_DOUBLE_EQ(s.angular_velocity[1], -0.02);
  EXPECT_DOUBLE_EQ(s.angular_velocity[2], 0.03);
  EXPECT_DOUBLE_EQ(s.linear_acceleration[0], 0.5);
  EXPECT_DOUBLE_EQ(s.linear_acceleration[1], -0.6);
  EXPECT_DOUBLE_EQ(s.linear_acceleration[2], 9.81);
}

TEST(ParseImu, HandlesEmptyFrameId)
{
  const auto payload = make_imu_payload(1, 0, "", {1.0, 2.0, 3.0}, {4.0, 5.0, 6.0});

  const auto result = slam::parse_imu(payload);
  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_EQ(result.sample->frame_id, "");
  EXPECT_DOUBLE_EQ(result.sample->linear_acceleration[2], 6.0);
}

TEST(ParseImu, RejectsTruncatedPayload)
{
  // Only the encapsulation header + a partial stamp: parsing must underflow and
  // return a non-ok result rather than throwing.
  const std::vector<std::byte> payload = {
    std::byte{0x00}, std::byte{0x01}, std::byte{0x00}, std::byte{0x00}, std::byte{0x2A}};

  const auto result = slam::parse_imu(payload);
  EXPECT_FALSE(result.ok());
  EXPECT_FALSE(result.error.empty());
}

}  // namespace
