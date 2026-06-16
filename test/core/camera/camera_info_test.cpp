// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/camera/camera_info.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <string>
#include <vector>

namespace
{
using bagwiz::core::camera::CameraInfo;
using bagwiz::core::camera::extract_camera_info;

// Little-endian CDR-1 buffer builder matching the wire layout the production
// CdrReader consumes (see raw_image_test.cpp for the alignment rationale).
class CdrBuilder
{
public:
  CdrBuilder()
  {
    for (int b : {0x00, 0x01, 0x00, 0x00}) {
      buf_.push_back(static_cast<std::byte>(b));
    }
  }

  void u8(std::uint8_t v) { buf_.push_back(static_cast<std::byte>(v)); }
  void bool_(bool v) { u8(v ? 1U : 0U); }

  void u32(std::uint32_t v)
  {
    align(4);
    for (int i = 0; i < 4; ++i) {
      buf_.push_back(static_cast<std::byte>((v >> (8 * i)) & 0xFFU));
    }
  }

  void i32(std::int32_t v) { u32(static_cast<std::uint32_t>(v)); }

  void str(const std::string & s)
  {
    u32(static_cast<std::uint32_t>(s.size() + 1));
    for (char c : s) {
      buf_.push_back(static_cast<std::byte>(static_cast<unsigned char>(c)));
    }
    buf_.push_back(std::byte{0});
  }

  void f64(double v)
  {
    align(8);
    std::array<std::byte, 8> bytes{};
    std::memcpy(bytes.data(), &v, sizeof(v));
    if constexpr (std::endian::native == std::endian::big) {
      std::reverse(bytes.begin(), bytes.end());
    }
    for (auto b : bytes) {
      buf_.push_back(b);
    }
  }

  void fixed_f64_array(std::span<const double> values)
  {
    for (double v : values) {
      f64(v);
    }
  }

  void f64_seq(std::span<const double> values)
  {
    u32(static_cast<std::uint32_t>(values.size()));
    for (double v : values) {
      f64(v);
    }
  }

  [[nodiscard]] std::vector<std::byte> take() const { return buf_; }
  [[nodiscard]] std::span<const std::byte> view() const { return {buf_.data(), buf_.size()}; }

private:
  void align(std::size_t n)
  {
    while ((buf_.size() - 4) % n != 0) {
      buf_.push_back(std::byte{0});
    }
  }

  std::vector<std::byte> buf_;
};

std::vector<std::byte> make_camera_info_payload(
  std::uint32_t width, std::uint32_t height,
  const std::string & distortion_model,
  const std::array<double, 9> & K,
  const std::vector<double> & D,
  const std::array<double, 9> & R,
  const std::array<double, 12> & P,
  const std::string & frame_id)
{
  CdrBuilder b;
  b.i32(0);                       // header.stamp.sec
  b.u32(0);                       // header.stamp.nanosec
  b.str(frame_id);                // header.frame_id
  b.u32(height);                  // height
  b.u32(width);                   // width
  b.str(distortion_model);        // distortion_model
  b.fixed_f64_array({K.data(), K.size()});
  b.f64_seq({D.data(), D.size()});
  b.fixed_f64_array({R.data(), R.size()});
  b.fixed_f64_array({P.data(), P.size()});
  b.u32(0);                       // binning_x
  b.u32(0);                       // binning_y
  b.u32(0);                       // roi.x_offset
  b.u32(0);                       // roi.y_offset
  b.u32(0);                       // roi.height
  b.u32(0);                       // roi.width
  b.bool_(false);                 // roi.do_rectify
  return b.take();
}

TEST(CameraInfoTest, ExtractsKAndDimensions)
{
  const auto K = std::array<double, 9>{500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0};
  const auto R = std::array<double, 9>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  const auto P = std::array<double, 12>{
    500.0, 0.0, 320.0, 0.0, 0.0, 500.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  const auto payload = make_camera_info_payload(
    640, 480, "plumb_bob", K, std::vector<double>{0.1, -0.05, 0.0, 0.0, 0.0}, R, P, "/camera");

  const auto result = extract_camera_info(payload);
  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_EQ(result.image->width, 640U);
  EXPECT_EQ(result.image->height, 480U);
  EXPECT_DOUBLE_EQ(result.image->K[0], 500.0);
  EXPECT_DOUBLE_EQ(result.image->K[2], 320.0);
  EXPECT_DOUBLE_EQ(result.image->K[4], 500.0);
  EXPECT_DOUBLE_EQ(result.image->K[5], 240.0);
}

TEST(CameraInfoTest, ExtractsDistortionModelFrameIdAndDP)
{
  const auto K = std::array<double, 9>{600.0, 0.0, 300.0, 0.0, 600.0, 200.0, 0.0, 0.0, 1.0};
  const auto D = std::vector<double>{0.0, 0.0, 0.0};
  const auto R = std::array<double, 9>{
    0.9999, 0.0012, -0.0034,
    -0.0011, 1.0000, 0.0056,
    0.0035, -0.0055, 0.9998};
  const auto P = std::array<double, 12>{
    600.0, 0.0, 300.0, 0.0, 0.0, 600.0, 200.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  const auto payload = make_camera_info_payload(
    800, 600, "rational_polynomial", K, D, R, P, "camera_optical_link");

  const auto result = extract_camera_info(payload);
  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_EQ(result.image->width, 800U);
  EXPECT_EQ(result.image->height, 600U);
  EXPECT_EQ(result.image->frame_id, "camera_optical_link");
  EXPECT_EQ(result.image->distortion_model, "rational_polynomial");
  EXPECT_EQ(result.image->D.size(), 3U);
  EXPECT_DOUBLE_EQ(result.image->D[0], 0.0);
  for (std::size_t i = 0; i < R.size(); ++i) {
    EXPECT_DOUBLE_EQ(result.image->R[i], R[i]) << "R[" << i << "]";
  }
  for (std::size_t i = 0; i < P.size(); ++i) {
    EXPECT_DOUBLE_EQ(result.image->P[i], P[i]) << "P[" << i << "]";
  }
}

TEST(CameraInfoTest, RejectsZeroFx)
{
  const auto K = std::array<double, 9>{0.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0};
  const auto R = std::array<double, 9>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  const auto P = std::array<double, 12>{};
  const auto payload = make_camera_info_payload(
    640, 480, "plumb_bob", K, std::vector<double>{}, R, P, "/camera");

  const auto result = extract_camera_info(payload);
  EXPECT_FALSE(result.ok());
  EXPECT_FALSE(result.error.empty());
}

TEST(CameraInfoTest, RejectsZeroFy)
{
  const auto K = std::array<double, 9>{500.0, 0.0, 320.0, 0.0, 0.0, 240.0, 0.0, 0.0, 1.0};
  const auto R = std::array<double, 9>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  const auto P = std::array<double, 12>{};
  const auto payload = make_camera_info_payload(
    640, 480, "plumb_bob", K, std::vector<double>{}, R, P, "/camera");

  const auto result = extract_camera_info(payload);
  EXPECT_FALSE(result.ok());
  EXPECT_FALSE(result.error.empty());
}

TEST(CameraInfoTest, TruncatedPayloadYieldsError)
{
  CdrBuilder b;
  b.i32(0);
  b.u32(0);
  b.str("cam");
  b.u32(480);
  b.u32(640);
  b.str("plumb_bob");
  // Stop before K: payload is too short for the fixed K array.
  const auto result = extract_camera_info(b.view());
  EXPECT_FALSE(result.ok());
  EXPECT_FALSE(result.error.empty());
}

}  // namespace
