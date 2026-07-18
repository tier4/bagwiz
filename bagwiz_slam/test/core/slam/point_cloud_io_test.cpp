// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/point_cloud_io.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace
{
namespace slam = bagwiz::core::slam;

// Split a PCD stream into its ASCII header (through "DATA binary\n") and the
// binary body that follows.
std::pair<std::string, std::string> split_pcd(const std::string & blob)
{
  const std::string marker = "DATA binary\n";
  const auto pos = blob.find(marker);
  if (pos == std::string::npos) {
    return {blob, ""};
  }
  const auto body_start = pos + marker.size();
  return {blob.substr(0, body_start), blob.substr(body_start)};
}

float read_float(const std::string & body, std::size_t index)
{
  float value = 0.0F;
  std::memcpy(&value, body.data() + index * sizeof(float), sizeof(float));
  return value;
}

TEST(PointCloudIo, WritesXyzPcdHeaderAndBody)
{
  const std::vector<std::array<float, 3>> points = {{1.0F, 2.0F, 3.0F}, {4.0F, 5.0F, 6.0F}};
  std::ostringstream os;
  slam::write_pcd(os, points);
  const auto [header, body] = split_pcd(os.str());

  EXPECT_NE(header.find("VERSION 0.7\n"), std::string::npos);
  EXPECT_NE(header.find("FIELDS x y z\n"), std::string::npos);
  EXPECT_NE(header.find("SIZE 4 4 4\n"), std::string::npos);
  EXPECT_NE(header.find("TYPE F F F\n"), std::string::npos);
  EXPECT_NE(header.find("WIDTH 2\n"), std::string::npos);
  EXPECT_NE(header.find("HEIGHT 1\n"), std::string::npos);
  EXPECT_NE(header.find("POINTS 2\n"), std::string::npos);
  EXPECT_EQ(header.find("intensity"), std::string::npos);

  ASSERT_EQ(body.size(), 2U * 3U * sizeof(float));
  EXPECT_FLOAT_EQ(read_float(body, 0), 1.0F);
  EXPECT_FLOAT_EQ(read_float(body, 2), 3.0F);
  EXPECT_FLOAT_EQ(read_float(body, 3), 4.0F);
  EXPECT_FLOAT_EQ(read_float(body, 5), 6.0F);
}

TEST(PointCloudIo, IncludesIntensityWhenSizesMatch)
{
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 0.0F}, {1.0F, 1.0F, 1.0F}};
  const std::vector<float> intensities = {10.0F, 20.0F};
  std::ostringstream os;
  slam::write_pcd(os, points, intensities);
  const auto [header, body] = split_pcd(os.str());

  EXPECT_NE(header.find("FIELDS x y z intensity\n"), std::string::npos);
  EXPECT_NE(header.find("SIZE 4 4 4 4\n"), std::string::npos);
  EXPECT_NE(header.find("TYPE F F F F\n"), std::string::npos);
  ASSERT_EQ(body.size(), 2U * 4U * sizeof(float));
  EXPECT_FLOAT_EQ(read_float(body, 3), 10.0F);  // first point's intensity
  EXPECT_FLOAT_EQ(read_float(body, 7), 20.0F);  // second point's intensity
}

TEST(PointCloudIo, OmitsIntensityOnSizeMismatch)
{
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 0.0F}, {1.0F, 1.0F, 1.0F}};
  const std::vector<float> intensities = {10.0F};  // wrong length
  std::ostringstream os;
  slam::write_pcd(os, points, intensities);
  const auto [header, body] = split_pcd(os.str());

  EXPECT_EQ(header.find("intensity"), std::string::npos);
  EXPECT_EQ(body.size(), 2U * 3U * sizeof(float));
}

TEST(PointCloudIo, EmptyCloudWritesZeroPoints)
{
  std::ostringstream os;
  slam::write_pcd(os, {});
  const auto [header, body] = split_pcd(os.str());
  EXPECT_NE(header.find("WIDTH 0\n"), std::string::npos);
  EXPECT_NE(header.find("POINTS 0\n"), std::string::npos);
  EXPECT_TRUE(body.empty());
}

// Unpack the PCL packed-float rgb convention: the float's bits are the uint32
// 0x00RRGGBB.
std::array<std::uint8_t, 3> unpack_rgb(float value)
{
  std::uint32_t packed = 0;
  std::memcpy(&packed, &value, sizeof(packed));
  return {
    static_cast<std::uint8_t>((packed >> 16) & 0xFFU),
    static_cast<std::uint8_t>((packed >> 8) & 0xFFU), static_cast<std::uint8_t>(packed & 0xFFU)};
}

TEST(PointCloudIo, IncludesRgbWhenSizesMatch)
{
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 0.0F}, {1.0F, 1.0F, 1.0F}};
  const std::vector<std::array<std::uint8_t, 3>> colors = {{255, 0, 128}, {1, 2, 3}};
  std::ostringstream os;
  slam::write_pcd(os, points, {}, colors);
  const auto [header, body] = split_pcd(os.str());

  EXPECT_NE(header.find("FIELDS x y z rgb\n"), std::string::npos);
  EXPECT_NE(header.find("SIZE 4 4 4 4\n"), std::string::npos);
  EXPECT_NE(header.find("TYPE F F F F\n"), std::string::npos);
  EXPECT_NE(header.find("COUNT 1 1 1 1\n"), std::string::npos);
  EXPECT_EQ(header.find("intensity"), std::string::npos);

  ASSERT_EQ(body.size(), 2U * 4U * sizeof(float));
  EXPECT_EQ(unpack_rgb(read_float(body, 3)), (std::array<std::uint8_t, 3>{255, 0, 128}));
  EXPECT_EQ(unpack_rgb(read_float(body, 7)), (std::array<std::uint8_t, 3>{1, 2, 3}));
}

TEST(PointCloudIo, IncludesIntensityAndRgbTogether)
{
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 0.0F}, {1.0F, 1.0F, 1.0F}};
  const std::vector<float> intensities = {10.0F, 20.0F};
  const std::vector<std::array<std::uint8_t, 3>> colors = {{9, 8, 7}, {6, 5, 4}};
  std::ostringstream os;
  slam::write_pcd(os, points, intensities, colors);
  const auto [header, body] = split_pcd(os.str());

  EXPECT_NE(header.find("FIELDS x y z intensity rgb\n"), std::string::npos);
  EXPECT_NE(header.find("SIZE 4 4 4 4 4\n"), std::string::npos);
  EXPECT_NE(header.find("TYPE F F F F F\n"), std::string::npos);

  ASSERT_EQ(body.size(), 2U * 5U * sizeof(float));
  EXPECT_FLOAT_EQ(read_float(body, 3), 10.0F);
  EXPECT_EQ(unpack_rgb(read_float(body, 4)), (std::array<std::uint8_t, 3>{9, 8, 7}));
  EXPECT_FLOAT_EQ(read_float(body, 8), 20.0F);
  EXPECT_EQ(unpack_rgb(read_float(body, 9)), (std::array<std::uint8_t, 3>{6, 5, 4}));
}

TEST(PointCloudIo, OmitsRgbOnSizeMismatch)
{
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 0.0F}, {1.0F, 1.0F, 1.0F}};
  const std::vector<std::array<std::uint8_t, 3>> colors = {{9, 8, 7}};  // wrong length
  std::ostringstream os;
  slam::write_pcd(os, points, {}, colors);
  const auto [header, body] = split_pcd(os.str());

  EXPECT_EQ(header.find("rgb"), std::string::npos);
  EXPECT_EQ(body.size(), 2U * 3U * sizeof(float));
}

TEST(PointCloudIo, RoundTripsRgbThroughReadPcd)
{
  const std::vector<std::array<float, 3>> points = {{1.0F, 2.0F, 3.0F}, {4.0F, 5.0F, 6.0F}};
  const std::vector<float> intensities = {10.0F, 20.0F};
  const std::vector<std::array<std::uint8_t, 3>> colors = {{255, 128, 0}, {0, 64, 255}};
  std::ostringstream os;
  slam::write_pcd(os, points, intensities, colors);

  std::istringstream is(os.str());
  const auto result = slam::read_pcd(is);
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_EQ(result.cloud.points, points);
  EXPECT_EQ(result.cloud.intensities, intensities);
  EXPECT_EQ(result.cloud.colors, colors);
}

TEST(PointCloudIo, RoundTripsRgbWithoutIntensity)
{
  const std::vector<std::array<float, 3>> points = {{1.0F, 2.0F, 3.0F}};
  const std::vector<std::array<std::uint8_t, 3>> colors = {{12, 34, 56}};
  std::ostringstream os;
  slam::write_pcd(os, points, {}, colors);

  std::istringstream is(os.str());
  const auto result = slam::read_pcd(is);
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_EQ(result.cloud.points, points);
  EXPECT_TRUE(result.cloud.intensities.empty());
  EXPECT_EQ(result.cloud.colors, colors);
}

}  // namespace
