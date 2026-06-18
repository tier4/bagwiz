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
#include <cstring>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace
{
namespace slam = bagwiz::core::slam;

// Split a PLY stream into its ASCII header (through "end_header\n") and the
// binary body that follows.
std::pair<std::string, std::string> split_ply(const std::string & blob)
{
  const std::string marker = "end_header\n";
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

TEST(PointCloudIo, WritesXyzPlyHeaderAndBody)
{
  const std::vector<std::array<float, 3>> points = {{1.0F, 2.0F, 3.0F}, {4.0F, 5.0F, 6.0F}};
  std::ostringstream os;
  slam::write_ply(os, points);
  const auto [header, body] = split_ply(os.str());

  EXPECT_NE(header.find("ply\n"), std::string::npos);
  EXPECT_NE(header.find("format binary_little_endian 1.0\n"), std::string::npos);
  EXPECT_NE(header.find("element vertex 2\n"), std::string::npos);
  EXPECT_NE(header.find("property float x\n"), std::string::npos);
  EXPECT_NE(header.find("property float z\n"), std::string::npos);
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
  slam::write_ply(os, points, intensities);
  const auto [header, body] = split_ply(os.str());

  EXPECT_NE(header.find("property float intensity\n"), std::string::npos);
  ASSERT_EQ(body.size(), 2U * 4U * sizeof(float));
  EXPECT_FLOAT_EQ(read_float(body, 3), 10.0F);  // first point's intensity
  EXPECT_FLOAT_EQ(read_float(body, 7), 20.0F);  // second point's intensity
}

TEST(PointCloudIo, OmitsIntensityOnSizeMismatch)
{
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 0.0F}, {1.0F, 1.0F, 1.0F}};
  const std::vector<float> intensities = {10.0F};  // wrong length
  std::ostringstream os;
  slam::write_ply(os, points, intensities);
  const auto [header, body] = split_ply(os.str());

  EXPECT_EQ(header.find("intensity"), std::string::npos);
  EXPECT_EQ(body.size(), 2U * 3U * sizeof(float));
}

TEST(PointCloudIo, EmptyCloudWritesZeroVertices)
{
  std::ostringstream os;
  slam::write_ply(os, {});
  const auto [header, body] = split_ply(os.str());
  EXPECT_NE(header.find("element vertex 0\n"), std::string::npos);
  EXPECT_TRUE(body.empty());
}

}  // namespace
