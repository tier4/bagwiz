// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/fetcher.hpp"
#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/core/pointcloud/property.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace
{

using bagwiz::core::pointcloud::accumulate_property_ranges;
using bagwiz::core::pointcloud::PointCloud2;
using bagwiz::core::pointcloud::PointCloudProperty;
using bagwiz::core::pointcloud::PointField;
using bagwiz::core::pointcloud::PointFieldType;
using bagwiz::core::pointcloud::PropertyRanges;

struct Pt
{
  float x;
  float y;
  float z;
  float intensity;
};

// Build an in-memory, densely packed PointCloud2 (float32 x/y/z, optional
// float32 intensity) so the range accumulator can be exercised without a bag.
PointCloud2 make_cloud(const std::vector<Pt> & pts, bool with_intensity)
{
  PointCloud2 cloud;
  cloud.height = 1;
  cloud.width = static_cast<std::uint32_t>(pts.size());
  const std::uint32_t step = with_intensity ? 16U : 12U;
  cloud.point_step = step;
  cloud.row_step = step * cloud.width;
  cloud.fields.push_back({"x", 0U, PointFieldType::kFloat32, 1U});
  cloud.fields.push_back({"y", 4U, PointFieldType::kFloat32, 1U});
  cloud.fields.push_back({"z", 8U, PointFieldType::kFloat32, 1U});
  if (with_intensity) {
    cloud.fields.push_back({"intensity", 12U, PointFieldType::kFloat32, 1U});
  }
  cloud.data.assign(static_cast<std::size_t>(step) * pts.size(), std::byte{0});
  for (std::size_t i = 0; i < pts.size(); ++i) {
    std::byte * base = cloud.data.data() + i * step;
    std::memcpy(base + 0, &pts[i].x, sizeof(float));
    std::memcpy(base + 4, &pts[i].y, sizeof(float));
    std::memcpy(base + 8, &pts[i].z, sizeof(float));
    if (with_intensity) {
      std::memcpy(base + 12, &pts[i].intensity, sizeof(float));
    }
  }
  return cloud;
}

TEST(PropertyRanges, DefaultResolveIsNeutralFallback)
{
  // A freshly constructed range has seen nothing, so every property resolves to
  // the neutral [0, 1] fallback rather than the +inf/-inf sentinels.
  const PropertyRanges ranges;
  for (auto prop :
       {PointCloudProperty::kX, PointCloudProperty::kY, PointCloudProperty::kZ,
        PointCloudProperty::kDistance, PointCloudProperty::kIntensity}) {
    const auto [lo, hi] = ranges.resolve(prop);
    EXPECT_DOUBLE_EQ(lo, 0.0);
    EXPECT_DOUBLE_EQ(hi, 1.0);
  }
  EXPECT_FALSE(ranges.has_intensity);
}

TEST(PropertyRanges, AccumulateComputesEveryPropertyInOnePass)
{
  const auto cloud = make_cloud(
    {{1.0F, 2.0F, 3.0F, 0.0F}, {-4.0F, 0.0F, 0.0F, 0.0F}, {0.0F, 5.0F, 0.0F, 0.0F}},
    /*with_intensity=*/false);
  PropertyRanges ranges;
  std::string error;
  ASSERT_TRUE(accumulate_property_ranges(cloud, ranges, error)) << error;

  const auto x = ranges.resolve(PointCloudProperty::kX);
  EXPECT_DOUBLE_EQ(x.first, -4.0);
  EXPECT_DOUBLE_EQ(x.second, 1.0);
  const auto y = ranges.resolve(PointCloudProperty::kY);
  EXPECT_DOUBLE_EQ(y.first, 0.0);
  EXPECT_DOUBLE_EQ(y.second, 5.0);
  const auto z = ranges.resolve(PointCloudProperty::kZ);
  EXPECT_DOUBLE_EQ(z.first, 0.0);
  EXPECT_DOUBLE_EQ(z.second, 3.0);

  const auto dist = ranges.resolve(PointCloudProperty::kDistance);
  EXPECT_FLOAT_EQ(static_cast<float>(dist.first), std::sqrt(14.0F));  // (1,2,3)
  EXPECT_FLOAT_EQ(static_cast<float>(dist.second), 5.0F);             // (0,5,0)

  // No intensity field: flag stays false and intensity resolves to fallback.
  EXPECT_FALSE(ranges.has_intensity);
  const auto intensity = ranges.resolve(PointCloudProperty::kIntensity);
  EXPECT_DOUBLE_EQ(intensity.first, 0.0);
  EXPECT_DOUBLE_EQ(intensity.second, 1.0);
}

TEST(PropertyRanges, AccumulateCapturesIntensityWhenPresent)
{
  const auto cloud = make_cloud(
    {{0.0F, 0.0F, 0.0F, 10.0F}, {0.0F, 0.0F, 0.0F, 255.0F}, {0.0F, 0.0F, 0.0F, 100.0F}},
    /*with_intensity=*/true);
  PropertyRanges ranges;
  std::string error;
  ASSERT_TRUE(accumulate_property_ranges(cloud, ranges, error)) << error;

  EXPECT_TRUE(ranges.has_intensity);
  const auto intensity = ranges.resolve(PointCloudProperty::kIntensity);
  EXPECT_DOUBLE_EQ(intensity.first, 10.0);
  EXPECT_DOUBLE_EQ(intensity.second, 255.0);
}

TEST(PropertyRanges, AccumulateFoldsSuccessiveClouds)
{
  // Accumulating a second cloud into the same running range widens the bounds,
  // mirroring scan_point_cloud folding every message of a topic.
  PropertyRanges ranges;
  std::string error;
  ASSERT_TRUE(
    accumulate_property_ranges(make_cloud({{1.0F, 0.0F, 0.0F, 0.0F}}, false), ranges, error));
  ASSERT_TRUE(
    accumulate_property_ranges(make_cloud({{-9.0F, 0.0F, 0.0F, 0.0F}}, false), ranges, error));

  const auto x = ranges.resolve(PointCloudProperty::kX);
  EXPECT_DOUBLE_EQ(x.first, -9.0);
  EXPECT_DOUBLE_EQ(x.second, 1.0);
}

TEST(PropertyRanges, MergeCombinesTwoTopics)
{
  PropertyRanges a;
  PropertyRanges b;
  std::string error;
  ASSERT_TRUE(accumulate_property_ranges(make_cloud({{-4.0F, 0.0F, 0.0F, 0.0F}}, false), a, error));
  ASSERT_TRUE(accumulate_property_ranges(make_cloud({{10.0F, 0.0F, 0.0F, 7.0F}}, true), b, error));

  a.merge(b);
  const auto x = a.resolve(PointCloudProperty::kX);
  EXPECT_DOUBLE_EQ(x.first, -4.0);
  EXPECT_DOUBLE_EQ(x.second, 10.0);
  // Intensity presence propagates from either side of the merge.
  EXPECT_TRUE(a.has_intensity);
}

TEST(PropertyRanges, AccumulateFailsWithoutXyz)
{
  PointCloud2 cloud;
  cloud.height = 1;
  cloud.width = 1;
  cloud.point_step = 4;
  cloud.row_step = 4;
  cloud.fields.push_back({"intensity", 0U, PointFieldType::kFloat32, 1U});
  cloud.data.assign(4, std::byte{0});

  PropertyRanges ranges;
  std::string error;
  EXPECT_FALSE(accumulate_property_ranges(cloud, ranges, error));
  EXPECT_FALSE(error.empty());
}

}  // namespace
