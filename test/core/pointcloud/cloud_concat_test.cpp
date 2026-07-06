// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/cloud_concat.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace
{

using bagwiz::core::pointcloud::concat_clouds;
using bagwiz::core::pointcloud::ConcatInput;
using bagwiz::core::pointcloud::PointCloud2;
using bagwiz::core::pointcloud::PointFieldType;

constexpr std::int64_t kSec = 1'000'000'000LL;

// Cloud with x/y/z (FLOAT32) and a FLOAT32 per-point `time` at offset 12,
// point_step 16. Each element of `pts` is {x, y, z, time}.
PointCloud2 make_cloud(const std::vector<std::array<float, 4>> & pts, bool dense = true)
{
  PointCloud2 c;
  c.height = 1;
  c.width = static_cast<std::uint32_t>(pts.size());
  c.fields = {
    {"x", 0, PointFieldType::kFloat32, 1},
    {"y", 4, PointFieldType::kFloat32, 1},
    {"z", 8, PointFieldType::kFloat32, 1},
    {"time", 12, PointFieldType::kFloat32, 1},
  };
  c.point_step = 16;
  c.row_step = 16 * c.width;
  c.is_dense = dense;
  c.data.assign(static_cast<std::size_t>(c.width) * 16, std::byte{0});
  for (std::size_t i = 0; i < pts.size(); ++i) {
    std::memcpy(c.data.data() + i * 16, pts[i].data(), sizeof(float) * 4);
  }
  return c;
}

float time_at(const PointCloud2 & c, std::size_t i)
{
  float t = 0.0f;
  std::memcpy(&t, c.data.data() + i * c.point_step + 12, sizeof(float));
  return t;
}
float x_at(const PointCloud2 & c, std::size_t i)
{
  float v = 0.0f;
  std::memcpy(&v, c.data.data() + i * c.point_step, sizeof(float));
  return v;
}

// Cloud with x/y/z (FLOAT32) and a UINT32-ns per-point `time` at offset 12,
// point_step 16. `xyz` gives the coordinates and `t_ns` each point's time.
PointCloud2 make_cloud_u32(
  const std::vector<std::array<float, 3>> & xyz, const std::vector<std::uint32_t> & t_ns)
{
  PointCloud2 c;
  c.height = 1;
  c.width = static_cast<std::uint32_t>(xyz.size());
  c.fields = {
    {"x", 0, PointFieldType::kFloat32, 1},
    {"y", 4, PointFieldType::kFloat32, 1},
    {"z", 8, PointFieldType::kFloat32, 1},
    {"time", 12, PointFieldType::kUint32, 1},
  };
  c.point_step = 16;
  c.row_step = 16 * c.width;
  c.is_dense = true;
  c.data.assign(static_cast<std::size_t>(c.width) * 16, std::byte{0});
  for (std::size_t i = 0; i < xyz.size(); ++i) {
    std::memcpy(c.data.data() + i * 16, xyz[i].data(), sizeof(float) * 3);
    std::memcpy(c.data.data() + i * 16 + 12, &t_ns[i], sizeof(std::uint32_t));
  }
  return c;
}

}  // namespace

TEST(CloudConcat, ConcatenatesInOrderAndSumsWidth)
{
  const auto a = make_cloud({{1.0f, 0, 0, 0.0f}, {2.0f, 0, 0, 0.0f}});
  const auto b = make_cloud({{3.0f, 0, 0, 0.0f}});
  const std::array<ConcatInput, 2> inputs{ConcatInput{&a, 0}, ConcatInput{&b, 0}};

  const auto r = concat_clouds(inputs, 0, "base_link");
  ASSERT_TRUE(r.ok()) << r.error;
  EXPECT_EQ(r.cloud->frame_id, "base_link");
  EXPECT_EQ(r.cloud->height, 1u);
  EXPECT_EQ(r.cloud->width, 3u);
  EXPECT_EQ(r.cloud->row_step, 48u);
  EXPECT_FLOAT_EQ(x_at(*r.cloud, 0), 1.0f);
  EXPECT_FLOAT_EQ(x_at(*r.cloud, 1), 2.0f);
  EXPECT_FLOAT_EQ(x_at(*r.cloud, 2), 3.0f);
}

TEST(CloudConcat, MismatchedLayoutIsError)
{
  const auto a = make_cloud({{1.0f, 0, 0, 0.0f}});
  PointCloud2 b;  // different layout: only x/y
  b.height = 1;
  b.width = 1;
  b.fields = {{"x", 0, PointFieldType::kFloat32, 1}, {"y", 4, PointFieldType::kFloat32, 1}};
  b.point_step = 8;
  b.data.assign(8, std::byte{0});
  const std::array<ConcatInput, 2> inputs{ConcatInput{&a, 0}, ConcatInput{&b, 0}};

  const auto r = concat_clouds(inputs, 0, "base_link");
  EXPECT_FALSE(r.ok());
  EXPECT_FALSE(r.error.empty());
}

TEST(CloudConcat, IsDenseIsLogicalAnd)
{
  const auto a = make_cloud({{1.0f, 0, 0, 0.0f}}, /*dense=*/true);
  const auto b = make_cloud({{2.0f, 0, 0, 0.0f}}, /*dense=*/false);
  const std::array<ConcatInput, 2> inputs{ConcatInput{&a, 0}, ConcatInput{&b, 0}};
  const auto r = concat_clouds(inputs, 0, "f");
  ASSERT_TRUE(r.ok()) << r.error;
  EXPECT_FALSE(r.cloud->is_dense);
}

// Header-relative per-point time is re-based so the absolute acquisition time is
// preserved: out_stamp + t' == header_k + t. The reference (delta 0) is
// unchanged; the earlier cloud's relative time shifts by (H_k - out_stamp).
TEST(CloudConcat, RelativeTimeRebasedToPreserveAbsolute)
{
  const std::int64_t h_ref = 1700 * kSec;          // reference header stamp
  const std::int64_t h_early = h_ref - kSec / 20;  // 50 ms earlier
  const auto ref = make_cloud({{1.0f, 0, 0, 0.02f}});
  const auto early = make_cloud({{2.0f, 0, 0, 0.03f}});
  const std::array<ConcatInput, 2> inputs{ConcatInput{&ref, h_ref}, ConcatInput{&early, h_early}};

  const auto r = concat_clouds(inputs, h_ref, "base_link");
  ASSERT_TRUE(r.ok()) << r.error;
  // reference point: delta 0, time unchanged.
  EXPECT_NEAR(time_at(*r.cloud, 0), 0.02f, 1e-6);
  // early point: t' = 0.03 + (h_early - h_ref)s = 0.03 - 0.05 = -0.02.
  EXPECT_NEAR(time_at(*r.cloud, 1), -0.02f, 1e-6);
  // absolute time is preserved: out_stamp + t' == h_early + t.
  const double abs_out = static_cast<double>(h_ref) * 1e-9 + time_at(*r.cloud, 1);
  const double abs_in = static_cast<double>(h_early) * 1e-9 + 0.03;
  EXPECT_NEAR(abs_out, abs_in, 1e-6);
}

// Absolute (epoch-scale) per-point time is copied verbatim regardless of the
// output stamp — it does not depend on the header, so re-basing would corrupt it.
TEST(CloudConcat, AbsoluteTimeCopiedVerbatim)
{
  const std::int64_t h_ref = 1700 * kSec;
  const std::int64_t h_other = h_ref - kSec / 20;
  const float abs_t = 1700.03f;  // epoch-scale seconds, near the header
  const auto a = make_cloud({{1.0f, 0, 0, 1700.00f}});
  const auto b = make_cloud({{2.0f, 0, 0, abs_t}});
  const std::array<ConcatInput, 2> inputs{ConcatInput{&a, h_ref}, ConcatInput{&b, h_other}};

  const auto r = concat_clouds(inputs, h_ref, "base_link");
  ASSERT_TRUE(r.ok()) << r.error;
  EXPECT_FLOAT_EQ(time_at(*r.cloud, 1), abs_t);  // unchanged
}

// A UINT32-ns header-relative time cannot hold the negative value re-basing needs
// when an input is earlier than out_stamp, so the output emits it as FLOAT32
// seconds (same 4-byte slot, point_step unchanged) and preserves each point's
// absolute time.
TEST(CloudConcat, Uint32TimeConvertedToFloat32)
{
  const std::int64_t h_ref = 1700 * kSec;
  const std::int64_t h_early = h_ref - kSec / 20;                   // 50 ms earlier
  const auto ref = make_cloud_u32({{1.0f, 0, 0}}, {20'000'000});    // t = 20 ms
  const auto early = make_cloud_u32({{2.0f, 0, 0}}, {30'000'000});  // t = 30 ms
  const std::array<ConcatInput, 2> inputs{ConcatInput{&ref, h_ref}, ConcatInput{&early, h_early}};

  const auto r = concat_clouds(inputs, h_ref, "base_link");
  ASSERT_TRUE(r.ok()) << r.error;
  EXPECT_EQ(r.cloud->width, 2u);
  EXPECT_EQ(r.cloud->point_step, 16u);  // same 4-byte slot, no widening
  ASSERT_EQ(r.cloud->fields.size(), 4u);
  EXPECT_EQ(r.cloud->fields[3].name, "time");
  EXPECT_EQ(r.cloud->fields[3].datatype, PointFieldType::kFloat32);
  EXPECT_EQ(r.cloud->fields[3].offset, 12u);
  EXPECT_FLOAT_EQ(x_at(*r.cloud, 0), 1.0f);
  EXPECT_FLOAT_EQ(x_at(*r.cloud, 1), 2.0f);
  // reference point: delta 0 -> 0.02 s.
  EXPECT_NEAR(time_at(*r.cloud, 0), 0.02f, 1e-6);
  // early point: 0.03 + (h_early - h_ref) = 0.03 - 0.05 = -0.02 s (negative, OK).
  EXPECT_NEAR(time_at(*r.cloud, 1), -0.02f, 1e-6);
  // absolute time preserved: out_stamp + t' == h_early + 0.03.
  const double abs_out = static_cast<double>(h_ref) * 1e-9 + time_at(*r.cloud, 1);
  const double abs_in = static_cast<double>(h_early) * 1e-9 + 0.03;
  EXPECT_NEAR(abs_out, abs_in, 1e-6);
}

// A FLOAT64 header-relative time is re-based in place (rebase_time's FLOAT64
// branch), preserving each point's absolute time exactly.
TEST(CloudConcat, Float64RelativeTimeRebased)
{
  const std::int64_t h_ref = 1700 * kSec;
  const std::int64_t h_early = h_ref - kSec / 20;  // 50 ms earlier
  const auto make_f64 = [](float x, double t) {
    PointCloud2 c;
    c.height = 1;
    c.width = 1;
    c.fields = {{"x", 0, PointFieldType::kFloat32, 1}, {"time", 4, PointFieldType::kFloat64, 1}};
    c.point_step = 12;
    c.is_dense = true;
    c.data.assign(12, std::byte{0});
    std::memcpy(c.data.data(), &x, sizeof(x));
    std::memcpy(c.data.data() + 4, &t, sizeof(t));
    return c;
  };
  const auto ref = make_f64(1.0f, 0.02);
  const auto early = make_f64(2.0f, 0.03);
  const std::array<ConcatInput, 2> inputs{ConcatInput{&ref, h_ref}, ConcatInput{&early, h_early}};
  const auto r = concat_clouds(inputs, h_ref, "base_link");
  ASSERT_TRUE(r.ok()) << r.error;
  EXPECT_EQ(r.cloud->fields[1].datatype, PointFieldType::kFloat64);
  double t0 = 0.0;
  double t1 = 0.0;
  std::memcpy(&t0, r.cloud->data.data() + 4, sizeof(double));
  std::memcpy(&t1, r.cloud->data.data() + r.cloud->point_step + 4, sizeof(double));
  EXPECT_NEAR(t0, 0.02, 1e-12);   // reference: delta 0
  EXPECT_NEAR(t1, -0.02, 1e-12);  // 0.03 + (h_early - h_ref)
}

// With no recognised per-point time field, clouds are concatenated verbatim.
TEST(CloudConcat, NoTimeFieldConcatenatesVerbatim)
{
  const auto make_xyz = [](float x) {
    PointCloud2 c;
    c.height = 1;
    c.width = 1;
    c.fields = {
      {"x", 0, PointFieldType::kFloat32, 1},
      {"y", 4, PointFieldType::kFloat32, 1},
      {"z", 8, PointFieldType::kFloat32, 1},
    };
    c.point_step = 12;
    c.is_dense = true;
    c.data.assign(12, std::byte{0});
    std::memcpy(c.data.data(), &x, sizeof(x));
    return c;
  };
  const auto a = make_xyz(1.0f);
  const auto b = make_xyz(2.0f);
  const std::array<ConcatInput, 2> inputs{
    ConcatInput{&a, 1700 * kSec}, ConcatInput{&b, 1699 * kSec}};
  const auto r = concat_clouds(inputs, 1700 * kSec, "base_link");
  ASSERT_TRUE(r.ok()) << r.error;
  EXPECT_EQ(r.cloud->width, 2u);
  EXPECT_EQ(r.cloud->point_step, 12u);
  EXPECT_FLOAT_EQ(x_at(*r.cloud, 0), 1.0f);
  EXPECT_FLOAT_EQ(x_at(*r.cloud, 1), 2.0f);
}

TEST(CloudConcat, BigEndianIsError)
{
  auto a = make_cloud({{1.0f, 0, 0, 0.0f}});
  a.is_bigendian = true;
  const std::array<ConcatInput, 1> inputs{ConcatInput{&a, 0}};
  const auto r = concat_clouds(inputs, 0, "f");
  EXPECT_FALSE(r.ok());
  EXPECT_FALSE(r.error.empty());
}

TEST(CloudConcat, FlattensOrganizedCloud)
{
  auto org = make_cloud({{1, 0, 0, 0}, {2, 0, 0, 0}, {3, 0, 0, 0}, {4, 0, 0, 0}});
  org.height = 2;
  org.width = 2;  // 2x2 organized
  const std::array<ConcatInput, 1> inputs{ConcatInput{&org, 0}};
  const auto r = concat_clouds(inputs, 0, "f");
  ASSERT_TRUE(r.ok()) << r.error;
  EXPECT_EQ(r.cloud->height, 1u);
  EXPECT_EQ(r.cloud->width, 4u);
}

TEST(CloudConcat, EmptyInputsIsError)
{
  const auto r = concat_clouds({}, 0, "f");
  EXPECT_FALSE(r.ok());
  EXPECT_FALSE(r.error.empty());
}

// A time field whose offset + size exceeds point_step would read past the buffer
// during re-basing; it must be rejected.
TEST(CloudConcat, TimeFieldPastPointStepIsError)
{
  PointCloud2 c;
  c.height = 1;
  c.width = 1;
  c.fields = {
    {"x", 0, PointFieldType::kFloat32, 1},
    {"y", 4, PointFieldType::kFloat32, 1},
    {"z", 8, PointFieldType::kFloat32, 1},
    {"time", 12, PointFieldType::kUint32, 1},  // 12 + 4 = 16 > point_step 14
  };
  c.point_step = 14;
  c.data.assign(14, std::byte{0});
  const std::array<ConcatInput, 1> inputs{ConcatInput{&c, 1000}};
  const auto r = concat_clouds(inputs, 1000, "base_link");
  EXPECT_FALSE(r.ok());
  EXPECT_FALSE(r.error.empty());
}
