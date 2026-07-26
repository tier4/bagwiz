// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/deskew.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

namespace
{

using bagwiz::core::TrajectoryPose;
using bagwiz::core::pointcloud::deskew_pointcloud2;
using bagwiz::core::pointcloud::PointCloud2;
using bagwiz::core::pointcloud::PointField;
using bagwiz::core::pointcloud::PointFieldType;

// One point = [x y z t] as 4x float32 (point_step 16). `pts`: {x,y,z,t_seconds}.
PointCloud2 make_cloud_xyzt(const std::vector<std::array<float, 4>> & pts)
{
  PointCloud2 c;
  c.height = 1;
  c.width = static_cast<std::uint32_t>(pts.size());
  c.point_step = 16;
  c.row_step = c.point_step * c.width;
  c.is_bigendian = false;
  c.is_dense = true;
  c.frame_id = "lidar";
  c.fields = {
    {"x", 0, PointFieldType::kFloat32, 1},
    {"y", 4, PointFieldType::kFloat32, 1},
    {"z", 8, PointFieldType::kFloat32, 1},
    {"t", 12, PointFieldType::kFloat32, 1},
  };
  c.data.resize(static_cast<std::size_t>(c.point_step) * c.width);
  for (std::size_t i = 0; i < pts.size(); ++i) {
    std::memcpy(c.data.data() + i * c.point_step, pts[i].data(), sizeof(float) * 4);
  }
  return c;
}

std::array<float, 3> xyz_at(const PointCloud2 & c, std::size_t i)
{
  std::array<float, 3> o{};
  std::memcpy(o.data(), c.data.data() + i * c.point_step, sizeof(float) * 3);
  return o;
}

float t_at(const PointCloud2 & c, std::size_t i)
{
  float v;
  std::memcpy(&v, c.data.data() + i * c.point_step + 12, sizeof(float));
  return v;
}

// FLOAT64 variant of make_cloud_xyzt: one point = [x y z t] as 4x float64
// (point_step 32).
PointCloud2 make_cloud_xyzt_f64(const std::vector<std::array<double, 4>> & pts)
{
  PointCloud2 c;
  c.height = 1;
  c.width = static_cast<std::uint32_t>(pts.size());
  c.point_step = 32;
  c.row_step = c.point_step * c.width;
  c.is_bigendian = false;
  c.is_dense = true;
  c.frame_id = "lidar";
  c.fields = {
    {"x", 0, PointFieldType::kFloat64, 1},
    {"y", 8, PointFieldType::kFloat64, 1},
    {"z", 16, PointFieldType::kFloat64, 1},
    {"t", 24, PointFieldType::kFloat64, 1},
  };
  c.data.resize(static_cast<std::size_t>(c.point_step) * c.width);
  for (std::size_t i = 0; i < pts.size(); ++i) {
    std::memcpy(c.data.data() + i * c.point_step, pts[i].data(), sizeof(double) * 4);
  }
  return c;
}

double x_at_f64(const PointCloud2 & c, std::size_t i)
{
  double v;
  std::memcpy(&v, c.data.data() + i * c.point_step, sizeof(double));
  return v;
}

double t_at_f64(const PointCloud2 & c, std::size_t i)
{
  double v;
  std::memcpy(&v, c.data.data() + i * c.point_step + 24, sizeof(double));
  return v;
}

// x/y/z FLOAT32 (point_step 16) with a UINT32 nanosecond "t" field at offset 12,
// one point.
PointCloud2 make_cloud_xyz_u32time(float x, float y, float z, std::uint32_t t_ns)
{
  PointCloud2 c;
  c.height = 1;
  c.width = 1;
  c.point_step = 16;
  c.row_step = c.point_step;
  c.is_bigendian = false;
  c.is_dense = true;
  c.frame_id = "lidar";
  c.fields = {
    {"x", 0, PointFieldType::kFloat32, 1},
    {"y", 4, PointFieldType::kFloat32, 1},
    {"z", 8, PointFieldType::kFloat32, 1},
    {"t", 12, PointFieldType::kUint32, 1},
  };
  c.data.resize(c.point_step);
  const std::array<float, 3> xyz{x, y, z};
  std::memcpy(c.data.data(), xyz.data(), sizeof(float) * 3);
  std::memcpy(c.data.data() + 12, &t_ns, sizeof(t_ns));
  return c;
}

std::uint32_t time_u32_at(const PointCloud2 & c, std::size_t i)
{
  std::uint32_t v;
  std::memcpy(&v, c.data.data() + i * c.point_step + 12, sizeof(v));
  return v;
}

}  // namespace

TEST(Deskew, PureTranslationMovesPointToRefPose)
{
  // Sensor at t=0 at origin, at t=0.1s at x=+2. Point captured at t=0.1s with local x=0.
  // In the ref (t=0) frame the sensor is +2 ahead, so the point maps to x=+2.
  auto cloud = make_cloud_xyzt({{0.0f, 0.0f, 0.0f, 0.1f}});  // relative time 0.1s
  std::vector<TrajectoryPose> traj{{0, 0, 0, 0, 0, 0, 0, 1}, {100'000'000, 2, 0, 0, 0, 0, 0, 1}};
  auto r = deskew_pointcloud2(cloud, /*t_ref_ns=*/0, traj);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.points_deskewed, 1u);
  EXPECT_NEAR(xyz_at(*r.cloud, 0)[0], 2.0f, 1e-4);
  EXPECT_NEAR(t_at(*r.cloud, 0), 0.0f, 1e-6);  // relative time reset to 0
}

TEST(Deskew, RefTimePointUnchanged)
{
  auto cloud = make_cloud_xyzt({{1.0f, 2.0f, 3.0f, 0.0f}});
  std::vector<TrajectoryPose> traj{{0, 0, 0, 0, 0, 0, 0, 1}, {100'000'000, 5, 0, 0, 0, 0, 0, 1}};
  auto r = deskew_pointcloud2(cloud, 0, traj);
  ASSERT_TRUE(r.ok());
  EXPECT_NEAR(xyz_at(*r.cloud, 0)[0], 1.0f, 1e-5);  // t==t_ref -> no motion
  EXPECT_NEAR(t_at(*r.cloud, 0), 0.0f, 1e-6);       // time field reset to 0 too
}

TEST(Deskew, RejectsBigEndian)
{
  auto cloud = make_cloud_xyzt({{0, 0, 0, 0}});
  cloud.is_bigendian = true;
  auto r = deskew_pointcloud2(cloud, 0, std::vector<TrajectoryPose>{{0, 0, 0, 0, 0, 0, 0, 1}});
  EXPECT_FALSE(r.ok());
  EXPECT_NE(r.error.find("big-endian"), std::string::npos);
}

TEST(Deskew, NonFinitePointPassedThrough)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  auto cloud = make_cloud_xyzt({{nan, nan, nan, 0.05f}});
  std::vector<TrajectoryPose> traj{{0, 0, 0, 0, 0, 0, 0, 1}, {100'000'000, 9, 0, 0, 0, 0, 0, 1}};
  auto r = deskew_pointcloud2(cloud, 0, traj);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.points_nonfinite, 1u);
  EXPECT_TRUE(std::isnan(xyz_at(*r.cloud, 0)[0]));
}

TEST(Deskew, NoTimeFieldReturnsVerbatimWithCounter)
{
  PointCloud2 c;
  c.height = 1;
  c.width = 1;
  c.point_step = 12;
  c.row_step = 12;
  c.frame_id = "lidar";
  c.fields = {
    {"x", 0, PointFieldType::kFloat32, 1},
    {"y", 4, PointFieldType::kFloat32, 1},
    {"z", 8, PointFieldType::kFloat32, 1},
  };
  c.data.assign(12, std::byte{0});
  auto r = deskew_pointcloud2(c, 0, std::vector<TrajectoryPose>{{0, 0, 0, 0, 0, 0, 0, 1}});
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.points_no_time, 1u);
  EXPECT_EQ(r.points_deskewed, 0u);
}

TEST(Deskew, Float64XyzAndTime)
{
  // Same pure-translation scenario as PureTranslationMovesPointToRefPose, but
  // xyz + time are stored as FLOAT64 (point_step 32) to exercise the F64
  // load/store and F64 write_ref_time paths.
  auto cloud = make_cloud_xyzt_f64({{0.0, 0.0, 0.0, 0.1}});
  std::vector<TrajectoryPose> traj{{0, 0, 0, 0, 0, 0, 0, 1}, {100'000'000, 2, 0, 0, 0, 0, 0, 1}};
  auto r = deskew_pointcloud2(cloud, 0, traj);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.points_deskewed, 1u);
  EXPECT_NEAR(x_at_f64(*r.cloud, 0), 2.0, 1e-9);
  EXPECT_NEAR(t_at_f64(*r.cloud, 0), 0.0, 1e-12);  // relative time reset to 0
}

TEST(Deskew, Uint32NanosecondTimeResetsToZero)
{
  // Same pure-translation scenario, but the time field is UINT32 nanoseconds
  // (100'000'000 ns == 0.1s) instead of FLOAT32 seconds. write_ref_time's
  // UINT32 branch always writes 0 (ns-relative), regardless of the `relative`
  // flag, so this holds even though the flag is also true here.
  auto cloud = make_cloud_xyz_u32time(0.0f, 0.0f, 0.0f, 100'000'000u);
  std::vector<TrajectoryPose> traj{{0, 0, 0, 0, 0, 0, 0, 1}, {100'000'000, 2, 0, 0, 0, 0, 0, 1}};
  auto r = deskew_pointcloud2(cloud, 0, traj);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.points_deskewed, 1u);
  EXPECT_NEAR(xyz_at(*r.cloud, 0)[0], 2.0f, 1e-4);
  EXPECT_EQ(time_u32_at(*r.cloud, 0), 0u);
}

TEST(Deskew, OrganizedCloudHeightTwoWidthOne)
{
  // height=2, width=1, row_step=point_step (no row padding): each row is one
  // point. Row 0 repeats PureTranslationMovesPointToRefPose's point; row 1
  // repeats RefTimePointUnchanged's point. Exercises the height/row_step
  // addressing (as opposed to a single unorganized row of width points).
  auto cloud = make_cloud_xyzt({{0.0f, 0.0f, 0.0f, 0.1f}, {1.0f, 2.0f, 3.0f, 0.0f}});
  cloud.height = 2;
  cloud.width = 1;
  cloud.row_step = cloud.point_step;
  std::vector<TrajectoryPose> traj{{0, 0, 0, 0, 0, 0, 0, 1}, {100'000'000, 2, 0, 0, 0, 0, 0, 1}};
  auto r = deskew_pointcloud2(cloud, 0, traj);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.points_total, 2u);
  EXPECT_EQ(r.points_deskewed, 2u);
  EXPECT_NEAR(xyz_at(*r.cloud, 0)[0], 2.0f, 1e-4);  // row 0: moved to ref pose
  EXPECT_NEAR(xyz_at(*r.cloud, 1)[0], 1.0f, 1e-5);  // row 1: t==t_ref, unchanged
}

TEST(Deskew, NonIdentityExtrinsicRotatesTheMotionDelta)
{
  // extrinsic E = +90 deg about Z, zero translation (cloud frame -> traj
  // frame). The trajectory is a pure +2m translation along X between t=0 and
  // t=0.1s (identity rotation), a motion delta d=(2,0,0) in the traj frame.
  //
  // A *translation-only* extrinsic would be a degenerate choice here:
  // translations commute, so conjugating a translation-only trajectory step
  // by a translation-only E leaves the result completely independent of E
  // (E cancels exactly) -- that would pass even if E were ignored entirely.
  // A rotating E is required to actually exercise the extrinsic plumbing.
  //
  // Working through p' = E^-1 * (T_ref^-1 * T(t_i)) * E * p analytically:
  // T_ref^-1 * T(t_i) is the pure translation (I, d); conjugating it by
  // E = (R_E, 0) gives the pure translation (I, R_E^-1 * d) -- E's rotation
  // cancels out of the result's rotation but rotates the translation delta.
  // So p' = p + R_E^-1 * d.
  //
  // R_E^-1 is -90 deg about Z: R_E^-1 * (2,0,0) = (0,-2,0) (the same
  // "R_z(90 deg) * (1,0,0) -> (0,1,0)" convention independently exercised by
  // ComposeTrajectoryPose.ReferenceBridgeRotatesIntoFromFrame in
  // trajectory_test.cpp, read in reverse). A local point at (1,0,0) therefore
  // lands at (1,0,0) + (0,-2,0) = (1,-2,0).
  constexpr double kSinPiOver4 = 0.7071067811865476;
  geometry_msgs::msg::Transform extrinsic;
  extrinsic.translation.x = 0.0;
  extrinsic.translation.y = 0.0;
  extrinsic.translation.z = 0.0;
  extrinsic.rotation.x = 0.0;
  extrinsic.rotation.y = 0.0;
  extrinsic.rotation.z = kSinPiOver4;
  extrinsic.rotation.w = kSinPiOver4;

  auto cloud = make_cloud_xyzt({{1.0f, 0.0f, 0.0f, 0.1f}});
  std::vector<TrajectoryPose> traj{{0, 0, 0, 0, 0, 0, 0, 1}, {100'000'000, 2, 0, 0, 0, 0, 0, 1}};
  auto r = deskew_pointcloud2(cloud, 0, traj, extrinsic);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.points_deskewed, 1u);
  const auto xyz = xyz_at(*r.cloud, 0);
  EXPECT_NEAR(xyz[0], 1.0f, 1e-4);
  EXPECT_NEAR(xyz[1], -2.0f, 1e-4);
  EXPECT_NEAR(xyz[2], 0.0f, 1e-4);
}

TEST(Deskew, TimeFieldExceedingPointStepTreatedAsNoTime)
{
  // "t" declared as FLOAT64 (8 bytes) at offset 12 with point_step 16: the
  // field's own bytes [12,20) run 4 bytes past the point. Regression test for
  // an OOB read (the relative/absolute scan and point_time_seconds) and OOB
  // write (write_ref_time) this layout used to trigger -- corrupting the next
  // point's x and, for the last point, writing past the end of `data`
  // entirely. Built by hand (not make_cloud_xyzt, whose "t" is FLOAT32) so
  // the mismatched field/point_step combination is explicit.
  PointCloud2 c;
  c.height = 1;
  c.width = 2;
  c.point_step = 16;
  c.row_step = c.point_step * c.width;
  c.is_bigendian = false;
  c.is_dense = true;
  c.frame_id = "lidar";
  c.fields = {
    {"x", 0, PointFieldType::kFloat32, 1},
    {"y", 4, PointFieldType::kFloat32, 1},
    {"z", 8, PointFieldType::kFloat32, 1},
    {"t", 12, PointFieldType::kFloat64, 1},  // offset 12 + size 8 = 20 > point_step 16
  };
  c.data.resize(static_cast<std::size_t>(c.point_step) * c.width);
  const std::array<float, 3> p0{1.0f, 2.0f, 3.0f};
  const std::array<float, 3> p1{4.0f, 5.0f, 6.0f};
  std::memcpy(c.data.data(), p0.data(), sizeof(float) * 3);
  std::memcpy(c.data.data() + c.point_step, p1.data(), sizeof(float) * 3);

  std::vector<TrajectoryPose> traj{{0, 0, 0, 0, 0, 0, 0, 1}, {100'000'000, 2, 0, 0, 0, 0, 0, 1}};
  auto r = deskew_pointcloud2(c, 0, traj);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.points_total, 2u);
  EXPECT_EQ(r.points_no_time, 2u);
  EXPECT_EQ(r.points_deskewed, 0u);
  const auto xyz0 = xyz_at(*r.cloud, 0);
  const auto xyz1 = xyz_at(*r.cloud, 1);
  EXPECT_NEAR(xyz0[0], 1.0f, 1e-6);
  EXPECT_NEAR(xyz0[1], 2.0f, 1e-6);
  EXPECT_NEAR(xyz0[2], 3.0f, 1e-6);
  EXPECT_NEAR(xyz1[0], 4.0f, 1e-6);  // unchanged: not corrupted by point 0's time write
  EXPECT_NEAR(xyz1[1], 5.0f, 1e-6);
  EXPECT_NEAR(xyz1[2], 6.0f, 1e-6);
}

TEST(Deskew, RowStepSmallerThanWidthTimesPointStepIsError)
{
  // width=2, point_step=16 needs row_step >= 32; row_step is set to 16 (one
  // point's worth), which would let col=1's point run past the row (and, on
  // the last row, past `data`). Verifies the row_step hardening added
  // alongside the time-field bounds fix actually rejects this.
  auto cloud = make_cloud_xyzt({{0, 0, 0, 0}, {0, 0, 0, 0}});
  cloud.row_step = cloud.point_step;  // 16, but width*point_step = 32
  auto r = deskew_pointcloud2(cloud, 0, std::vector<TrajectoryPose>{{0, 0, 0, 0, 0, 0, 0, 1}});
  EXPECT_FALSE(r.ok());
  EXPECT_NE(r.error.find("row_step"), std::string::npos);
}

TEST(Deskew, OrganizedCloudWithRowPadding)
{
  // height=2, width=1, but row_step (32) is LARGER than width*point_step
  // (16): 16 bytes of padding after each row's single point. Complements
  // OrganizedCloudHeightTwoWidthOne (row_step == width*point_step, no
  // padding) by confirming the padding bytes are correctly skipped rather
  // than read as point data.
  PointCloud2 c;
  c.height = 2;
  c.width = 1;
  c.point_step = 16;
  c.row_step = 32;  // 16 bytes of padding per row
  c.is_bigendian = false;
  c.is_dense = true;
  c.frame_id = "lidar";
  c.fields = {
    {"x", 0, PointFieldType::kFloat32, 1},
    {"y", 4, PointFieldType::kFloat32, 1},
    {"z", 8, PointFieldType::kFloat32, 1},
    {"t", 12, PointFieldType::kFloat32, 1},
  };
  c.data.assign(static_cast<std::size_t>(c.row_step) * c.height, std::byte{0});
  const std::array<float, 4> row0{0.0f, 0.0f, 0.0f, 0.1f};  // PureTranslation-style point
  const std::array<float, 4> row1{1.0f, 2.0f, 3.0f, 0.0f};  // RefTimeUnchanged-style point
  std::memcpy(c.data.data(), row0.data(), sizeof(float) * 4);
  std::memcpy(c.data.data() + c.row_step, row1.data(), sizeof(float) * 4);

  std::vector<TrajectoryPose> traj{{0, 0, 0, 0, 0, 0, 0, 1}, {100'000'000, 2, 0, 0, 0, 0, 0, 1}};
  auto r = deskew_pointcloud2(c, 0, traj);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.points_deskewed, 2u);

  std::array<float, 3> xyz0{};
  std::memcpy(xyz0.data(), r.cloud->data.data(), sizeof(float) * 3);
  std::array<float, 3> xyz1{};
  std::memcpy(xyz1.data(), r.cloud->data.data() + c.row_step, sizeof(float) * 3);
  EXPECT_NEAR(xyz0[0], 2.0f, 1e-4);  // row 0: moved to ref pose
  EXPECT_NEAR(xyz1[0], 1.0f, 1e-5);  // row 1: t==t_ref, unchanged
}

TEST(Deskew, NonMonotonicPointTimes)
{
  // Point times jump backwards mid-scan (0.1 -> 0.05 -> 0.15 s relative):
  // exercises the trajectory lookup's non-monotone handling (each point must
  // still resolve its own pose, independent of scan order). The trajectory is
  // a pure +20 m/s X translation (2 m per 0.1 s), so a point at relative
  // time t_i maps to x + 20*t_i in the t_ref=0 frame.
  auto cloud = make_cloud_xyzt(
    {{1.0f, 0.0f, 0.0f, 0.1f}, {10.0f, 0.0f, 0.0f, 0.05f}, {100.0f, 0.0f, 0.0f, 0.15f}});
  std::vector<TrajectoryPose> traj{
    {0, 0, 0, 0, 0, 0, 0, 1},
    {100'000'000, 2, 0, 0, 0, 0, 0, 1},
    {200'000'000, 4, 0, 0, 0, 0, 0, 1}};
  auto r = deskew_pointcloud2(cloud, 0, traj);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.points_deskewed, 3u);
  EXPECT_NEAR(xyz_at(*r.cloud, 0)[0], 1.0f + 2.0f, 1e-4);    // t=0.1s: x + 20*0.1
  EXPECT_NEAR(xyz_at(*r.cloud, 1)[0], 10.0f + 1.0f, 1e-4);   // t=0.05s: x + 20*0.05
  EXPECT_NEAR(xyz_at(*r.cloud, 2)[0], 100.0f + 3.0f, 1e-4);  // t=0.15s: x + 20*0.15
}
