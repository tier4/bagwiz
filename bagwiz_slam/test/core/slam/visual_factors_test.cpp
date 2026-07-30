// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "visual_factors.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <vector>

namespace
{
namespace visual = bagwiz::core::slam::visual;

Eigen::Isometry3d make_pose(double x, double y, double z, double yaw_rad)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(x, y, z);
  pose.linear() = Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return pose;
}

// Two frames: identity at t=0, translation (1,0,0) + yaw 90 deg at t=1.
visual::SubmapView make_two_frame_view()
{
  visual::SubmapView view;
  view.id = 1;
  view.T_world_origin = Eigen::Isometry3d::Identity();
  view.frame_stamps = {0.0, 1.0};
  view.T_origin_frames = {make_pose(0.0, 0.0, 0.0, 0.0), make_pose(1.0, 0.0, 0.0, M_PI / 2.0)};
  return view;
}

void expect_pose_near(
  const Eigen::Isometry3d & actual, const Eigen::Isometry3d & expected, double tol)
{
  const double translation_error = (actual.translation() - expected.translation()).norm();
  EXPECT_LT(translation_error, tol) << "translation actual=" << actual.translation().transpose()
                                    << " expected=" << expected.translation().transpose();
  const Eigen::Quaterniond q_actual(actual.rotation());
  const Eigen::Quaterniond q_expected(expected.rotation());
  // Compare via the angular difference so the double-cover (q vs -q) doesn't
  // spuriously fail the comparison.
  const double angle = Eigen::AngleAxisd(q_actual.inverse() * q_expected).angle();
  EXPECT_LT(std::abs(angle), tol) << "rotation angle diff=" << angle;
}

}  // namespace

TEST(VisualFactorsTest, InterpolateAtFrameStampIsExact)
{
  const auto view = make_two_frame_view();

  const auto at_start = visual::interpolate_origin_pose(view, 0.0);
  ASSERT_TRUE(at_start.has_value());
  expect_pose_near(*at_start, Eigen::Isometry3d::Identity(), 1e-12);

  const auto at_end = visual::interpolate_origin_pose(view, 1.0);
  ASSERT_TRUE(at_end.has_value());
  expect_pose_near(*at_end, view.T_origin_frames.back(), 1e-12);
}

TEST(VisualFactorsTest, InterpolateMidpointSlerps)
{
  const auto view = make_two_frame_view();

  const auto mid = visual::interpolate_origin_pose(view, 0.5);
  ASSERT_TRUE(mid.has_value());
  expect_pose_near(*mid, make_pose(0.5, 0.0, 0.0, M_PI / 4.0), 1e-9);
}

TEST(VisualFactorsTest, OutsideSpanReturnsNullopt)
{
  const auto view = make_two_frame_view();

  EXPECT_FALSE(visual::interpolate_origin_pose(view, -0.1).has_value());
  EXPECT_FALSE(visual::interpolate_origin_pose(view, 1.1).has_value());
}

TEST(VisualFactorsTest, SubmapForStampPicksContainingSpanAndRejectsGaps)
{
  visual::SubmapView view_a;
  view_a.id = 0;
  view_a.frame_stamps = {0.0, 1.0};
  view_a.T_origin_frames = {Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity()};

  visual::SubmapView view_b;
  view_b.id = 1;
  view_b.frame_stamps = {2.0, 3.0};
  view_b.T_origin_frames = {Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity()};

  const std::vector<visual::SubmapView> views = {view_a, view_b};

  EXPECT_EQ(visual::submap_for_stamp(views, 0.5), 0u);
  EXPECT_EQ(visual::submap_for_stamp(views, 2.5), 1u);
  EXPECT_EQ(visual::submap_for_stamp(views, 1.5), std::nullopt);
  EXPECT_EQ(visual::submap_for_stamp(views, 3.5), std::nullopt);
}
