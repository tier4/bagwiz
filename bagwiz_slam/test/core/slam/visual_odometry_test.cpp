// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "visual_odometry.hpp"  // NOLINT(build/include_subdir) src-local header

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace
{
namespace slam = bagwiz::core::slam;

constexpr std::int64_t kPeriod = 100'000'000;
constexpr double kImuRate = 200.0;
constexpr double kWallX = 15.0;

// T_imu_cam for a forward-looking optical frame (z forward, x right, y down)
// mounted on a body frame with x forward, y left, z up: body x = optical z,
// body y = -optical x, body z = -optical y. Copied verbatim from
// visual_odometry_window_test.cpp (itself copied from visual_factors_test.cpp).
Eigen::Isometry3d optical_extrinsic()
{
  Eigen::Isometry3d extrinsic = Eigen::Isometry3d::Identity();
  extrinsic.linear() << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0;
  return extrinsic;
}

// Undistorted normalized image coordinates, matching what the frontend emits.
// Nullopt when the point is behind the camera. Copied verbatim from
// visual_odometry_window_test.cpp.
std::optional<Eigen::Vector2d> project(
  const Eigen::Isometry3d & T_world_cam, const Eigen::Vector3d & p_world)
{
  const Eigen::Vector3d p_cam = T_world_cam.inverse() * p_world;
  if (p_cam.z() <= 0.0) {
    return std::nullopt;
  }
  return Eigen::Vector2d(p_cam.x() / p_cam.z(), p_cam.y() / p_cam.z());
}

// 20 landmarks on the wall. Copied verbatim from visual_odometry_window_test.cpp.
std::vector<Eigen::Vector3d> wall_landmarks()
{
  std::vector<Eigen::Vector3d> landmarks;
  for (const double y : {-5.0, -2.5, 0.0, 2.5, 5.0}) {
    for (const double z : {1.0, 2.0, 3.0, 4.0}) {
      landmarks.emplace_back(kWallX, y, z);
    }
  }
  return landmarks;
}

// Synthetic world: z-up, gravity (0, 0, -9.81); body (= IMU) frame
// axis-aligned with world, ACCELERATING diagonally in x/y. Copied verbatim
// from visual_odometry_window_test.cpp -- see that file's comment on this
// function for the full rationale (scale observability, error absorbability).
Eigen::Isometry3d gt_pose(std::int64_t stamp_ns)  // T_world_imu at stamp
{
  const double t = 1e-9 * static_cast<double>(stamp_ns);
  const double s = t + 0.25 * t * t;
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(s, s, 0.0);
  return T;
}

// Adapted from visual_odometry_window_test.cpp's feed_imu: targets the
// estimator's own insert_imu (which forwards to both the window solver and,
// until initialized, the gravity-alignment initializer) instead of a bare
// WindowSolver.
void feed_imu(slam::VisualInertialOdometry & estimator, std::int64_t from_ns, std::int64_t to_ns)
{
  const std::int64_t step = static_cast<std::int64_t>(1e9 / kImuRate);
  for (std::int64_t t = from_ns; t <= to_ns; t += step) {
    estimator.insert_imu(
      1e-9 * static_cast<double>(t), Eigen::Vector3d(0.5, 0.5, 9.81), Eigen::Vector3d::Zero());
  }
}

// A purely static (non-accelerating) IMU feed: acc = (0, 0, 9.81), no
// horizontal component. Not one of the helpers copied from
// visual_odometry_window_test.cpp -- added because
// glim::NaiveInitialStateEstimation's gravity alignment (see its header
// comment: "would not work well when the sensor is moving") naively treats
// its own averaged accelerometer reading as pure gravity. Feeding it
// gt_pose's ACCELERATING reading (0.5, 0.5, 9.81) directly would tilt the
// estimated "up" axis toward the direction of travel, which then makes every
// subsequent IMU-only prediction (predict_T_world_imu, before any keyframe
// exists to supply a visual correction) come out near motionless regardless
// of elapsed time -- observed empirically while developing this test file
// (the displacement gate never opened past the first, unconditional
// keyframe). Feeding a static phase first -- standard practice for real IMU
// gravity alignment too (stay still, then start moving) -- keeps the
// estimated orientation aligned with gt_pose's true (identity) rotation, so
// the moving-world feed below actually produces the expected displacement.
void feed_static_imu(
  slam::VisualInertialOdometry & estimator, std::int64_t from_ns, std::int64_t to_ns)
{
  const std::int64_t step = static_cast<std::int64_t>(1e9 / kImuRate);
  for (std::int64_t t = from_ns; t <= to_ns; t += step) {
    estimator.insert_imu(
      1e-9 * static_cast<double>(t), Eigen::Vector3d(0.0, 0.0, 9.81), Eigen::Vector3d::Zero());
  }
}

// When the moving-world phase (see feed_static_imu's comment) begins:
// comfortably past NaiveInitialStateEstimation's ~1 s default init window,
// so gravity alignment is ready before any visual group ever arrives.
constexpr std::int64_t kMotionStart = 1'200'000'000;

// gt_pose, shifted so elapsed motion-world time restarts at 0 at
// kMotionStart -- test group anchors stay in the estimator's own absolute
// timeline (kMotionStart onward), while the synthetic trajectory itself
// still starts from gt_pose(0)'s initial conditions (v0 = (1, 1, 0)).
Eigen::Isometry3d gt_pose_shifted(std::int64_t stamp_ns)
{
  return gt_pose(stamp_ns - kMotionStart);
}

using PoseFn = Eigen::Isometry3d (*)(std::int64_t);

// One group: for each (camera_id, stamp offset) render every visible wall
// landmark through that camera at the true pose; track_id = landmark index.
// Copied verbatim from visual_odometry_window_test.cpp (the yawing-world
// pose_fn parameter is kept for fidelity to the original; no test below
// uses it -- NoFramesBeforeGravityAlignmentCompletes uses the default
// gt_pose directly, the rest pass gt_pose_shifted via push_groups()).
slam::ObservationGroup make_group(
  std::int64_t anchor_ns, const std::vector<std::pair<std::int32_t, std::int64_t>> & cams,
  const std::vector<Eigen::Isometry3d> & t_imu_cams, const std::vector<Eigen::Vector3d> & landmarks,
  PoseFn pose_fn = gt_pose)
{
  slam::ObservationGroup g;
  g.anchor_stamp_ns = anchor_ns;
  const auto anchor_rgb = static_cast<std::uint8_t>(anchor_ns / kPeriod);
  for (const auto & [cam, offset] : cams) {
    const Eigen::Isometry3d T_world_cam = pose_fn(anchor_ns + offset) * t_imu_cams[cam];
    for (std::size_t k = 0; k < landmarks.size(); ++k) {
      const auto uv = project(T_world_cam, landmarks[k]);
      if (!uv) continue;
      slam::VisualObservation o;
      o.camera_id = cam;
      o.track_id = k;
      o.stamp_ns = anchor_ns + offset;
      o.x = uv->x();
      o.y = uv->y();
      o.rgb = {anchor_rgb, 20, 30};
      g.observations.push_back(o);
    }
  }
  return g;
}

// Shared baseline config for every test below: one forward-looking camera,
// the grouping period as the window span, a displacement gate loose enough
// to require several periods' worth of motion, and a small window so
// marginalization is reachable within a short synthetic sequence.
slam::VisualInertialOdometryConfig make_config(const std::vector<Eigen::Isometry3d> & t_imu_cams)
{
  slam::VisualInertialOdometryConfig config;
  config.t_imu_cams = t_imu_cams;
  config.anchor_period_ns = kPeriod;
  config.keyframe_min_trans = 0.15;
  config.max_window_keyframes = 3;
  return config;
}

// Static calibration phase (see feed_static_imu's comment) followed by the
// moving-world IMU feed covering every group this test will push.
void prime_gravity_alignment(slam::VisualInertialOdometry & estimator, int num_groups)
{
  feed_static_imu(estimator, 0, kMotionStart);
  feed_imu(estimator, kMotionStart, kMotionStart + static_cast<std::int64_t>(num_groups) * kPeriod);
}

// Feeds `num_groups` single-camera groups, anchored every kPeriod starting at
// kMotionStart, through insert_visual_observations(); returns the
// accumulated marginalized_states (append-only, per the estimator's
// contract). Call prime_gravity_alignment() first.
std::vector<glim::EstimationFrame::ConstPtr> push_groups(
  slam::VisualInertialOdometry & estimator, const std::vector<Eigen::Isometry3d> & t_imu_cams,
  const std::vector<Eigen::Vector3d> & landmarks, int num_groups)
{
  std::vector<glim::EstimationFrame::ConstPtr> marginalized;
  for (int i = 0; i < num_groups; ++i) {
    const std::int64_t anchor = kMotionStart + static_cast<std::int64_t>(i) * kPeriod;
    const auto group = make_group(anchor, {{0, 0}}, t_imu_cams, landmarks, gt_pose_shifted);
    static_cast<void>(estimator.insert_visual_observations(group.observations, marginalized));
  }
  return marginalized;
}

// Checks the glim contract bullets shared by tests 3 and 4: frame_id, a
// non-null sparse cloud, no raw_frame, strictly increasing id/stamp across
// the sequence, at least 10 landmarks, and the T_world_lidar/T_world_imu
// sync convention. Mutates *previous_id / *previous_stamp for the next call.
void expect_glim_contract(
  const glim::EstimationFrame::ConstPtr & frame, const Eigen::Isometry3d & T_lidar_imu,
  std::int64_t * previous_id, double * previous_stamp)
{
  ASSERT_NE(frame, nullptr);
  EXPECT_EQ(frame->frame_id, glim::FrameID::IMU);
  ASSERT_NE(frame->frame, nullptr);
  // gtsam_points' voxel accumulation (SubMapping::insert_keyframe) reads
  // per-point covariances unconditionally; the placeholder covs must be there.
  EXPECT_TRUE(frame->frame->has_covs());
  EXPECT_EQ(frame->raw_frame, nullptr);
  EXPECT_GT(frame->id, *previous_id);
  EXPECT_GT(frame->stamp, *previous_stamp);
  *previous_id = frame->id;
  *previous_stamp = frame->stamp;

  EXPECT_GE(frame->frame->size(), 10u);

  const Eigen::Isometry3d expected_T_world_lidar = frame->T_world_imu * T_lidar_imu.inverse();
  EXPECT_TRUE(frame->T_world_lidar.isApprox(expected_T_world_lidar, 1e-9))
    << "T_world_lidar=\n"
    << frame->T_world_lidar.matrix() << "\nexpected=\n"
    << expected_T_world_lidar.matrix();
}

}  // namespace

TEST(VisualInertialOdometryTest, NoFramesBeforeGravityAlignmentCompletes)
{
  const std::vector<Eigen::Isometry3d> t_imu_cams = {optical_extrinsic()};
  const auto landmarks = wall_landmarks();

  slam::VisualInertialOdometry estimator(make_config(t_imu_cams));

  // Below NaiveInitialStateEstimation's ~1 s default init window.
  constexpr std::int64_t kBurnIn = 500'000'000;
  feed_imu(estimator, 0, kBurnIn);

  std::vector<glim::EstimationFrame::ConstPtr> marginalized;
  for (std::int64_t anchor = 0; anchor <= kBurnIn; anchor += kPeriod) {
    const auto group = make_group(anchor, {{0, 0}}, t_imu_cams, landmarks);
    const auto latest = estimator.insert_visual_observations(group.observations, marginalized);
    EXPECT_EQ(latest, nullptr);
  }

  EXPECT_TRUE(marginalized.empty());
  EXPECT_GT(estimator.stats().groups_before_init, 0);
}

TEST(VisualInertialOdometryTest, KeyframesFollowTheDisplacementGate)
{
  const std::vector<Eigen::Isometry3d> t_imu_cams = {optical_extrinsic()};
  const auto landmarks = wall_landmarks();

  slam::VisualInertialOdometry estimator(make_config(t_imu_cams));

  prime_gravity_alignment(estimator, 16);
  static_cast<void>(push_groups(estimator, t_imu_cams, landmarks, 16));

  const auto stats = estimator.stats();
  ASSERT_GT(stats.groups_total, 0);
  EXPECT_GT(stats.keyframes, 0);
  // 0.2 m of displacement (two periods, at ~1 m/s) clears the 0.15 m gate,
  // 0.1 m (one period) does not: keyframes should land roughly every other
  // group, so this loose upper bound catches a gate that fires far more
  // often than it should (e.g. every group).
  EXPECT_LE(stats.keyframes, stats.groups_total / 2 + 2);
}

TEST(VisualInertialOdometryTest, MarginalizedFramesHonorTheGlimContract)
{
  const std::vector<Eigen::Isometry3d> t_imu_cams = {optical_extrinsic()};
  const auto landmarks = wall_landmarks();

  auto config = make_config(t_imu_cams);
  // Non-identity, so the T_world_lidar sync path (set_T_world_sensor) is
  // actually exercised rather than trivially satisfied by T_lidar_imu = I.
  config.T_lidar_imu = optical_extrinsic().inverse();  // T_cam0_imu

  slam::VisualInertialOdometry estimator(config);
  prime_gravity_alignment(estimator, 20);

  const auto marginalized = push_groups(estimator, t_imu_cams, landmarks, 20);
  ASSERT_GE(marginalized.size(), 2u);

  std::int64_t previous_id = -1;
  double previous_stamp = -1.0;
  for (const auto & frame : marginalized) {
    expect_glim_contract(frame, config.T_lidar_imu, &previous_id, &previous_stamp);

    const auto stamp_ns = static_cast<std::int64_t>(std::llround(frame->stamp * 1e9));
    const double pos_error =
      (frame->T_world_imu.translation() - gt_pose_shifted(stamp_ns).translation()).norm();
    EXPECT_LT(pos_error, 0.05) << "stamp_ns=" << stamp_ns << " error=" << pos_error;
  }
}

TEST(VisualInertialOdometryTest, RemainingFramesDrainTheWindow)
{
  const std::vector<Eigen::Isometry3d> t_imu_cams = {optical_extrinsic()};
  const auto landmarks = wall_landmarks();

  auto config = make_config(t_imu_cams);
  config.T_lidar_imu = optical_extrinsic().inverse();

  slam::VisualInertialOdometry estimator(config);
  prime_gravity_alignment(estimator, 20);
  const auto already_marginalized = push_groups(estimator, t_imu_cams, landmarks, 20);

  const auto remaining = estimator.get_remaining_frames();
  const auto stats = estimator.stats();

  // grouping_.finish() may itself turn one more pending group into a
  // keyframe (GroupingBuffer's single-camera "off by one" lag: the very
  // last group pushed via insert_visual_observations() is never ready
  // until a later anchor confirms its window closed, so it is still
  // pending here) -- which, with the window already at capacity, both
  // grows stats().keyframes AND immediately marginalizes one more frame
  // before window_snapshot() is taken. So the robust invariant is not a
  // fixed min(...) count but that every keyframe ever accepted ends up
  // EITHER already reported (already_marginalized, from the earlier
  // push_groups() calls) OR in this final remaining batch.
  EXPECT_EQ(
    already_marginalized.size() + remaining.size(), static_cast<std::size_t>(stats.keyframes));
  // The window itself never exceeds max_window_keyframes, and at most one
  // further keyframe (the flushed pending group) is marginalized alongside
  // it in this same call.
  EXPECT_LE(remaining.size(), static_cast<std::size_t>(config.max_window_keyframes) + 1);
  EXPECT_GT(remaining.size(), 0u);

  std::int64_t previous_id = -1;
  double previous_stamp = -1.0;
  for (const auto & frame : remaining) {
    expect_glim_contract(frame, config.T_lidar_imu, &previous_id, &previous_stamp);
  }
}

TEST(VisualInertialOdometryTest, LandmarkCloudIsSensorLocal)
{
  const std::vector<Eigen::Isometry3d> t_imu_cams = {optical_extrinsic()};
  const auto landmarks = wall_landmarks();

  slam::VisualInertialOdometry estimator(make_config(t_imu_cams));
  prime_gravity_alignment(estimator, 20);

  const auto marginalized = push_groups(estimator, t_imu_cams, landmarks, 20);
  ASSERT_FALSE(marginalized.empty());

  const auto & frame = marginalized.front();
  ASSERT_GT(frame->frame->size(), 0u);

  for (std::size_t i = 0; i < frame->frame->size(); ++i) {
    const Eigen::Vector3d p_world = frame->T_world_imu * frame->frame->points[i].head<3>();
    double nearest = std::numeric_limits<double>::max();
    for (const auto & landmark : landmarks) {
      nearest = std::min(nearest, (p_world - landmark).norm());
    }
    EXPECT_LT(nearest, 0.2) << "p_world=" << p_world.transpose();
  }
}
