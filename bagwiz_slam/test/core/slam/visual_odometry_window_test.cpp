// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "visual_odometry_window.hpp"  // NOLINT(build/include_subdir) src-local header

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
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
// body y = -optical x, body z = -optical y. Copied from
// visual_factors_test.cpp:82-124 (T_lidar_cam there is T_imu_cam here; same
// geometry, same conventions).
Eigen::Isometry3d optical_extrinsic()
{
  Eigen::Isometry3d extrinsic = Eigen::Isometry3d::Identity();
  extrinsic.linear() << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0;
  return extrinsic;
}

// Undistorted normalized image coordinates, matching what the frontend emits.
// Nullopt when the point is behind the camera.
std::optional<Eigen::Vector2d> project(
  const Eigen::Isometry3d & T_world_cam, const Eigen::Vector3d & p_world)
{
  const Eigen::Vector3d p_cam = T_world_cam.inverse() * p_world;
  if (p_cam.z() <= 0.0) {
    return std::nullopt;
  }
  return Eigen::Vector2d(p_cam.x() / p_cam.z(), p_cam.y() / p_cam.z());
}

// 20 landmarks on the wall, none of them on the camera's optical axis (a point
// dead ahead has no parallax from the trajectory's forward component of
// motion, which is what actually matters for this choice -- the trajectory
// itself is diagonal, not purely forward).
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
// axis-aligned with world, ACCELERATING diagonally in x/y:
// v(t) = (1 + 0.5t, 1 + 0.5t, 0), p(t) = (s(t), s(t), 0) with
// s(t) = t + 0.25t^2, so a perfect IMU reads acc = (0.5, 0.5, +9.81),
// gyro = 0 at every sample. Two load-bearing properties:
// (a) Nonzero acceleration makes metric SCALE observable at all: under
//     constant velocity a perfect IMU reads zero specific force, which is
//     consistent with ANY constant velocity, and monocular vision alone is
//     scale-free -- the initial-velocity error the tests below inject would
//     be completely unobservable without it.
// (b) An injected velocity error is mostly -- not entirely -- absorbable by
//     a re-triangulating smart factor, regardless of its direction: given
//     only a short window, the factor can push a track's own best-fit depth
//     to explain away most of the resulting apparent drift. An ALONG-axis
//     error (a pure boresight speed error) is nearly indistinguishable this
//     way (measured in this package's own test development: ~0.0008 of
//     whitened visual residual "savings" vs. a 0.045 velocity-prior cost to
//     fully correct it at edge-prior sigma 1.0 -- not a side the factor
//     picks on its own). A LATERAL error is only PARTLY absorbable the same
//     way: pushing the 15 m wall's best fit out to ~21.5 m explains away
//     most, but not all, of it, leaving a small residual (~0.13 sigma on the
//     worst-placed view). What lets the visual factors correct the error
//     below is not that this residual is unabsorbable -- it mostly is --
//     but that the velocity edge-prior is loose enough (see
//     set_initial_edge_noises() in visual_odometry_window.cpp) to let that
//     small surviving signal win. The y-only error below is chosen so that
//     residual exists at all; the retained forward (x) component of the
//     motion still supplies the parallax the triangulation-quality checks
//     (see test 3) rely on.
Eigen::Isometry3d gt_pose(std::int64_t stamp_ns)  // T_world_imu at stamp
{
  const double t = 1e-9 * static_cast<double>(stamp_ns);
  const double s = t + 0.25 * t * t;
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(s, s, 0.0);
  return T;
}

void feed_imu(slam::vio::WindowSolver & solver, std::int64_t from_ns, std::int64_t to_ns)
{
  const std::int64_t step = static_cast<std::int64_t>(1e9 / kImuRate);
  for (std::int64_t t = from_ns; t <= to_ns; t += step) {
    solver.insert_imu(
      1e-9 * static_cast<double>(t), Eigen::Vector3d(0.5, 0.5, 9.81), Eigen::Vector3d::Zero());
  }
}

// StaggeredSecondCameraFoldsCorrectly-only variant of the synthetic world: the
// same diagonal translation, PLUS a constant yaw rate of 0.3 rad/s, so the
// per-observation Delta a correct fold computes carries a real ROTATION
// (unlike the translation-only world above, whose Delta a wrong,
// unfolded implementation can still often survive by re-triangulating a
// shifted point -- see the corrected story on gt_pose above). A rotation is
// not absorbable by any landmark placement at any depth, which is the whole
// point of this variant.
Eigen::Isometry3d gt_pose_yawing(std::int64_t stamp_ns)
{
  const double t = 1e-9 * static_cast<double>(stamp_ns);
  const double s = t + 0.25 * t * t;
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(s, s, 0.0);
  T.linear() = Eigen::AngleAxisd(0.3 * t, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return T;
}

// Specific force a perfect accelerometer reads (BODY frame) for
// gt_pose_yawing's trajectory: world-frame acceleration is the same constant
// (0.5, 0.5, 0) as the non-yawing world (translation dynamics are unchanged),
// so the specific force is R(t)^T * (0.5, 0.5, 9.81) -- the world-frame
// acceleration minus gravity, rotated into the (now yawing) body frame. Yaw
// is about world/body Z, the same axis as gravity, so the Z component is
// untouched by the rotation; only x/y mix.
Eigen::Vector3d acc_body_yawing(double t)
{
  const Eigen::Vector3d specific_force_world(0.5, 0.5, 9.81);
  const Eigen::Matrix3d R = Eigen::AngleAxisd(0.3 * t, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return R.transpose() * specific_force_world;
}

void feed_imu_yawing(slam::vio::WindowSolver & solver, std::int64_t from_ns, std::int64_t to_ns)
{
  const std::int64_t step = static_cast<std::int64_t>(1e9 / kImuRate);
  for (std::int64_t t_ns = from_ns; t_ns <= to_ns; t_ns += step) {
    const double t = 1e-9 * static_cast<double>(t_ns);
    solver.insert_imu(t, acc_body_yawing(t), Eigen::Vector3d(0.0, 0.0, 0.3));
  }
}

using PoseFn = Eigen::Isometry3d (*)(std::int64_t);

// One group: for each (camera_id, stamp offset) render every visible wall
// landmark through that camera at the true pose; track_id = landmark index.
// rgb varies with the anchor (r = anchor_ns / kPeriod) so a test can pin down
// WHICH keyframe's own observation a triangulated landmark's rgb came from,
// rather than every group emitting the same constant color regardless of
// source. pose_fn defaults to the translation-only gt_pose; pass
// gt_pose_yawing for StaggeredSecondCameraFoldsCorrectly.
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

void expect_translation_near(
  const Eigen::Isometry3d & actual, const Eigen::Isometry3d & expected, double tol)
{
  const double error = (actual.translation() - expected.translation()).norm();
  EXPECT_LT(error, tol) << "actual=" << actual.translation().transpose()
                        << " expected=" << expected.translation().transpose();
}

}  // namespace

TEST(VisualOdometryWindowTest, VisualFactorsCorrectAnInitialVelocityError)
{
  const std::vector<Eigen::Isometry3d> t_imu_cams = {optical_extrinsic()};
  const auto landmarks = wall_landmarks();

  slam::vio::WindowSolver solver(
    slam::vio::WindowConfig{.t_imu_cams = t_imu_cams, .max_keyframes = 10});
  // GT initial velocity is (1, 1, 0); start the filter 0.3 m/s too fast in y
  // -- LATERAL to the camera's +x boresight, so the error is observable (see
  // the synthetic-world comment on gt_pose above for why an along-axis error
  // would not be).
  solver.initialize(
    0, Eigen::Isometry3d::Identity(), Eigen::Vector3d(1.0, 1.3, 0.0),
    Eigen::Matrix<double, 6, 1>::Zero());

  // Covers both inter-keyframe gaps (0->100ms, 100->200ms) and the trailing
  // fold window the 200ms keyframe's own visual factors need ([200,300]ms).
  feed_imu(solver, 0, 3 * kPeriod);

  for (const std::int64_t anchor : {std::int64_t{0}, kPeriod, 2 * kPeriod}) {
    static_cast<void>(solver.push_keyframe(make_group(anchor, {{0, 0}}, t_imu_cams, landmarks)));
  }

  const auto snapshot = solver.window_snapshot();
  ASSERT_EQ(snapshot.size(), 3u);

  expect_translation_near(snapshot.back().T_world_imu, gt_pose(2 * kPeriod), 0.03);

  // If uncorrected, propagating the wrong v0 = 1.3 m/s (vs GT 1.0 m/s) alone
  // over the 0.2 s to the newest keyframe would drift the position by
  // (1.3 - 1.0) * 0.2 = 0.06 m -- twice the 0.03 m tolerance just checked --
  // so passing it requires the visual factors to have actually pulled the
  // velocity back toward GT, which the check below on the oldest keyframe
  // (anchored at the initialized state) verifies directly.
  constexpr double kUncorrectedDriftIfIgnored = 0.3 * 0.2;
  ASSERT_GT(kUncorrectedDriftIfIgnored, 0.03);

  const Eigen::Vector3d & oldest_velocity = snapshot.front().v_world_imu;
  EXPECT_NEAR(oldest_velocity.x(), 1.0, 0.05);
  EXPECT_NEAR(oldest_velocity.y(), 1.0, 0.05);
  EXPECT_NEAR(oldest_velocity.z(), 0.0, 0.05);
}

TEST(VisualOdometryWindowTest, StaggeredSecondCameraFoldsCorrectly)
{
  Eigen::Isometry3d cam1 = optical_extrinsic();
  cam1.translation() += Eigen::Vector3d(0.0, 0.4, 0.0);  // +0.4 m along body y
  const std::vector<Eigen::Isometry3d> t_imu_cams = {optical_extrinsic(), cam1};
  const auto landmarks = wall_landmarks();

  slam::vio::WindowSolver solver(
    slam::vio::WindowConfig{.t_imu_cams = t_imu_cams, .max_keyframes = 10});
  // Same lateral (y) velocity error as VisualFactorsCorrectAnInitialVelocityError.
  solver.initialize(
    0, Eigen::Isometry3d::Identity(), Eigen::Vector3d(1.0, 1.3, 0.0),
    Eigen::Matrix<double, 6, 1>::Zero());
  // The yawing world: a translation-only world's Delta-mislocation is mostly
  // absorbable by re-triangulating the landmark at a shifted depth (see the
  // corrected story on gt_pose above), so it does not actually discriminate
  // an unfolded implementation. Verified by forcing Delta = Identity in
  // add_visual_factors(): this test still PASSED against the
  // translation-only world. The yaw rate makes Delta carry a rotation
  // instead, which no landmark placement at any depth can explain away.
  feed_imu_yawing(solver, 0, 3 * kPeriod);

  constexpr std::int64_t kStagger = 40'000'000;  // camera 1 triggers 40 ms into each group
  for (const std::int64_t anchor : {std::int64_t{0}, kPeriod, 2 * kPeriod}) {
    static_cast<void>(solver.push_keyframe(
      make_group(anchor, {{0, 0}, {1, kStagger}}, t_imu_cams, landmarks, gt_pose_yawing)));
  }

  const auto snapshot = solver.window_snapshot();
  ASSERT_EQ(snapshot.size(), 3u);

  expect_translation_near(snapshot.back().T_world_imu, gt_pose_yawing(2 * kPeriod), 0.03);

  const Eigen::Vector3d & oldest_velocity = snapshot.front().v_world_imu;
  EXPECT_NEAR(oldest_velocity.x(), 1.0, 0.05);
  EXPECT_NEAR(oldest_velocity.y(), 1.0, 0.05);
  EXPECT_NEAR(oldest_velocity.z(), 0.0, 0.05);

  // A rotation of 0.3 rad/s * 0.04 s = 12 mrad accrues between the keyframe
  // anchor and camera 1's +40 ms stagger. An unfolded implementation
  // (Delta = identity) omits this rotation entirely, mislocating every
  // camera-1 observation's bearing by ~12 mrad -- at obs_sigma = 1e-3 that is
  // a ~12-sigma systematic bias, and unlike a pure translation mislocation,
  // no landmark placement at any depth can explain away a missing rotation,
  // which is why an unfolded implementation drags the solution off the
  // tolerances checked above.
  constexpr double kUnfoldedRotationBias = 0.3 * 0.04;
  ASSERT_NEAR(kUnfoldedRotationBias, 12.0e-3, 1e-4);
  ASSERT_GT(kUnfoldedRotationBias / 1.0e-3, 10.0);  // sigmas, at obs_sigma = 1e-3
}

TEST(VisualOdometryWindowTest, MarginalizationEmitsKeyframesWithLandmarks)
{
  const std::vector<Eigen::Isometry3d> t_imu_cams = {optical_extrinsic()};
  const auto landmarks = wall_landmarks();

  slam::vio::WindowSolver solver(
    slam::vio::WindowConfig{.t_imu_cams = t_imu_cams, .max_keyframes = 3});
  // No injected error here: initialized exactly at GT (1, 1, 0).
  solver.initialize(
    0, Eigen::Isometry3d::Identity(), Eigen::Vector3d(1.0, 1.0, 0.0),
    Eigen::Matrix<double, 6, 1>::Zero());
  feed_imu(solver, 0, 5 * kPeriod);

  std::vector<slam::vio::MarginalizedKeyframe> marginalized;
  for (std::int64_t i = 0; i < 5; ++i) {
    auto popped = solver.push_keyframe(make_group(i * kPeriod, {{0, 0}}, t_imu_cams, landmarks));
    marginalized.insert(
      marginalized.end(), std::make_move_iterator(popped.begin()),
      std::make_move_iterator(popped.end()));
  }

  ASSERT_EQ(marginalized.size(), 2u);
  EXPECT_EQ(marginalized[0].stamp_ns, 0);
  EXPECT_EQ(marginalized[1].stamp_ns, kPeriod);

  for (const auto & kf : marginalized) {
    ASSERT_EQ(kf.landmarks_world.size(), kf.landmark_rgb.size());
    EXPECT_GE(kf.landmarks_world.size(), 10u);
    // rgb is keyed to the OBSERVING keyframe's own anchor (see make_group),
    // not a fixed constant every group happens to share -- this is what
    // actually pins down that landmark_rgb comes from THIS keyframe's own
    // observation of the track, not e.g. an arbitrary track-mate's.
    const auto expected_rgb =
      std::array<std::uint8_t, 3>{static_cast<std::uint8_t>(kf.stamp_ns / kPeriod), 20, 30};
    for (std::size_t i = 0; i < kf.landmarks_world.size(); ++i) {
      double nearest = std::numeric_limits<double>::max();
      for (const Eigen::Vector3d & landmark : landmarks) {
        nearest = std::min(nearest, (kf.landmarks_world[i] - landmark).norm());
      }
      EXPECT_LT(nearest, 0.15) << "landmark=" << kf.landmarks_world[i].transpose();
      EXPECT_EQ(kf.landmark_rgb[i], expected_rgb);
    }
  }
}

TEST(VisualOdometryWindowTest, EmptyGroupsRideTheImuChain)
{
  const std::vector<Eigen::Isometry3d> t_imu_cams = {optical_extrinsic()};
  const auto landmarks = wall_landmarks();

  slam::vio::WindowSolver solver(
    slam::vio::WindowConfig{.t_imu_cams = t_imu_cams, .max_keyframes = 10});
  // Same lateral (y) velocity error as VisualFactorsCorrectAnInitialVelocityError:
  // with the empty group at index 1, every track's surviving observations
  // span keyframes 0, 2, 3 -- exactly min_track_obs = 3, right at the
  // qualifying threshold -- so this exercises the visual layer actually
  // doing correction work across the gap, not merely an exact-GT init that
  // would pass regardless of whether any visual factor fired at all.
  solver.initialize(
    0, Eigen::Isometry3d::Identity(), Eigen::Vector3d(1.0, 1.3, 0.0),
    Eigen::Matrix<double, 6, 1>::Zero());
  feed_imu(solver, 0, 4 * kPeriod);

  ASSERT_NO_THROW({
    static_cast<void>(solver.push_keyframe(make_group(0, {{0, 0}}, t_imu_cams, landmarks)));

    slam::ObservationGroup empty_group;
    empty_group.anchor_stamp_ns = kPeriod;
    // No observations: this keyframe contributes nothing visually; the IMU
    // chain alone carries it, and every track's min_track_obs = 3 comes
    // entirely from the other three keyframes.
    static_cast<void>(solver.push_keyframe(empty_group));

    static_cast<void>(
      solver.push_keyframe(make_group(2 * kPeriod, {{0, 0}}, t_imu_cams, landmarks)));
    static_cast<void>(
      solver.push_keyframe(make_group(3 * kPeriod, {{0, 0}}, t_imu_cams, landmarks)));
  });

  const auto snapshot = solver.window_snapshot();
  ASSERT_EQ(snapshot.size(), 4u);

  expect_translation_near(snapshot.back().T_world_imu, gt_pose(3 * kPeriod), 0.03);

  const Eigen::Vector3d & oldest_velocity = snapshot.front().v_world_imu;
  EXPECT_NEAR(oldest_velocity.x(), 1.0, 0.05);
  EXPECT_NEAR(oldest_velocity.y(), 1.0, 0.05);
  EXPECT_NEAR(oldest_velocity.z(), 0.0, 0.05);
}

// Robustness smoke test for the "empty group + too-few-IMU-samples gap"
// scenario: X(cur) would otherwise have zero rows/columns in the linear
// system (see solve()'s comment on the pose BetweenFactor fallback).
// Verified empirically: forcing the exact same scenario with that fallback
// temporarily removed did NOT reproduce IndeterminantLinearSystemException,
// even with 3 preceding real keyframes
// forcing a genuinely nonzero-residual solve (gtsam's default
// LevenbergMarquardtParams uses additive, not diagonal-scaled, damping,
// which happens to tolerate a fully disconnected variable gracefully). This
// test therefore cannot regress-guard the fallback's presence specifically
// -- it passes either way -- but it still pins down the documented
// contract (no throw, finite states) for this input shape.
TEST(VisualOdometryWindowTest, EmptyGroupWithoutImuDataDoesNotThrow)
{
  const std::vector<Eigen::Isometry3d> t_imu_cams = {optical_extrinsic()};
  const auto landmarks = wall_landmarks();

  slam::vio::WindowSolver solver(
    slam::vio::WindowConfig{.t_imu_cams = t_imu_cams, .max_keyframes = 10});
  // No injected error here: initialized exactly at GT (1, 1, 0).
  solver.initialize(
    0, Eigen::Isometry3d::Identity(), Eigen::Vector3d(1.0, 1.0, 0.0),
    Eigen::Matrix<double, 6, 1>::Zero());
  feed_imu(solver, 0, 3 * kPeriod);  // covers all 3 real keyframes' gaps and fold windows

  static_cast<void>(solver.push_keyframe(make_group(0, {{0, 0}}, t_imu_cams, landmarks)));
  static_cast<void>(solver.push_keyframe(make_group(kPeriod, {{0, 0}}, t_imu_cams, landmarks)));
  static_cast<void>(solver.push_keyframe(make_group(2 * kPeriod, {{0, 0}}, t_imu_cams, landmarks)));

  // No feed_imu() beyond 3*kPeriod: this 4th push predicts a new keyframe
  // with zero visual observations AND fewer than 2 IMU samples in its own
  // gap.
  slam::ObservationGroup empty_group;
  empty_group.anchor_stamp_ns = 3 * kPeriod;

  std::vector<slam::vio::MarginalizedKeyframe> popped;
  ASSERT_NO_THROW({ popped = solver.push_keyframe(empty_group); });
  EXPECT_TRUE(popped.empty());

  const auto snapshot = solver.window_snapshot();
  ASSERT_EQ(snapshot.size(), 4u);
  for (const auto & kf : snapshot) {
    EXPECT_TRUE(kf.T_world_imu.translation().allFinite());
    EXPECT_TRUE(kf.v_world_imu.allFinite());
    EXPECT_TRUE(kf.imu_bias.allFinite());
  }
}

TEST(VisualOdometryWindowTest, InvalidConfigThrows)
{
  const std::vector<Eigen::Isometry3d> t_imu_cams = {optical_extrinsic()};

  EXPECT_THROW(
    slam::vio::WindowSolver(slam::vio::WindowConfig{.t_imu_cams = t_imu_cams, .max_keyframes = 0}),
    std::invalid_argument);
  EXPECT_THROW(
    slam::vio::WindowSolver(slam::vio::WindowConfig{.t_imu_cams = t_imu_cams, .min_track_obs = 0}),
    std::invalid_argument);
  EXPECT_THROW(
    slam::vio::WindowSolver(slam::vio::WindowConfig{.t_imu_cams = t_imu_cams, .window_span_ns = 0}),
    std::invalid_argument);
  EXPECT_THROW(
    slam::vio::WindowSolver(
      slam::vio::WindowConfig{.t_imu_cams = t_imu_cams, .window_span_ns = -1}),
    std::invalid_argument);
}

TEST(VisualOdometryWindowTest, PredictFollowsImuBetweenKeyframes)
{
  const std::vector<Eigen::Isometry3d> t_imu_cams = {optical_extrinsic()};
  const auto landmarks = wall_landmarks();

  slam::vio::WindowSolver solver(
    slam::vio::WindowConfig{.t_imu_cams = t_imu_cams, .max_keyframes = 10});
  // No injected error here: initialized exactly at GT (1, 1, 0).
  solver.initialize(
    0, Eigen::Isometry3d::Identity(), Eigen::Vector3d(1.0, 1.0, 0.0),
    Eigen::Matrix<double, 6, 1>::Zero());
  feed_imu(solver, 0, 3 * kPeriod);  // covers the predict query 50 ms past the newest keyframe too

  static_cast<void>(solver.push_keyframe(make_group(0, {{0, 0}}, t_imu_cams, landmarks)));
  static_cast<void>(solver.push_keyframe(make_group(kPeriod, {{0, 0}}, t_imu_cams, landmarks)));

  const std::int64_t query_ns = kPeriod + 50'000'000;
  const auto predicted = solver.predict_T_world_imu(query_ns);
  ASSERT_TRUE(predicted.has_value());
  expect_translation_near(*predicted, gt_pose(query_ns), 0.01);
}
