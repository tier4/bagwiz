// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "visual_factors.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <gtest/gtest.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <set>
#include <vector>

namespace
{
namespace visual = bagwiz::core::slam::visual;
namespace slam = bagwiz::core::slam;

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

// ---------------------------------------------------------------------------
// Two-submap co-visibility scene shared by the factor-construction tests.
//
// A platform drives straight along +x at 1 m/s past a wall at x = 15 with one
// forward-looking camera. Submap A's origin is the world origin and covers
// t in {0, 0.5, 1}; submap B's origin sits at (2, 0, 0) yawed 5 deg and covers
// t in {2, 2.5, 3}. Every landmark is therefore seen from both submaps, which
// is exactly the configuration a co-visibility factor is supposed to tie.
// ---------------------------------------------------------------------------

constexpr std::uint64_t kSubmapIdA = 5;  // deliberately not the view index, so
constexpr std::uint64_t kSubmapIdB = 6;  // the tests pin keys to submap ids
constexpr double kWallX = 15.0;

// T_lidar_cam for a forward-looking optical frame (z forward, x right, y down)
// mounted on a body frame with x forward, y left, z up: body x = optical z,
// body y = -optical x, body z = -optical y.
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
// dead ahead has no parallax under this purely forward motion).
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

// The LiDAR view of the same wall: a 0.5 m grid over y in [-5, 5], z in [0, 4].
std::vector<Eigen::Vector3d> wall_cloud_points()
{
  std::vector<Eigen::Vector3d> points;
  for (int iy = -10; iy <= 10; ++iy) {
    for (int iz = 0; iz <= 8; ++iz) {
      points.emplace_back(kWallX, 0.5 * iy, 0.5 * iz);
    }
  }
  return points;
}

struct Scene
{
  std::vector<visual::SubmapView> views;
  std::vector<slam::VisualObservation> observations;
  std::vector<Eigen::Isometry3d> t_lidar_cams;
  // Owns what SubmapView::cloud points at.
  std::vector<gtsam_points::PointCloudCPU::Ptr> clouds;
};

gtsam_points::PointCloudCPU::Ptr make_cloud(
  const std::vector<Eigen::Vector3d> & world_points, const Eigen::Isometry3d & T_world_origin)
{
  std::vector<Eigen::Vector4d> local;
  local.reserve(world_points.size());
  const Eigen::Isometry3d T_origin_world = T_world_origin.inverse();
  for (const Eigen::Vector3d & p_world : world_points) {
    const Eigen::Vector3d p_origin = T_origin_world * p_world;
    local.emplace_back(p_origin.x(), p_origin.y(), p_origin.z(), 1.0);
  }
  return std::make_shared<gtsam_points::PointCloudCPU>(local);
}

// Project one camera's landmarks through every frame of both submaps. Track ids
// restart at 0 for each camera, exactly as separate VisualFrontend instances
// number them. `scene.t_lidar_cams[camera_id]` must already exist.
void emit_observations(
  Scene & scene, std::int32_t camera_id, const std::vector<Eigen::Vector3d> & landmarks_a,
  const std::vector<Eigen::Vector3d> & landmarks_b)
{
  const std::vector<const std::vector<Eigen::Vector3d> *> landmarks = {&landmarks_a, &landmarks_b};
  for (std::size_t vi = 0; vi < scene.views.size(); ++vi) {
    const visual::SubmapView & view = scene.views[vi];
    for (std::size_t fi = 0; fi < view.frame_stamps.size(); ++fi) {
      const Eigen::Isometry3d T_world_cam = view.T_world_origin * view.T_origin_frames[fi] *
                                            scene.t_lidar_cams[static_cast<std::size_t>(camera_id)];
      for (std::size_t k = 0; k < landmarks[vi]->size(); ++k) {
        const auto measurement = project(T_world_cam, (*landmarks[vi])[k]);
        if (!measurement.has_value()) {
          continue;
        }
        slam::VisualObservation obs;
        obs.camera_id = camera_id;
        obs.track_id = k;
        obs.stamp_ns = static_cast<std::int64_t>(std::llround(view.frame_stamps[fi] * 1e9));
        obs.x = measurement->x();
        obs.y = measurement->y();
        scene.observations.push_back(obs);
      }
    }
  }
}

// `landmarks_a` / `landmarks_b` are the same landmark ids seen from submap A and
// from submap B; passing different positions for one id makes that track behave
// like a moving object.
Scene make_scene(
  const std::vector<Eigen::Vector3d> & landmarks_a,
  const std::vector<Eigen::Vector3d> & landmarks_b, bool with_clouds)
{
  Scene scene;
  scene.t_lidar_cams = {optical_extrinsic()};

  visual::SubmapView view_a;
  view_a.id = kSubmapIdA;
  view_a.T_world_origin = Eigen::Isometry3d::Identity();
  view_a.frame_stamps = {0.0, 0.5, 1.0};

  visual::SubmapView view_b;
  view_b.id = kSubmapIdB;
  view_b.T_world_origin = make_pose(2.0, 0.0, 0.0, 5.0 * M_PI / 180.0);
  view_b.frame_stamps = {2.0, 2.5, 3.0};

  // Constant 1 m/s along +x, so a frame's world x equals its stamp.
  for (visual::SubmapView * view : {&view_a, &view_b}) {
    for (const double t : view->frame_stamps) {
      view->T_origin_frames.push_back(view->T_world_origin.inverse() * make_pose(t, 0.0, 0.0, 0.0));
    }
  }
  scene.views = {view_a, view_b};

  emit_observations(scene, 0, landmarks_a, landmarks_b);

  if (with_clouds) {
    const auto wall = wall_cloud_points();
    for (visual::SubmapView & view : scene.views) {
      scene.clouds.push_back(make_cloud(wall, view.T_world_origin));
      view.cloud = scene.clouds.back().get();
    }
  }
  return scene;
}

// Bolt a second camera onto a scene: its own extrinsic, its own physical
// landmarks, and track ids restarting at 0 - which is what makes it collide
// with camera 0's ids.
void add_camera(
  Scene & scene, const Eigen::Isometry3d & t_lidar_cam,
  const std::vector<Eigen::Vector3d> & landmarks)
{
  scene.t_lidar_cams.push_back(t_lidar_cam);
  emit_observations(
    scene, static_cast<std::int32_t>(scene.t_lidar_cams.size() - 1), landmarks, landmarks);
}

visual::Stats build(
  const Scene & scene, const visual::Params & params,
  std::vector<gtsam::NonlinearFactor::shared_ptr> & factors)
{
  return visual::build_visual_factors(
    scene.observations, scene.t_lidar_cams, scene.views, params, factors);
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

// Guards the scene's optical convention: get T_lidar_cam backwards and every
// factor test below would still be self-consistent but geometrically wrong.
TEST(VisualFactorsTest, SceneCameraLooksAlongBodyX)
{
  const Eigen::Isometry3d T_world_cam = optical_extrinsic();

  const auto ahead = project(T_world_cam, Eigen::Vector3d(10.0, 0.0, 0.0));
  ASSERT_TRUE(ahead.has_value());
  EXPECT_NEAR(ahead->x(), 0.0, 1e-12);
  EXPECT_NEAR(ahead->y(), 0.0, 1e-12);

  // Body +y is to the left, so it lands left of the principal point (u < 0),
  // and body +z is up, so it lands above it (v < 0, image y grows downward).
  const auto left = project(T_world_cam, Eigen::Vector3d(10.0, 1.0, 0.0));
  ASSERT_TRUE(left.has_value());
  EXPECT_NEAR(left->x(), -0.1, 1e-12);
  const auto up = project(T_world_cam, Eigen::Vector3d(10.0, 0.0, 1.0));
  ASSERT_TRUE(up.has_value());
  EXPECT_NEAR(up->y(), -0.1, 1e-12);

  EXPECT_FALSE(project(T_world_cam, Eigen::Vector3d(-10.0, 0.0, 0.0)).has_value());
}

TEST(VisualFactorsTest, PerfectTracksYieldOneFactorPerLandmark)
{
  const Scene scene = make_scene(wall_landmarks(), wall_landmarks(), true);
  const visual::Params params;

  std::vector<gtsam::NonlinearFactor::shared_ptr> factors;
  const visual::Stats stats = build(scene, params, factors);

  EXPECT_EQ(stats.tracks_total, 20u);
  EXPECT_EQ(stats.factors, 20u);
  EXPECT_EQ(factors.size(), 20u);
  EXPECT_EQ(stats.tracks_single_submap, 0u);
  EXPECT_EQ(stats.tracks_too_short, 0u);
  EXPECT_EQ(stats.tracks_triangulation_failed, 0u);
  EXPECT_EQ(stats.tracks_gated, 0u);

  using gtsam::symbol_shorthand::X;
  const std::set<gtsam::Key> expected_keys = {X(kSubmapIdA), X(kSubmapIdB)};
  for (const auto & factor : factors) {
    ASSERT_NE(factor, nullptr);
    const std::set<gtsam::Key> keys(factor->keys().begin(), factor->keys().end());
    EXPECT_EQ(keys, expected_keys);
    // Six observations collapse onto the two submap keys.
    EXPECT_EQ(factor->keys().size(), 2u);
  }
}

TEST(VisualFactorsTest, SingleSubmapTracksAreDropped)
{
  const Scene scene = make_scene(wall_landmarks(), wall_landmarks(), true);
  std::vector<slam::VisualObservation> only_a;
  for (const slam::VisualObservation & obs : scene.observations) {
    if (obs.stamp_ns <= 1'000'000'000) {
      only_a.push_back(obs);
    }
  }
  ASSERT_EQ(only_a.size(), 60u);  // 20 landmarks x 3 frames of submap A

  const visual::Params params;
  std::vector<gtsam::NonlinearFactor::shared_ptr> factors;
  const visual::Stats stats =
    visual::build_visual_factors(only_a, scene.t_lidar_cams, scene.views, params, factors);

  EXPECT_EQ(stats.tracks_total, 20u);
  EXPECT_EQ(stats.factors, 0u);
  EXPECT_TRUE(factors.empty());
  EXPECT_EQ(stats.tracks_single_submap, 20u);
  EXPECT_EQ(stats.tracks_too_short, 0u);
  EXPECT_EQ(stats.tracks_triangulation_failed, 0u);
  EXPECT_EQ(stats.tracks_gated, 0u);
}

TEST(VisualFactorsTest, MovingLandmarkIsRejected)
{
  auto landmarks_b = wall_landmarks();
  landmarks_b.front().y() += 0.5;  // the tracked object slid sideways
  const Scene scene = make_scene(wall_landmarks(), landmarks_b, true);
  const visual::Params params;

  std::vector<gtsam::NonlinearFactor::shared_ptr> factors;
  const visual::Stats stats = build(scene, params, factors);

  EXPECT_EQ(stats.tracks_total, 20u);
  EXPECT_EQ(stats.factors, 19u);
  EXPECT_EQ(factors.size(), 19u);
  EXPECT_EQ(stats.tracks_triangulation_failed, 1u);
  EXPECT_EQ(stats.tracks_single_submap, 0u);
  EXPECT_EQ(stats.tracks_too_short, 0u);
  EXPECT_EQ(stats.tracks_gated, 0u);
}

TEST(VisualFactorsTest, LidarGateDropsUnsupportedLandmark)
{
  // Static and perfectly triangulable, but 5 m clear of the wall the LiDAR
  // clouds cover, so nothing in either submap cloud supports it.
  auto landmarks = wall_landmarks();
  landmarks.front() = Eigen::Vector3d(kWallX, 10.0, 2.0);
  const Scene scene = make_scene(landmarks, landmarks, true);

  visual::Params params;
  std::vector<gtsam::NonlinearFactor::shared_ptr> gated_factors;
  const visual::Stats gated = build(scene, params, gated_factors);

  EXPECT_EQ(gated.tracks_total, 20u);
  EXPECT_EQ(gated.factors, 19u);
  EXPECT_EQ(gated_factors.size(), 19u);
  EXPECT_EQ(gated.tracks_gated, 1u);
  EXPECT_EQ(gated.tracks_triangulation_failed, 0u);

  params.gate_distance = 0.0;
  std::vector<gtsam::NonlinearFactor::shared_ptr> ungated_factors;
  const visual::Stats ungated = build(scene, params, ungated_factors);

  EXPECT_EQ(ungated.factors, 20u);
  EXPECT_EQ(ungated_factors.size(), 20u);
  EXPECT_EQ(ungated.tracks_gated, 0u);
}

// track_id is unique only per camera, so two cameras both numbering their tracks
// from 0 must stay separate. Grouping on track_id alone fuses each pair into one
// 12-observation "track" spanning two physical points 0.7 m apart, which then
// triangulates as an outlier - so the bug shows up as 20 tracks, 0 factors.
TEST(VisualFactorsTest, TwoCamerasWithSameTrackIdsDoNotFuse)
{
  Scene scene = make_scene(wall_landmarks(), wall_landmarks(), true);

  // Same forward-looking optical frame, mounted 0.3 m to the right, watching a
  // different set of 20 wall points.
  Eigen::Isometry3d second_mount = optical_extrinsic();
  second_mount.translation() = Eigen::Vector3d(0.0, -0.3, 0.0);
  auto second_landmarks = wall_landmarks();
  for (Eigen::Vector3d & landmark : second_landmarks) {
    landmark.z() += 0.7;
  }
  add_camera(scene, second_mount, second_landmarks);

  ASSERT_EQ(scene.t_lidar_cams.size(), 2u);
  ASSERT_EQ(scene.observations.size(), 240u);  // 2 cameras x 20 landmarks x 6 frames

  const visual::Params params;
  std::vector<gtsam::NonlinearFactor::shared_ptr> factors;
  const visual::Stats stats = build(scene, params, factors);

  EXPECT_EQ(stats.tracks_total, 40u);
  EXPECT_EQ(stats.factors, 40u);
  EXPECT_EQ(factors.size(), 40u);
  EXPECT_EQ(stats.tracks_triangulation_failed, 0u);
  EXPECT_EQ(stats.tracks_single_submap, 0u);
  EXPECT_EQ(stats.tracks_too_short, 0u);
  EXPECT_EQ(stats.tracks_gated, 0u);
}

// An empty submap cloud supports nothing anywhere, so treating it as a usable
// cloud would reject every track in that submap. The gate must abstain instead.
TEST(VisualFactorsTest, GateAbstainsWhenSubmapCloudIsEmpty)
{
  Scene scene = make_scene(wall_landmarks(), wall_landmarks(), false);
  for (visual::SubmapView & view : scene.views) {
    scene.clouds.push_back(std::make_shared<gtsam_points::PointCloudCPU>());
    view.cloud = scene.clouds.back().get();
  }
  ASSERT_EQ(scene.views.front().cloud->size(), 0u);

  const visual::Params params;
  ASSERT_GT(params.gate_distance, 0.0);
  std::vector<gtsam::NonlinearFactor::shared_ptr> factors;
  const visual::Stats stats = build(scene, params, factors);

  EXPECT_EQ(stats.factors, 20u);
  EXPECT_EQ(stats.tracks_gated, 0u);
}

// A GPU-resident cloud reports num_points > 0 while its host `points` array is
// null. The gate has no host geometry to read, so it must abstain rather than
// dereference the null pointer.
TEST(VisualFactorsTest, GateAbstainsWhenCloudHasNoHostPoints)
{
  Scene scene = make_scene(wall_landmarks(), wall_landmarks(), false);
  for (visual::SubmapView & view : scene.views) {
    auto cloud = std::make_shared<gtsam_points::PointCloudCPU>();
    cloud->num_points = 189;  // as if the points lived only on the GPU
    scene.clouds.push_back(cloud);
    view.cloud = scene.clouds.back().get();
  }
  ASSERT_GT(scene.views.front().cloud->size(), 0u);
  ASSERT_EQ(scene.views.front().cloud->points, nullptr);

  const visual::Params params;
  ASSERT_GT(params.gate_distance, 0.0);
  std::vector<gtsam::NonlinearFactor::shared_ptr> factors;
  const visual::Stats stats = build(scene, params, factors);

  EXPECT_EQ(stats.factors, 20u);
  EXPECT_EQ(stats.tracks_gated, 0u);
}

TEST(VisualFactorsTest, FactorsImproveAPerturbedPose)
{
  const Scene scene = make_scene(wall_landmarks(), wall_landmarks(), true);
  const visual::Params params;
  std::vector<gtsam::NonlinearFactor::shared_ptr> factors;
  const visual::Stats stats = build(scene, params, factors);
  ASSERT_EQ(stats.factors, 20u);

  const gtsam::Pose3 truth_a(scene.views.front().T_world_origin.matrix());
  const gtsam::Pose3 truth_b(scene.views.back().T_world_origin.matrix());

  Eigen::Isometry3d drifted = scene.views.back().T_world_origin;
  drifted.translation() += Eigen::Vector3d(0.3, 0.2, 0.0);
  const Eigen::Matrix3d drifted_rotation =
    Eigen::AngleAxisd(2.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
    drifted.linear();
  drifted.linear() = drifted_rotation;
  const gtsam::Pose3 perturbed_b(drifted.matrix());

  const double initial_translation_error =
    (perturbed_b.translation() - truth_b.translation()).norm();
  ASSERT_NEAR(initial_translation_error, std::sqrt(0.13), 1e-9);

  using gtsam::symbol_shorthand::X;
  gtsam::NonlinearFactorGraph graph;
  graph.addPrior(X(kSubmapIdA), truth_a, gtsam::noiseModel::Isotropic::Sigma(6, 1.0e-6));
  graph.addPrior(X(kSubmapIdB), perturbed_b, gtsam::noiseModel::Isotropic::Sigma(6, 1.0));
  for (const auto & factor : factors) {
    graph.add(factor);
  }

  gtsam::Values initial;
  initial.insert(X(kSubmapIdA), truth_a);
  initial.insert(X(kSubmapIdB), perturbed_b);

  const gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
  const gtsam::Pose3 estimated_b = result.at<gtsam::Pose3>(X(kSubmapIdB));

  const double translation_error = (estimated_b.translation() - truth_b.translation()).norm();
  const double rotation_error = std::abs(
    Eigen::AngleAxisd(
      Eigen::Quaterniond(estimated_b.rotation().matrix()).inverse() *
      Eigen::Quaterniond(truth_b.rotation().matrix()))
      .angle());
  EXPECT_LT(translation_error, 0.05) << "initial was " << initial_translation_error;
  EXPECT_LT(rotation_error, 0.5 * M_PI / 180.0);
}

// The camera-only sparse-map export: one landmark per qualifying track,
// re-triangulated at the submaps' poses. Track selection mirrors factor
// construction, so the same scene yields exactly the 20 wall points — in
// track-key order, which for this single-camera scene is track-id order, and
// track_id is the landmark index. Exact agreement is asserted because the
// input is noiseless: DLT triangulation of exact projections through exact
// poses is a per-element computation over immutable input (AGENTS.md
// "Numerical Reproducibility"), so any deviation is a bug, not drift.
TEST(VisualFactorsTest, TriangulateLandmarksRecoversWallPoints)
{
  const Scene scene = make_scene(wall_landmarks(), wall_landmarks(), false);
  const visual::Params params;

  const std::vector<visual::Landmark> landmarks =
    visual::triangulate_landmarks(scene.observations, scene.t_lidar_cams, scene.views, params);

  const std::vector<Eigen::Vector3d> truth = wall_landmarks();
  ASSERT_EQ(landmarks.size(), truth.size());
  for (std::size_t i = 0; i < landmarks.size(); ++i) {
    const Eigen::Vector3d p(landmarks[i].point[0], landmarks[i].point[1], landmarks[i].point[2]);
    EXPECT_LT((p - truth[i]).norm(), 1e-6) << "landmark " << i << " p=" << p.transpose();
  }
}

// Each landmark's color comes from its track's FIRST associated observation.
// Encoding the observation's stamp (seconds + 1) in the red channel makes the
// first observation red 1 (t=0) and the last red 4 (t=3), so any other rgb
// source fails this.
TEST(VisualFactorsTest, TriangulateLandmarksCarriesFirstObservationRgb)
{
  Scene scene = make_scene(wall_landmarks(), wall_landmarks(), false);
  for (slam::VisualObservation & obs : scene.observations) {
    obs.rgb = {static_cast<std::uint8_t>(obs.stamp_ns / 1'000'000'000 + 1), 20, 30};
  }
  const visual::Params params;

  const std::vector<visual::Landmark> landmarks =
    visual::triangulate_landmarks(scene.observations, scene.t_lidar_cams, scene.views, params);

  ASSERT_EQ(landmarks.size(), 20u);
  for (const visual::Landmark & landmark : landmarks) {
    EXPECT_EQ(landmark.rgb, (std::array<std::uint8_t, 3>{1, 20, 30}));
  }
}

// The export applies the same track selection as factor construction:
// single-submap tracks qualify for nothing, and a moved landmark fails
// triangulation — so neither becomes a map point.
TEST(VisualFactorsTest, TriangulateLandmarksDropsIneligibleTracks)
{
  const Scene full = make_scene(wall_landmarks(), wall_landmarks(), false);
  const visual::Params params;

  std::vector<slam::VisualObservation> only_a;
  for (const slam::VisualObservation & obs : full.observations) {
    if (obs.stamp_ns <= 1'000'000'000) {
      only_a.push_back(obs);
    }
  }
  ASSERT_EQ(only_a.size(), 60u);  // 20 landmarks x 3 frames of submap A
  EXPECT_TRUE(visual::triangulate_landmarks(only_a, full.t_lidar_cams, full.views, params).empty());

  auto moved = wall_landmarks();
  moved.front().y() += 0.5;  // the tracked object slid sideways
  const Scene scene = make_scene(wall_landmarks(), moved, false);
  EXPECT_EQ(
    visual::triangulate_landmarks(scene.observations, scene.t_lidar_cams, scene.views, params)
      .size(),
    19u);
}
