// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef CORE__SLAM__VISUAL_ODOMETRY_HPP_
#define CORE__SLAM__VISUAL_ODOMETRY_HPP_

#include "visual_odometry_grouping.hpp"  // NOLINT(build/include_subdir) src-local header
#include "visual_odometry_window.hpp"    // NOLINT(build/include_subdir) src-local header

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glim/odometry/estimation_frame.hpp>
#include <glim/odometry/odometry_estimation_base.hpp>

#include <cstdint>
#include <memory>
#include <span>
#include <vector>

// NaiveInitialStateEstimation is only held behind a unique_ptr here (see
// glim::NaiveInitialStateEstimation below); its full definition, and every
// gtsam_points type this class touches, stays in the .cpp so this header
// needs nothing beyond glim's odometry headers, Eigen, and std.
namespace glim
{
class NaiveInitialStateEstimation;
}  // namespace glim

namespace bagwiz::core::slam
{

// Configuration for the camera-only visual-inertial odometry estimator
// (issue #376 Phase 3). Mirrors the knobs of GroupingBuffer::Config and
// vio::WindowConfig that this class builds internally, plus its own
// keyframe-gating thresholds.
struct VisualInertialOdometryConfig
{
  std::vector<Eigen::Isometry3d> t_imu_cams;  // per camera_id: p_imu = T * p_cam
  Eigen::Isometry3d T_lidar_imu =
    Eigen::Isometry3d::Identity();  // := T_cam0_imu; makes T_world_lidar = T_world_cam0
  std::int32_t anchor_camera_id = 0;
  std::int64_t anchor_period_ns = 100'000'000;
  double keyframe_min_trans = 0.25;  // m, displacement gate
  double keyframe_min_rot = 0.17;    // rad (~10 deg)
  int max_window_keyframes = 10;
  double obs_sigma = 1.0e-3;
};

// glim::OdometryEstimationBase implementation for the camera-only mode of
// map slam: multi-camera VisualObservation batches are grouped into anchor
// windows (GroupingBuffer), gravity-aligned via glim's NaiveInitialStateEstimation,
// gated into keyframes by displacement, and estimated by the rebuilt-window
// batch smoother (vio::WindowSolver). Images never enter GLIM -- only the
// EstimationFrames this class produces (from keyframes leaving the window)
// cross into the stock SubMapping/GlobalMapping pipeline.
class VisualInertialOdometry : public glim::OdometryEstimationBase
{
public:
  explicit VisualInertialOdometry(VisualInertialOdometryConfig config);
  ~VisualInertialOdometry() override;

  [[nodiscard]] bool requires_imu() const override { return true; }
  void insert_imu(
    double stamp, const Eigen::Vector3d & linear_acc, const Eigen::Vector3d & angular_vel) override;

  // The camera-only analogue of insert_frame: consume one batch of frontend
  // observations, run grouping / keyframing / window solves, and append any
  // keyframes that left the window to marginalized_states (same append-only,
  // caller-owned contract as insert_frame). Returns the newest keyframe
  // estimate, or nullptr while uninitialized.
  glim::EstimationFrame::ConstPtr insert_visual_observations(
    std::span<const VisualObservation> observations,
    std::vector<glim::EstimationFrame::ConstPtr> & marginalized_states);

  // Flush grouping and hand out every keyframe still in the window (end of
  // sequence), oldest first -- the OdometryEstimationBase contract.
  std::vector<glim::EstimationFrame::ConstPtr> get_remaining_frames() override;

  struct Stats
  {
    std::int64_t groups_total = 0;
    std::int64_t groups_before_init = 0;  // dropped waiting for gravity alignment
    std::int64_t keyframes = 0;
    std::int64_t dropped_observations = 0;  // from GroupingBuffer
  };
  [[nodiscard]] Stats stats() const;

private:
  // Convert one keyframe that left the window into the glim contract: pose,
  // velocity, bias, plus a sparse IMU-local landmark cloud triangulated from
  // its folded observations.
  glim::EstimationFrame::Ptr to_estimation_frame(const vio::MarginalizedKeyframe & kf);

  // Run one group through the init gate (gravity alignment) and the
  // keyframe gate (displacement), pushing accepted groups into the window
  // solver and appending any keyframes that left the window to
  // marginalized_states. Shared by insert_visual_observations() and
  // get_remaining_frames() (the latter drives it from grouping_.finish()
  // instead of pop_ready()). Returns true iff the group was accepted as a
  // keyframe (i.e. push_keyframe() ran and the window's solved state may
  // have changed) -- callers use this to know whether latest_frame_ needs
  // refreshing.
  bool process_group(
    const ObservationGroup & group,
    std::vector<glim::EstimationFrame::ConstPtr> & marginalized_states);

  VisualInertialOdometryConfig config_;
  GroupingBuffer grouping_;
  vio::WindowSolver solver_;
  std::unique_ptr<glim::NaiveInitialStateEstimation> init_;

  bool has_keyframe_ = false;
  Eigen::Isometry3d last_keyframe_pose_ = Eigen::Isometry3d::Identity();
  long next_frame_id_ = 0;  // NOLINT(runtime/int) matches glim::EstimationFrame::id's type
  // Cache of insert_visual_observations()'s last return value: only
  // push_keyframe() (via process_group()) can change the window's solved
  // state or its landmark set, so re-deriving this via a full
  // window_snapshot() (window_snapshot() re-solves the whole window on
  // every call, see visual_odometry_window.hpp) on batches that accepted no
  // keyframe would be exact but wasteful. Refreshed only when at least one
  // keyframe was accepted this call.
  glim::EstimationFrame::ConstPtr latest_frame_;

  Stats stats_;
};

}  // namespace bagwiz::core::slam

#endif  // CORE__SLAM__VISUAL_ODOMETRY_HPP_
