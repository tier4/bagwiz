// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "visual_odometry.hpp"  // NOLINT(build/include_subdir) src-local header

#include <glim/odometry/callbacks.hpp>
#include <glim/odometry/initial_state_estimation.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <memory>
#include <utility>
#include <vector>

namespace bagwiz::core::slam
{

VisualInertialOdometry::VisualInertialOdometry(VisualInertialOdometryConfig config)
: config_(std::move(config)),
  grouping_(
    GroupingBuffer::Config{
      .anchor_camera_id = config_.anchor_camera_id,
      .period_ns = config_.anchor_period_ns,
      .camera_count = config_.t_imu_cams.size(),
    }),
  solver_(
    vio::WindowConfig{
      .t_imu_cams = config_.t_imu_cams,
      .window_span_ns = config_.anchor_period_ns,
      .obs_sigma = config_.obs_sigma,
      .max_keyframes = config_.max_window_keyframes,
    }),
  // NaiveInitialStateEstimation reads glim's GlobalConfig
  // (config_odometry's "initialization_window_size", default 1.0 s) and
  // falls back to that default when no config file is installed -- the
  // exact path SubMappingParams's default constructor already exercises in
  // production, so no config file is required to run this class.
  init_(
    std::make_unique<glim::NaiveInitialStateEstimation>(
      config_.T_lidar_imu, Eigen::Matrix<double, 6, 1>::Zero()))
{
}

// Out-of-line so glim::NaiveInitialStateEstimation (forward-declared in the
// header) is complete where init_'s unique_ptr destructor is instantiated.
VisualInertialOdometry::~VisualInertialOdometry() = default;

void VisualInertialOdometry::insert_imu(
  double stamp, const Eigen::Vector3d & linear_acc, const Eigen::Vector3d & angular_vel)
{
  // Fires glim::OdometryEstimationCallbacks::on_insert_imu, same as every
  // other OdometryEstimationBase override.
  OdometryEstimationBase::insert_imu(stamp, linear_acc, angular_vel);
  solver_.insert_imu(stamp, linear_acc, angular_vel);
  if (!solver_.initialized()) {
    // Gravity alignment still pending: feed the initializer too. Once
    // initialized, further IMU samples are the solver's concern alone.
    init_->insert_imu(stamp, linear_acc, angular_vel);
  }
}

bool VisualInertialOdometry::process_group(
  const ObservationGroup & group,
  std::vector<glim::EstimationFrame::ConstPtr> & marginalized_states)
{
  ++stats_.groups_total;

  if (!solver_.initialized()) {
    const glim::EstimationFrame::ConstPtr init_frame = init_->initial_pose();
    if (!init_frame) {
      // Gravity alignment not ready yet: KLT continuity across the dropped
      // group is the frontend's concern, not ours -- these observations are
      // simply never seen again.
      ++stats_.groups_before_init;
      return false;
    }
    solver_.initialize(
      group.anchor_stamp_ns, init_frame->T_world_imu, Eigen::Vector3d::Zero(),
      init_frame->imu_bias);
    // Fall through to the keyframe gate below: the group that completes
    // gravity alignment is always accepted as the window's first keyframe.
    // (WindowSolver::initialize() already pushed this same anchor stamp
    // into the window, so predict_T_world_imu() below is valid immediately;
    // push_keyframe() attaches this group's observations to that anchor
    // state rather than predicting a separate one.)
  }

  // Keyframe gate. predict_T_world_imu(group.anchor_stamp_ns) is always
  // valid here: the window holds at least the just-initialized (or
  // previously pushed) anchor state.
  const Eigen::Isometry3d predicted = solver_.predict_T_world_imu(group.anchor_stamp_ns).value();
  bool is_keyframe = !has_keyframe_;  // first keyframe ever is unconditional
  if (has_keyframe_) {
    const Eigen::Isometry3d delta = last_keyframe_pose_.inverse() * predicted;
    is_keyframe = delta.translation().norm() >= config_.keyframe_min_trans ||
                  Eigen::AngleAxisd(delta.linear()).angle() >= config_.keyframe_min_rot;
  }

  if (!is_keyframe) {
    return false;  // displacement gate not met: discard (not fed to the solver)
  }

  ++stats_.keyframes;
  has_keyframe_ = true;

  for (const vio::MarginalizedKeyframe & kf : solver_.push_keyframe(group)) {
    marginalized_states.push_back(to_estimation_frame(kf));
  }

  // Re-derive the gate's reference pose from the just-completed LM solve,
  // NOT the pre-solve `predicted` above: push_keyframe() folds in this
  // keyframe's own visual factors (and, once >= 2 keyframes share a track,
  // corrects earlier ones too), so `predicted` can be off from the solved
  // pose by the same order of magnitude as that correction (a few cm at
  // these sigmas). Gating the NEXT group against the stale, uncorrected
  // `predicted` would silently bake this keyframe's own vision correction
  // into the apparent displacement, contaminating keyframe SELECTION with
  // vision noise rather than just IMU drift (shipped poses are unaffected
  // either way -- they always read the solved kf.nav). Post-push,
  // group.anchor_stamp_ns equals the just-pushed keyframe's own anchor
  // (window.back()), so predict_T_world_imu's zero-length-integration
  // fallback returns that keyframe's solved pose directly -- the window
  // cannot be empty here, we just pushed into it.
  last_keyframe_pose_ = solver_.predict_T_world_imu(group.anchor_stamp_ns).value();
  return true;
}

glim::EstimationFrame::ConstPtr VisualInertialOdometry::insert_visual_observations(
  std::span<const VisualObservation> observations,
  std::vector<glim::EstimationFrame::ConstPtr> & marginalized_states)
{
  grouping_.insert(observations);

  const std::size_t batch_start = marginalized_states.size();
  bool any_keyframe_accepted = false;
  for (const ObservationGroup & group : grouping_.pop_ready()) {
    any_keyframe_accepted = process_group(group, marginalized_states) || any_keyframe_accepted;
  }

  // Fire on_marginalized_frames once per batch (mirroring
  // odometry_estimation_imu.cpp:346), with only the frames THIS call
  // marginalized -- marginalized_states itself is caller-owned and
  // append-only, so it may already carry earlier batches' entries.
  if (marginalized_states.size() > batch_start) {
    const std::vector<glim::EstimationFrame::ConstPtr> new_frames(
      marginalized_states.begin() + static_cast<std::ptrdiff_t>(batch_start),
      marginalized_states.end());
    glim::OdometryEstimationCallbacks::on_marginalized_frames(new_frames);
  }

  // Only push_keyframe() (inside process_group(), when it accepts a group)
  // can change the window's solved state or its landmark set, so
  // window_snapshot() -- which re-solves the whole window every call, see
  // its declaration -- only needs to run again when this batch actually
  // accepted a keyframe (which implies solver_.initialized()). Otherwise
  // latest_frame_ from the last such refresh is still exact.
  if (any_keyframe_accepted) {
    const std::vector<vio::MarginalizedKeyframe> snapshot = solver_.window_snapshot();
    latest_frame_ = snapshot.empty() ? nullptr : to_estimation_frame(snapshot.back());
  }
  return latest_frame_;
}

std::vector<glim::EstimationFrame::ConstPtr> VisualInertialOdometry::get_remaining_frames()
{
  std::vector<glim::EstimationFrame::ConstPtr> out;

  for (const ObservationGroup & group : grouping_.finish()) {
    process_group(group, out);
  }
  if (solver_.initialized()) {
    for (const vio::MarginalizedKeyframe & kf : solver_.window_snapshot()) {
      out.push_back(to_estimation_frame(kf));
    }
  }

  if (!out.empty()) {
    glim::OdometryEstimationCallbacks::on_marginalized_frames(out);
  }
  return out;
}

VisualInertialOdometry::Stats VisualInertialOdometry::stats() const
{
  Stats result = stats_;
  result.dropped_observations = grouping_.dropped_count();
  return result;
}

glim::EstimationFrame::Ptr VisualInertialOdometry::to_estimation_frame(
  const vio::MarginalizedKeyframe & kf)
{
  auto frame = std::make_shared<glim::EstimationFrame>();
  frame->id = next_frame_id_++;
  frame->stamp = 1e-9 * static_cast<double>(kf.stamp_ns);
  // T_lidar_imu carries T_cam0_imu, so set_T_world_sensor keeps
  // T_world_lidar == T_world_cam0 -- the camera-frame trajectory convention
  // the command layer's TUM export relies on. Assign it BEFORE
  // set_T_world_sensor, which syncs the other pose through it.
  frame->T_lidar_imu = config_.T_lidar_imu;
  frame->set_T_world_sensor(glim::FrameID::IMU, kf.T_world_imu);
  // set_T_world_sensor's frame_id parameter only steers ITS OWN dispatch; it
  // does not touch the member of the same name (it's shadowed). glim's own
  // odometry sets this member explicitly next to the pose assignment (see
  // odometry_estimation_imu.cpp:191,321) -- do the same here, otherwise
  // frame_id would stay at its value-initialized FrameID::WORLD.
  frame->frame_id = glim::FrameID::IMU;
  frame->v_world_imu = kf.v_world_imu;
  frame->imu_bias = kf.imu_bias;
  frame->raw_frame =
    nullptr;  // guarded downstream: SubMapping re-deskews only when raw_frame is set
  // The sparse landmark cloud is what satisfies GLIM's non-null frame
  // requirement downstream; points are sensor-local (the frame_id frame). A
  // keyframe can legitimately carry zero landmarks (textureless window) --
  // it is still emitted, with an empty cloud; that policy is owned by the
  // camera-only CloudMapper wiring, not this class.
  std::vector<Eigen::Vector4d> points;
  points.reserve(kf.landmarks_world.size());
  const Eigen::Isometry3d T_imu_world = kf.T_world_imu.inverse();
  for (const auto & p_world : kf.landmarks_world) {
    points.emplace_back((T_imu_world * p_world).homogeneous());
  }
  auto cloud = std::make_shared<gtsam_points::PointCloudCPU>(points);
  // gtsam_points' voxel accumulation (IncrementalVoxelMap::insert, run from
  // SubMapping::insert_keyframe) reads per-point covariances UNCONDITIONALLY,
  // so a points-only cloud segfaults there. Attach a constant isotropic
  // placeholder: in camera-only mode every covariance consumer downstream
  // (VGICP factors, overlap estimation) is suppressed by configuration, so
  // the magnitude is immaterial -- it satisfies the container contract, it
  // does not weight anything.
  Eigen::Matrix4d placeholder_cov = Eigen::Matrix4d::Zero();
  placeholder_cov.topLeftCorner<3, 3>() = 0.01 * Eigen::Matrix3d::Identity();  // (0.1 m)^2
  cloud->add_covs(std::vector<Eigen::Matrix4d>(points.size(), placeholder_cov));
  frame->frame = cloud;
  return frame;
}

}  // namespace bagwiz::core::slam
