// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/cloud_mapper.hpp"

#include "bagwiz/core/slam/cloud_filters.hpp"
#include "bagwiz/core/slam/glim_estimator.hpp"
#include "bagwiz/core/slam/lidar_scan.hpp"
#include "bagwiz/core/trajectory.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glim/mapping/global_mapping.hpp>
#include <glim/mapping/sub_map.hpp>
#include <glim/mapping/sub_mapping.hpp>
#include <glim/odometry/estimation_frame.hpp>
#include <glim/odometry/odometry_estimation_base.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/util/raw_points.hpp>
#include <glim/util/time_keeper.hpp>
#include <gtsam_points/types/point_cloud.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bagwiz::core::slam
{
namespace
{
core::TrajectoryPose to_pose(double stamp, const Eigen::Isometry3d & transform)
{
  const Eigen::Vector3d translation = transform.translation();
  Eigen::Quaterniond rotation(transform.rotation());
  rotation.normalize();

  core::TrajectoryPose pose;
  pose.timestamp_ns = static_cast<std::int64_t>(std::llround(stamp * 1e9));
  pose.tx = translation.x();
  pose.ty = translation.y();
  pose.tz = translation.z();
  pose.qx = rotation.x();
  pose.qy = rotation.y();
  pose.qz = rotation.z();
  pose.qw = rotation.w();
  return pose;
}

// IMU in the mapping stages is enabled iff we run the LiDAR-IMU backend (an
// extrinsic was provided and insert_imu is fed). In LiDAR-only mode it must stay
// off — GLIM's defaults enable it, which would make insert_submap build IMU
// factors from uninitialised bias/velocity and warn that frames are not
// IMU-framed. Everything else keeps GLIM's stock defaults: the exported map's
// density is controlled separately, by re-binning the optimized per-frame points
// (see CloudMapper::Impl::fill_map), so the sub mapping that drives the
// optimization is left untouched.
glim::SubMappingParams make_sub_mapping_params(bool enable_imu)
{
  glim::SubMappingParams params;
  params.enable_imu = enable_imu;
  return params;
}
glim::GlobalMappingParams make_global_mapping_params(bool enable_imu)
{
  glim::GlobalMappingParams params;
  params.enable_imu = enable_imu;
  return params;
}
}  // namespace

struct CloudMapper::Impl
{
  // Full points of one odometry frame, captured at insert() while GLIM still
  // holds them. Sub mapping subsamples keyframes and drops per-frame points to
  // save memory, so these full LiDAR-frame points (the only ones dense enough to
  // build a high-resolution map) must be copied out the moment they arrive.
  // Keyed by EstimationFrame::id so they can be paired with the frame's
  // optimized submap-relative pose at capture time.
  struct StashedPoints
  {
    std::vector<std::array<float, 3>> points;  // LiDAR-frame coordinates
    std::vector<float> intensities;            // empty unless the scan had intensities
  };

  // One frame of a submap, captured BEFORE the submap is inserted into global
  // mapping (which overwrites SubMap::T_world_origin with the chained global
  // estimate). T_origin_frame is the frame's pose relative to the submap origin
  // and is invariant under global optimization; the optimized world pose is
  // recovered later as (optimized T_world_origin) * T_origin_frame. The points
  // are this frame's full LiDAR-frame cloud, moved in from the stash.
  struct FrameRef
  {
    std::int64_t id = 0;
    double stamp = 0.0;
    Eigen::Isometry3d T_origin_frame = Eigen::Isometry3d::Identity();
    std::vector<std::array<float, 3>> points;  // LiDAR-frame, full density
    std::vector<float> intensities;            // parallel to points; may be empty
  };
  struct SubMapEntry
  {
    glim::SubMap::Ptr submap;  // kept so its T_world_origin can be read post-optimize
    std::vector<FrameRef> frames;
  };

  const CloudMapperConfig config;  // Con.4: set once at construction, never mutated
  glim::TimeKeeper time_keeper;
  glim::CloudPreprocessor preprocessor;
  // CT (LiDAR-only) or CPU (LiDAR-IMU) behind the common base interface.
  std::unique_ptr<glim::OdometryEstimationBase> odometry;
  std::unique_ptr<glim::SubMapping> sub_mapping;
  std::unique_ptr<glim::GlobalMapping> global_mapping;
  std::vector<SubMapEntry> entries;
  std::unordered_map<std::int64_t, StashedPoints> stash;  // frame id -> full points

  explicit Impl(const CloudMapperConfig & cfg)
  : config(cfg),
    odometry(detail::make_odometry_estimator(cfg.t_lidar_imu)),
    sub_mapping(
      std::make_unique<glim::SubMapping>(make_sub_mapping_params(cfg.t_lidar_imu.has_value()))),
    global_mapping(
      std::make_unique<glim::GlobalMapping>(
        make_global_mapping_params(cfg.t_lidar_imu.has_value())))
  {
  }

  // Copy a frame's full LiDAR-frame points (and intensities, if any) out of GLIM
  // before sub mapping drops them, keyed by id.
  void stash_frame(const glim::EstimationFrame::ConstPtr & frame)
  {
    if (!frame || !frame->frame || frame->frame->size() == 0) {
      return;
    }
    const auto & cloud = frame->frame;
    const std::size_t n = cloud->size();

    // GLIM stores each estimation frame's points in frame->frame_id coordinates,
    // and that frame DIFFERS between backends: the LiDAR frame for the CT
    // (LiDAR-only) backend, but the IMU frame for the CPU (LiDAR-IMU) backend
    // (GLIM builds it as points_imu = T_imu_lidar * points_lidar). The map and
    // trajectory downstream both place these points with T_world_lidar, so bring
    // every frame's points back into the LiDAR frame first. T_lidar_sensor is
    // identity for the CT backend (frame_id == LIDAR, points already LiDAR-frame)
    // and equals T_lidar_imu for the IMU backend (frame_id == IMU); GLIM's own
    // T_world_sensor() selects T_world_lidar / T_world_imu by frame_id. Skipping
    // this corrupts the whole map whenever the IMU<-LiDAR extrinsic is not
    // identity (e.g. a 180-deg-flipped IMU), placing every point off by exactly
    // that extrinsic.
    const Eigen::Isometry3d T_lidar_sensor =
      frame->T_world_lidar.inverse() * frame->T_world_sensor();

    StashedPoints stashed;
    stashed.points.reserve(n);
    // Intensities are sourced from the preprocessed input frame, NOT from
    // cloud->intensities. GLIM's LiDAR-only backend (OdometryEstimationCT) never
    // copies intensities onto its estimation-frame cloud, so cloud->has_intensities()
    // is false there; the LiDAR-IMU backend does copy them. raw_frame->intensities
    // is populated by the preprocessor in BOTH modes and is index-aligned 1:1 with
    // cloud->points (the estimation cloud is built from the same preprocessed points,
    // only deskewed — deskewing preserves count and order), so it pairs correctly
    // with the geometry read from cloud->points below.
    const bool has_intensities = frame->raw_frame && frame->raw_frame->intensities.size() == n;
    if (has_intensities) {
      stashed.intensities.reserve(n);
    }
    for (std::size_t i = 0; i < n; ++i) {
      const Eigen::Vector3d p = T_lidar_sensor * cloud->points[i].head<3>();
      stashed.points.push_back(
        {static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z())});
      if (has_intensities) {
        stashed.intensities.push_back(static_cast<float>(frame->raw_frame->intensities[i]));
      }
    }
    stash[frame->id] = std::move(stashed);
  }

  // Stash the frame's full points, then hand it to sub mapping.
  void feed_sub_mapping(const glim::EstimationFrame::ConstPtr & frame)
  {
    if (!frame) {
      return;
    }
    stash_frame(frame);
    sub_mapping->insert_frame(frame);
  }

  // Capture each frame's submap-local relative pose and pair it with the full
  // points stashed at insert time, then hand the submap to global mapping. Order
  // matters: insert_submap rewrites T_world_origin and drops the per-frame point
  // clouds, so the relative poses are read first.
  void capture_and_insert(const glim::SubMap::Ptr & submap)
  {
    SubMapEntry entry;
    entry.submap = submap;
    const Eigen::Isometry3d T_origin_world = submap->T_world_origin.inverse();
    entry.frames.reserve(submap->frames.size());
    for (const auto & frame : submap->frames) {
      if (!frame) {
        continue;
      }
      FrameRef ref;
      ref.id = frame->id;
      ref.stamp = frame->stamp;
      ref.T_origin_frame = T_origin_world * frame->T_world_lidar;
      const auto found = stash.find(frame->id);
      if (found != stash.end()) {
        ref.points = std::move(found->second.points);
        ref.intensities = std::move(found->second.intensities);
        stash.erase(found);
      }
      entry.frames.push_back(std::move(ref));
    }
    entries.push_back(std::move(entry));
    global_mapping->insert_submap(submap);
  }

  void drain_submaps()
  {
    for (const auto & submap : sub_mapping->get_submaps()) {
      if (submap) {
        capture_and_insert(submap);
      }
    }
  }

  // Optimized world pose per frame = T_world_origin * T_origin_frame. Keyed by
  // timestamp so the poses come out time-ordered and a duplicate stamp keeps one
  // entry (submap boundaries share no frames, but stay defensive).
  void fill_trajectory(CloudMap & result) const
  {
    std::map<std::int64_t, core::TrajectoryPose> poses;
    for (const auto & entry : entries) {
      if (!entry.submap) {
        continue;
      }
      const Eigen::Isometry3d & T_world_origin = entry.submap->T_world_origin;
      for (const auto & ref : entry.frames) {
        const Eigen::Isometry3d T_world_frame = T_world_origin * ref.T_origin_frame;
        const auto pose = to_pose(ref.stamp, T_world_frame);
        poses[pose.timestamp_ns] = pose;
      }
    }
    result.trajectory.reserve(poses.size());
    for (const auto & entry : poses) {
      result.trajectory.push_back(entry.second);
    }
  }

  // Rebuild the exported map from every frame's full points, placed at the
  // frame's globally-optimized world pose and merged at config.map_resolution.
  // This is the density decoupling: the optimization ran at GLIM's stock sub-map
  // density, but the map we emit is as dense as the requested export voxel allows.
  void fill_map(CloudMap & result) const
  {
    // Intensity is all-or-nothing across the whole map (mirrors GLIM's export and
    // what write_ply expects): keep it only if every frame with points also
    // carried intensities.
    bool any_points = false;
    bool all_intensity = true;
    for (const auto & entry : entries) {
      for (const auto & ref : entry.frames) {
        if (ref.points.empty()) {
          continue;
        }
        any_points = true;
        if (ref.intensities.size() != ref.points.size()) {
          all_intensity = false;
        }
      }
    }
    const bool with_intensity = any_points && all_intensity;

    VoxelGrid grid(config.map_resolution, with_intensity);
    for (const auto & entry : entries) {
      if (!entry.submap) {
        continue;
      }
      const Eigen::Isometry3d & T_world_origin = entry.submap->T_world_origin;
      for (const auto & ref : entry.frames) {
        if (ref.points.empty()) {
          continue;
        }
        const Eigen::Isometry3d T_world_frame = T_world_origin * ref.T_origin_frame;
        for (std::size_t i = 0; i < ref.points.size(); ++i) {
          const Eigen::Vector3d local(ref.points[i][0], ref.points[i][1], ref.points[i][2]);
          const Eigen::Vector3d world = T_world_frame * local;
          if (with_intensity) {
            grid.add(
              static_cast<float>(world.x()), static_cast<float>(world.y()),
              static_cast<float>(world.z()), ref.intensities[i]);
          } else {
            grid.add(
              static_cast<float>(world.x()), static_cast<float>(world.y()),
              static_cast<float>(world.z()));
          }
        }
      }
    }
    result.points = grid.points();
    result.intensities = grid.intensities();
  }
};

CloudMapper::CloudMapper(CloudMapperConfig config)
{
  // Silence GLIM's one-time construction chatter (it logs ~50 "config not found /
  // using default" lines while reading params; we drive it with no config dir on
  // purpose). RAII-restored so a throwing GLIM constructor cannot leave the
  // shared logger muted for the rest of the process; genuine runtime warnings
  // still surface afterwards.
  const detail::ScopedLoggerSilence silence;
  impl_ = std::make_unique<Impl>(config);
}
CloudMapper::~CloudMapper() = default;
CloudMapper::CloudMapper(CloudMapper &&) noexcept = default;
CloudMapper & CloudMapper::operator=(CloudMapper &&) noexcept = default;

void CloudMapper::insert_imu(const ImuSample & imu)
{
  const double stamp = static_cast<double>(imu.stamp_ns) * 1e-9;
  const Eigen::Vector3d linear_acc(
    imu.linear_acceleration[0], imu.linear_acceleration[1], imu.linear_acceleration[2]);
  const Eigen::Vector3d angular_vel(
    imu.angular_velocity[0], imu.angular_velocity[1], imu.angular_velocity[2]);
  // Route to all three stages (no-ops in LiDAR-only mode): odometry estimates
  // motion from it; sub/global mapping use it for their own IMU factors. Each
  // stage buffers IMU in its own preintegrator.
  impl_->odometry->insert_imu(stamp, linear_acc, angular_vel);
  impl_->sub_mapping->insert_imu(stamp, linear_acc, angular_vel);
  impl_->global_mapping->insert_imu(stamp, linear_acc, angular_vel);
}

void CloudMapper::insert(const LidarScan & scan)
{
  auto raw = std::make_shared<glim::RawPoints>();
  raw->stamp = static_cast<double>(scan.stamp_ns) * 1e-9;

  const std::size_t num_points = scan.points.size();
  raw->points.reserve(num_points);
  for (const auto & point : scan.points) {
    raw->points.emplace_back(point[0], point[1], point[2], 1.0);
  }
  if (!scan.intensities.empty()) {
    raw->intensities = scan.intensities;
  }

  // A time-less cloud is fed explicit zero per-point times (already
  // motion-undistorted), NOT an empty vector — that would make glim::TimeKeeper
  // synthesize order-based pseudo times and wrongly "deskew" a concatenated
  // cloud. (Mirrors CloudOdometry::insert.)
  if (scan.has_per_point_time && scan.times.size() == num_points) {
    raw->times = scan.times;
  } else {
    raw->times.assign(num_points, 0.0);
  }

  if (!impl_->time_keeper.process(raw)) {
    return;
  }

  const auto preprocessed = impl_->preprocessor.preprocess(raw);
  std::vector<glim::EstimationFrame::ConstPtr> marginalized;
  // The active-frame return is intentionally ignored here: the mapper's
  // trajectory comes from the globally-optimized submap poses in finish(), not
  // from the odometry estimate. Only the marginalized frames feed sub mapping.
  impl_->odometry->insert_frame(preprocessed, marginalized);

  // The marginalized odometry frames are this mapper's input to sub mapping; each
  // one's full points are stashed before sub mapping can drop them.
  for (const auto & frame : marginalized) {
    impl_->feed_sub_mapping(frame);
  }
  impl_->drain_submaps();
}

CloudMap CloudMapper::finish()
{
  // Flush the odometry smoother window — the remaining frames are marginalized
  // exactly as glim's async pipeline does at end of sequence — into sub mapping,
  // then force out the final submap.
  for (const auto & frame : impl_->odometry->get_remaining_frames()) {
    impl_->feed_sub_mapping(frame);
  }
  // This drain only sees submaps the flushed remaining frames newly completed —
  // get_submaps() destructively swaps its queue, so the submaps drained during
  // insert() are already gone. submit_end_of_sequence() then forces a final
  // submap out of whatever odometry frames remain; it builds a fresh submap
  // rather than pulling from that queue, so there is no overlap.
  impl_->drain_submaps();
  for (const auto & submap : impl_->sub_mapping->submit_end_of_sequence()) {
    if (submap) {
      impl_->capture_and_insert(submap);
    }
  }

  // Heavy step: global matching-based iSAM2 optimization. Updates each held
  // submap's T_world_origin in place (GlobalMapping::update_submaps).
  impl_->global_mapping->optimize();

  CloudMap result;
  impl_->fill_trajectory(result);
  impl_->fill_map(result);
  return result;
}

}  // namespace bagwiz::core::slam
