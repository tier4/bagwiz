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
#include "bagwiz/core/slam/gnss_alignment.hpp"
#include "bagwiz/core/slam/gnss_sample.hpp"
#include "bagwiz/core/slam/lidar_scan.hpp"
#include "bagwiz/core/trajectory.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glim/mapping/callbacks.hpp>
#include <glim/mapping/global_mapping.hpp>
#include <glim/mapping/sub_map.hpp>
#include <glim/mapping/sub_mapping.hpp>
#include <glim/odometry/estimation_frame.hpp>
#include <glim/odometry/odometry_estimation_base.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/util/raw_points.hpp>
#include <glim/util/time_keeper.hpp>
#include <gtsam_points/types/point_cloud.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PoseTranslationPrior.h>

#include <algorithm>
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
// Variance [m^2] for an "unconstrained" axis of a Gaussian prior: large enough
// that its information (1/variance) is negligible next to the LiDAR/IMU factors,
// emulating the fixed-precision path's zero z-information without an actual
// singular (infinite-variance) model.
constexpr double kUnconstrainedZVariance = 1e8;  // ~ (1e4 m)^2

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

  // One GNSS fix in the local metric frame, with its stamp in seconds (matching
  // EstimationFrame::stamp) so it can be interpolated against submap timestamps.
  struct GnssMetric
  {
    double stamp = 0.0;
    Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
    std::array<double, 9> covariance{};  // local ENU, m^2, row-major
    std::uint8_t covariance_type = 0;    // sensor_msgs/NavSatFix: 0 UNKNOWN .. 3 KNOWN
  };
  // GNSS fixes collected via insert_gnss (config.enable_gnss only), consumed by
  // build_gnss_factors() in finish().
  std::vector<GnssMetric> gnss_points;
  // GNSS translation-prior factors built in finish() and injected into the
  // global factor graph via the on_smoother_update callback during optimize().
  std::vector<gtsam::NonlinearFactor::shared_ptr> gnss_factors;

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

  // Record one GNSS fix (already projected to the local metric frame). Stamp is
  // converted to seconds to match EstimationFrame stamps; the ENU covariance is
  // carried through for per-prior weighting.
  void add_gnss(const GnssPoint & p)
  {
    GnssMetric m;
    m.stamp = static_cast<double>(p.stamp_ns) * 1e-9;
    m.xyz = Eigen::Vector3d(p.position[0], p.position[1], p.position[2]);
    m.covariance = p.covariance;
    m.covariance_type = p.covariance_type;
    gnss_points.push_back(m);
  }

  // Linear-interpolate the GNSS position at time `t` (seconds). gnss_points must
  // be sorted by stamp; `t` outside the span clamps to the nearest endpoint.
  Eigen::Vector3d interpolate_gnss(double t) const
  {
    const auto right = std::lower_bound(
      gnss_points.begin(), gnss_points.end(), t,
      [](const GnssMetric & g, double tt) { return g.stamp < tt; });
    if (right == gnss_points.begin()) {
      return right->xyz;
    }
    if (right == gnss_points.end()) {
      return gnss_points.back().xyz;
    }
    const auto left = right - 1;
    const double tl = left->stamp;
    const double tr = right->stamp;
    const double p = (tr > tl) ? (t - tl) / (tr - tl) : 0.0;
    return (1.0 - p) * left->xyz + p * right->xyz;
  }

  // Covariance + type of the GNSS fix closest in time to `t`. gnss_points must be
  // sorted by stamp. Used to weight a submap's prior; nearest (not interpolated)
  // since covariance matrices do not interpolate linearly and adjacent fixes are
  // ~equally representative over a submap's short span.
  const GnssMetric & nearest_gnss(double t) const
  {
    const auto right = std::lower_bound(
      gnss_points.begin(), gnss_points.end(), t,
      [](const GnssMetric & g, double tt) { return g.stamp < tt; });
    if (right == gnss_points.begin()) {
      return *right;
    }
    if (right == gnss_points.end()) {
      return gnss_points.back();
    }
    const auto left = right - 1;
    return (t - left->stamp <= right->stamp - t) ? *left : *right;
  }

  // Build GNSS translation-prior factors from the collected submaps + fixes
  // (ported from glim_ext's gnss_global backend, run synchronously instead of in
  // a background thread). Leaves gnss_factors empty unless at least two submaps
  // are fully covered by the GNSS timespan and the SLAM baseline between the
  // first and last of them exceeds config.gnss_min_baseline.
  void build_gnss_factors()
  {
    gnss_factors.clear();
    if (gnss_points.size() < 2 || entries.empty()) {
      return;
    }

    std::sort(
      gnss_points.begin(), gnss_points.end(),
      [](const GnssMetric & a, const GnssMetric & b) { return a.stamp < b.stamp; });
    const double t_lo = gnss_points.front().stamp;
    const double t_hi = gnss_points.back().stamp;

    // Antenna lever-arm in the submap-origin sensor frame. config.gnss_antenna_offset
    // is the antenna phase center in the cloud (LiDAR) frame; the submap origin X(i)
    // is the LiDAR pose for the CT backend but the IMU pose for the CPU backend, so
    // re-express the antenna point in the IMU frame there (p_imu = T_imu_lidar *
    // p_lidar). {0,0,0} leaves it zero -> identical to the no-correction path.
    Eigen::Vector3d lever_origin(
      config.gnss_antenna_offset[0], config.gnss_antenna_offset[1], config.gnss_antenna_offset[2]);
    if (config.t_lidar_imu) {
      const Eigen::Isometry3d T_lidar_imu = detail::to_isometry(*config.t_lidar_imu);
      lever_origin = T_lidar_imu.inverse() * lever_origin;
    }

    std::vector<std::uint64_t> ids;
    std::vector<std::array<double, 3>> est;
    std::vector<std::array<double, 3>> offsets;  // per-submap antenna offset in world
    std::vector<std::array<double, 3>> gnss;
    std::vector<std::array<double, 9>> covs;  // nearest-fix ENU covariance per submap
    std::vector<std::uint8_t> cov_types;
    for (const auto & entry : entries) {
      if (!entry.submap || entry.frames.empty()) {
        continue;
      }
      // Only constrain submaps whose whole frame span is covered by GNSS, so the
      // mid-frame stamp interpolates between real fixes (mirrors glim_ext's
      // submap-within-window check).
      if (entry.frames.front().stamp < t_lo || entry.frames.back().stamp > t_hi) {
        continue;
      }
      const double t_mid = entry.frames[entry.frames.size() / 2].stamp;
      const Eigen::Vector3d origin = entry.submap->T_world_origin.translation();
      // Rotate the body-fixed lever-arm into the world frame with the submap's
      // pre-optimization orientation (Option A: the heading used for the offset is
      // frozen at build time; iSAM2 then moves the origin under the prior).
      const Eigen::Vector3d offset_world = entry.submap->T_world_origin.rotation() * lever_origin;
      const Eigen::Vector3d fix = interpolate_gnss(t_mid);
      const GnssMetric & near = nearest_gnss(t_mid);
      ids.push_back(static_cast<std::uint64_t>(entry.submap->id));
      est.push_back({origin.x(), origin.y(), origin.z()});
      offsets.push_back({offset_world.x(), offset_world.y(), offset_world.z()});
      gnss.push_back({fix.x(), fix.y(), fix.z()});
      covs.push_back(near.covariance);
      cov_types.push_back(near.covariance_type);
    }
    if (ids.size() < 2) {
      return;
    }

    // Pre-optimization baseline: too little motion makes the planar alignment
    // ill-conditioned (matches glim_ext's min_baseline gate).
    const double dx = est.front()[0] - est.back()[0];
    const double dy = est.front()[1] - est.back()[1];
    const double dz = est.front()[2] - est.back()[2];
    if (std::sqrt(dx * dx + dy * dy + dz * dz) < config.gnss_min_baseline) {
      return;
    }

    // Estimate the world<-GNSS transform (antenna-to-antenna so the lever-arm does
    // not contaminate the fit) and map each fix back onto its submap origin; that
    // mapped position is the submap's translation-prior target. The fitted ENU->world
    // rotation lets each fix's covariance be expressed in the world frame.
    const GnssOffsetTargets aligned = gnss_targets_with_offset(est, offsets, gnss);
    if (aligned.targets.size() != ids.size()) {
      return;
    }

    // Fixed-precision fallback (used when a fix has no usable covariance or
    // gnss_use_covariance is off): the original glim_ext-style behavior.
    const Eigen::Vector3d precisions(
      config.gnss_prior_inf_scale[0], config.gnss_prior_inf_scale[1],
      config.gnss_prior_inf_scale[2]);
    const gtsam::SharedNoiseModel fixed_model = gtsam::noiseModel::Diagonal::Precisions(precisions);

    // Vertical (z) handling mirrors the fixed path: honor a configured z precision,
    // otherwise leave height effectively unconstrained (a large variance ~ zero
    // information) so GNSS height — the weakest GNSS axis — does not fight the LiDAR.
    const double z_variance = config.gnss_prior_inf_scale[2] > 0.0
                                ? 1.0 / config.gnss_prior_inf_scale[2]
                                : kUnconstrainedZVariance;

    using gtsam::symbol_shorthand::X;
    gnss_factors.reserve(ids.size());
    for (std::size_t i = 0; i < ids.size(); ++i) {
      const gtsam::Point3 target(
        aligned.targets[i][0], aligned.targets[i][1], aligned.targets[i][2]);

      gtsam::SharedNoiseModel base = fixed_model;
      if (config.gnss_use_covariance && cov_types[i] != kNavSatCovarianceTypeUnknown) {
        // Horizontal ENU covariance {c_ee, c_en, c_ne, c_nn} from the nearest fix,
        // rotated into the world frame, inflated and floored; z left per z_variance.
        const std::array<double, 4> cov_h = {covs[i][0], covs[i][1], covs[i][3], covs[i][4]};
        const std::array<double, 9> w = gnss_world_prior_covariance(
          cov_h, aligned.world_from_enu_cos, aligned.world_from_enu_sin,
          config.gnss_horizontal_sigma_floor, config.gnss_covariance_inflation, z_variance);
        Eigen::Matrix3d cov_w;
        cov_w << w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7], w[8];
        base = gtsam::noiseModel::Gaussian::Covariance(cov_w);
      }

      // Robust-wrap so one multipath outlier cannot dominate; a Huber k of 0
      // disables it.
      gtsam::SharedNoiseModel model = base;
      if (config.gnss_robust_huber_k > 0.0) {
        model = gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Huber::Create(config.gnss_robust_huber_k), base);
      }

      gtsam::NonlinearFactor::shared_ptr factor(
        new gtsam::PoseTranslationPrior<gtsam::Pose3>(X(ids[i]), target, model));
      gnss_factors.push_back(factor);
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
    // what write_pcd expects): keep it only if every frame with points also
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

  // Removert-style dynamic-point removal over the merged map. First runs a fine
  // range-image filter (remove step); then, if enabled, re-evaluates the removed
  // points at progressively coarser range-image resolutions to recover false-
  // negative static structure (revert step). Points and parallel intensities are
  // filtered together. A no-op for an empty map.
  void apply_removert_filter(CloudMap & result, bool with_intensity) const
  {
    if (result.points.empty()) {
      return;
    }

    RemovertConfig rc;
    rc.vertical_fov_deg = config.removert_vertical_fov_deg;
    rc.horizontal_fov_deg = config.removert_horizontal_fov_deg;
    rc.remove_resolutions = config.removert_remove_resolutions;
    rc.revert_resolutions = config.removert_revert_resolutions;
    rc.adaptive_coeff = config.removert_adaptive_coeff;
    rc.valid_diff_upper_bound = config.removert_valid_diff_upper_bound;
    rc.enable_revert = config.removert_revert;

    // Transform every frame's full points into world-frame scan views and feed
    // them directly into RemovertFilter. Using the move overload avoids keeping
    // an intermediate copy of all scan points.
    RemovertFilter filter(rc, result.points);
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
        const Eigen::Vector3d origin = T_world_frame.translation();
        std::vector<std::array<float, 3>> world_points;
        world_points.reserve(ref.points.size());
        for (const auto & p : ref.points) {
          const Eigen::Vector3d local(p[0], p[1], p[2]);
          const Eigen::Vector3d world = T_world_frame * local;
          world_points.push_back(
            {static_cast<float>(world.x()), static_cast<float>(world.y()),
             static_cast<float>(world.z())});
        }
        filter.add_scan({origin.x(), origin.y(), origin.z()}, std::move(world_points));
      }
    }

    std::vector<char> keep = filter.filter();
    const std::size_t reverted_count = filter.reverted_count();

    // Apply the final keep mask in place, compacting points (and intensities,
    // when present) to the survivors while preserving order.
    std::vector<std::array<float, 3>> kept_points;
    std::vector<float> kept_intensities;
    kept_points.reserve(result.points.size());
    if (with_intensity) {
      kept_intensities.reserve(result.intensities.size());
    }
    std::size_t removed = 0;
    for (std::size_t i = 0; i < result.points.size(); ++i) {
      if (keep[i] != 0) {
        kept_points.push_back(result.points[i]);
        if (with_intensity) {
          kept_intensities.push_back(result.intensities[i]);
        }
      } else {
        ++removed;
      }
    }
    result.points = std::move(kept_points);
    result.intensities = std::move(kept_intensities);
    result.removert_removed_count = removed;
    result.removert_reverted_count = reverted_count;
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

void CloudMapper::insert_gnss(const GnssPoint & gnss)
{
  // GNSS factors live only in the global graph, so a fix is meaningful only when
  // global mapping runs; ignore otherwise. Buffered now, turned into submap
  // priors in finish().
  if (!impl_->config.enable_gnss) {
    return;
  }
  impl_->add_gnss(gnss);
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
  // cloud.
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

  // Build GNSS translation priors (config.enable_gnss) from the collected
  // submaps + fixes. They are injected into the global factor graph during
  // optimize() via the on_smoother_update callback below.
  std::size_t gnss_count = 0;
  if (impl_->config.enable_gnss) {
    impl_->build_gnss_factors();
    gnss_count = impl_->gnss_factors.size();
  }

  // The on_smoother_update slot is process-global, so register our injector only
  // around our own optimize() and remove it right after. Register first, capturing
  // the slot id, then hand it to an RAII guard whose destructor removes it. The
  // guard also protects against a throwing optimize() leaving a dangling `impl`
  // callback on the slot (which would fire — and dereference freed memory — for any
  // later mapper instance in the same process, e.g. across tests).
  int gnss_slot_id = -1;
  if (gnss_count > 0) {
    Impl * impl = impl_.get();
    gnss_slot_id = glim::GlobalMappingCallbacks::on_smoother_update.add(
      [impl](gtsam_points::ISAM2Ext &, gtsam::NonlinearFactorGraph & new_factors, gtsam::Values &) {
        // GlobalMapping::optimize() fires on_smoother_update exactly once, and
        // all submap poses X(i) already exist in iSAM2 by now, so the translation
        // priors are valid. Clearing after adding is a belt-and-suspenders guard
        // so they enter the graph exactly once even if GLIM's call count changes.
        if (!impl->gnss_factors.empty()) {
          new_factors.add(impl->gnss_factors);
          impl->gnss_factors.clear();
        }
      });
  }
  // id is the slot handle from the registration above (or -1 when no injector was
  // registered), so the destructor's guard is genuinely conditional.
  struct ScopedGnssCallback
  {
    int id;
    ~ScopedGnssCallback()
    {
      if (id >= 0) {
        glim::GlobalMappingCallbacks::on_smoother_update.remove(id);
      }
    }
  } gnss_callback{gnss_slot_id};

  // Heavy step: global matching-based iSAM2 optimization. Updates each held
  // submap's T_world_origin in place (GlobalMapping::update_submaps). With the
  // GNSS callback registered, the priors enter the graph in this single update.
  impl_->global_mapping->optimize();

  CloudMap result;
  result.gnss_factor_count = gnss_count;
  impl_->fill_trajectory(result);
  impl_->fill_map(result);
  return result;
}

void CloudMapper::apply_removert_filter(CloudMap & map)
{
  if (!impl_->config.enable_removert || map.points.empty()) {
    return;
  }
  const bool with_intensity =
    !map.intensities.empty() && map.intensities.size() == map.points.size();
  impl_->apply_removert_filter(map, with_intensity);
}

}  // namespace bagwiz::core::slam
