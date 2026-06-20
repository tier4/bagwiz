// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__CLOUD_MAPPER_HPP_
#define BAGWIZ__CORE__SLAM__CLOUD_MAPPER_HPP_

#include "bagwiz/core/slam/imu_sample.hpp"
#include "bagwiz/core/slam/lidar_scan.hpp"
#include "bagwiz/core/slam/sensor_transform.hpp"
#include "bagwiz/core/trajectory.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

// LiDAR-only optimized mapping over a sequence of scans. Extends the M1
// odometry path (GLIM's OdometryEstimationCT) by routing the marginalized
// frames through GLIM's SubMapping -> GlobalMapping — the same pipeline
// glim_rosbag uses for its final globally-optimized output — and then reading
// back the optimized global point-cloud map plus the globally-optimized
// per-scan trajectory.
//
// As with CloudOdometry, every GLIM / Eigen / GTSAM type is hidden behind a
// pimpl so this header (and the `slam` command that drives it) stays free of
// GLIM includes; only cloud_mapper.cpp pulls GLIM in, and the whole translation
// unit is compiled only when BAGWIZ_WITH_SLAM is on. No ROS node / pub-sub is
// involved — GLIM's modules are called directly.
//
// Usage: feed scans in timestamp order with insert(), then call finish() once
// to flush, run the global optimization, and obtain the map + trajectory.
namespace bagwiz::core::slam
{

// Density control for CloudMapper's exported map. This is deliberately
// decoupled from GLIM's internal sub-map density: the optimization always runs
// with GLIM's stock defaults (so the trajectory is reproducible and unaffected),
// while the exported map is rebuilt from every frame's full points placed at
// their globally-optimized poses and merged at `map_resolution`. Changing this
// therefore changes only the map's appearance, never the trajectory.
struct CloudMapperConfig
{
  // Voxel side length [m] of the exported map. The optimized per-frame points are
  // merged into voxels of this size; smaller = denser. The cloud preprocessor's
  // input voxel (~0.15 m) and 1–100 m range crop still bound how fine the real
  // data is, but a value below ~0.15 m can still recover detail from the offset
  // grids of overlapping frames (at the cost of a much larger map). Must be > 0.
  double map_resolution = 0.2;

  // LiDAR↔IMU extrinsic. nullopt → LiDAR-only CT odometry (the M2 behavior; IMU
  // disabled in sub/global mapping). A value → LiDAR-IMU CPU odometry with that
  // extrinsic, and IMU enabled in sub/global mapping; feed IMU via insert_imu().
  // Convention is GLIM's T_lidar_imu (p_lidar = T_lidar_imu * p_imu).
  std::optional<SensorTransform> t_lidar_imu;

  // GNSS global constraint (ported from glim_ext's gnss_global). When true, GNSS
  // points fed via insert_gnss() add horizontal translation priors on the submap
  // poses during the final global optimization, pinning the world frame to GNSS
  // and curbing drift. Defaults below mirror glim_ext's config_gnss_global.json.
  bool enable_gnss = false;

  // Minimum SLAM-estimated baseline [m] (distance between the first and last
  // GNSS-associated submap origins) before the world<-GNSS alignment is
  // estimated. Too little motion makes the planar rotation ill-conditioned, so
  // no GNSS factors are added until this is exceeded. Defaults to 10.0 to match
  // glim_ext's shipped config_gnss_global.json (its in-code fallback is 5.0).
  double gnss_min_baseline = 10.0;

  // Per-axis information (precision) of each GNSS translation prior, in the
  // GNSS-aligned world frame {x, y, z}. Used as the FALLBACK when a fix carries no
  // usable covariance (covariance_type UNKNOWN) or gnss_use_covariance is false.
  // The default leaves z at 0 so only the horizontal position is constrained (GNSS
  // height is typically the weakest axis); x/y at 1e3 pulls the submaps onto the
  // GNSS track. Mirrors prior_inf_scale.
  std::array<double, 3> gnss_prior_inf_scale{1e3, 1e3, 0.0};

  // When true (default), a fix carrying a KNOWN position covariance weights its
  // prior by that covariance (rotated into the world frame, inflated, floored)
  // instead of the fixed gnss_prior_inf_scale precision. NavSatFix covariance is
  // ~metre-level for SBAS/standalone fixes and cm-level for RTK, so this keeps a
  // fixed cm-tight prior from over-trusting a metre-level fix (and vice versa).
  // Falls back to gnss_prior_inf_scale per fix when the covariance is unavailable.
  bool gnss_use_covariance = true;

  // Lower bound [m] on each horizontal stddev of a covariance-derived prior, added
  // as an isotropic floor (sigma_floor^2 on each horizontal diagonal). Guards
  // against an over-optimistic receiver covariance dominating the graph. Unused by
  // the fixed-precision fallback.
  double gnss_horizontal_sigma_floor = 0.05;

  // Multiplicative inflation (>=1) on a covariance-derived stddev. GNSS formal
  // covariance is optimistic and consecutive fixes are time-correlated, so the
  // independent-prior model double-counts; inflate to compensate. 1.0 applies the
  // covariance as reported.
  double gnss_covariance_inflation = 1.0;

  // Huber robust-kernel threshold (whitened-residual / sigma units) wrapping each
  // GNSS prior so one multipath outlier cannot dominate; 0 disables the kernel.
  // 1.345 is the classic 95%-efficiency value.
  double gnss_robust_huber_k = 1.345;

  // GNSS antenna lever-arm: the antenna phase-center position expressed in the
  // cloud (LiDAR) frame, i.e. T_cloud_gnss.translation(). A NavSatFix reports the
  // ANTENNA position, but the GNSS prior constrains the submap-origin sensor pose,
  // so without this offset a non-trivial antenna mount biases every prior by a
  // heading-dependent amount that the rigid world<-GNSS fit cannot absorb. The
  // command layer resolves it from the bag's static TF (cloud frame <- NavSatFix
  // frame_id); {0,0,0} (the default) disables the correction and reproduces the
  // raw-antenna behavior. For the LiDAR-IMU backend the submap origin is the IMU
  // pose, so this LiDAR-frame offset is re-expressed in the IMU frame internally
  // using t_lidar_imu.
  std::array<double, 3> gnss_antenna_offset{0.0, 0.0, 0.0};
};

// One GNSS fix already projected into the local metric (ENU) frame the mapper
// aligns to. GLIM-free plain data; produced by the command layer (NavSatFix ->
// gnss_projector) and consumed by CloudMapper::insert_gnss.
struct GnssPoint
{
  std::int64_t stamp_ns = 0;         // fix timestamp, nanoseconds since epoch
  std::array<double, 3> position{};  // local metric meters {east, north, up}

  // Position covariance (m^2), row-major 3x3, in the SAME local ENU frame as
  // `position` (the projector preserves ENU axes over the local trajectory area).
  // Used to weight the prior; ignored when covariance_type is UNKNOWN.
  std::array<double, 9> covariance{};
  std::uint8_t covariance_type = 0;  // sensor_msgs/NavSatFix: 0 UNKNOWN .. 3 KNOWN
};

// Result of CloudMapper::finish(). All fields are GLIM-free plain data so the
// caller can hand them straight to write_ply / write_tum.
struct CloudMap
{
  // Globally-optimized LiDAR poses in the world frame, sorted by timestamp.
  std::vector<core::TrajectoryPose> trajectory;

  // Optimized global map: world-frame xyz, one entry per point.
  std::vector<std::array<float, 3>> points;

  // Per-point intensity, parallel to `points`. Empty unless every submap
  // carried intensities (mirrors GLIM's all-or-nothing export).
  std::vector<float> intensities;

  // Number of GNSS translation-prior factors applied during global
  // optimization. 0 when GNSS was disabled or could not initialize (no fixes
  // overlapping the submap timespan, or baseline below gnss_min_baseline).
  std::size_t gnss_factor_count = 0;
};

class CloudMapper
{
public:
  // `config` tunes the exported map's density (see CloudMapperConfig); the
  // default reproduces GLIM's stock pipeline.
  explicit CloudMapper(CloudMapperConfig config = {});
  ~CloudMapper();

  CloudMapper(const CloudMapper &) = delete;
  CloudMapper & operator=(const CloudMapper &) = delete;
  CloudMapper(CloudMapper &&) noexcept;
  CloudMapper & operator=(CloudMapper &&) noexcept;

  // Feed one IMU sample (LiDAR-IMU mode only; a no-op in LiDAR-only mode).
  // Forwarded to the odometry and the sub/global mapping stages, all of which
  // buffer it for their own preintegration. Samples must arrive in
  // non-decreasing timestamp order, interleaved with scans.
  void insert_imu(const ImuSample & imu);

  // Feed one GNSS fix, already projected to the local metric frame (see
  // GnssPoint). A no-op unless config.enable_gnss is set. Points are buffered
  // and turned into submap translation priors in finish(); they should arrive
  // in non-decreasing timestamp order.
  void insert_gnss(const GnssPoint & gnss);

  // Feed one scan. Scans must arrive in non-decreasing timestamp order. A scan
  // with no per-point time is fed with explicit zero per-point times (treated
  // as already motion-undistorted), bypassing GLIM's pseudo-time synthesis.
  void insert(const LidarScan & scan);

  // Flush the remaining in-flight frames, run the global optimization, and
  // return the optimized map + trajectory. Heavy: the global matching-based
  // iSAM2 optimization runs here.
  [[nodiscard]] CloudMap finish();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__CLOUD_MAPPER_HPP_
