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
#include "bagwiz/core/tf/trajectory.hpp"

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
// Every GLIM / Eigen / GTSAM type is hidden behind a
// pimpl so this header (and the `slam` command that drives it) stays free of
// GLIM includes; only cloud_mapper.cpp pulls GLIM in, and the whole translation
// unit is compiled only when BAGWIZ_WITH_SLAM is on. No ROS node / pub-sub is
// involved — GLIM's modules are called directly.
//
// Usage: feed scans in timestamp order with insert(), then call finish() once
// to flush, run the global optimization, and obtain the map + trajectory.
namespace bagwiz::core::slam
{

// Point-cloud resolution/extent control for CloudMapper. `input_resolution` is
// the single "map resolution" knob: it sets GLIM's LiDAR preprocessor downsample
// voxel (the per-scan detail floor) AND the voxel the exported map is merged at.
// Unlike a pure export voxel it feeds the optimizer, so changing it also changes
// the trajectory (not just the map's appearance). The range crop bounds which
// returns enter the pipeline at all.
struct CloudMapperConfig
{
  // Voxel side length [m] used for BOTH the GLIM input downsample (per-scan,
  // applied after the range crop) and the exported-map merge (optimized per-frame
  // points are merged into voxels of this size). Smaller = denser map and finer
  // SLAM input, at more points and runtime. Default 0.15 matches GLIM's stock
  // downsample_resolution, so the default trajectory is unchanged from the
  // pre-flag behavior. Must be > 0.
  double input_resolution = 0.15;

  // Range crop [m] applied by the LiDAR preprocessor before downsampling: a point
  // whose sensor-frame range is below range_min or above range_max is dropped and
  // never enters the trajectory or the map. Defaults match GLIM's stock
  // 1.0 / 100.0. Require 0 < range_min < range_max.
  double range_min = 1.0;
  double range_max = 100.0;

  // LiDAR↔IMU extrinsic. nullopt → LiDAR-only CT odometry (the M2 behavior; IMU
  // disabled in sub/global mapping). A value → LiDAR-IMU CPU odometry with that
  // extrinsic, and IMU enabled in sub/global mapping; feed IMU via insert_imu().
  // Convention is GLIM's T_lidar_imu (p_lidar = T_lidar_imu * p_imu).
  std::optional<SensorTransform> t_lidar_imu;

  // Fill in poses for the SLAM initialization ("start") window. GLIM's odometry
  // emits no frame over its opening window (the LiDAR-IMU init, ~1 s), leaving the
  // trajectory's opening window empty. When true, the pre-init scans are buffered
  // and filled in by scan-matching each against the globally-optimized map (works
  // in LiDAR-only mode); with t_lidar_imu set the buffered IMU additionally seeds
  // each registration's initial guess and is the fallback on a failed fit. See
  // core/slam/scan_match_fill.hpp (and warmup_fill.hpp for the IMU path).
  bool fill_start = false;

  // Fill in poses for the SLAM cooldown ("end") window — the symmetric
  // counterpart of fill_start. The newest scans stay inside the odometry
  // smoother window at end-of-sequence and never reach a finalized submap, so the
  // trajectory stops one window short of the last input scan. When true, the
  // trailing scans are buffered and filled in by scan-matching each against the
  // optimized map (LiDAR-only included); with t_lidar_imu set the buffered IMU
  // additionally seeds each initial guess and is the fallback. See
  // core/slam/scan_match_fill.hpp.
  bool fill_end = false;

  // Inlier-fraction acceptance gate [0..1] for the warmup/cooldown fill
  // scan-matching: a scan-to-map fit is accepted only when at least this fraction
  // of its source points find an inlier correspondence. Higher = stricter (an
  // endpoint may stay unfilled); lower = looser (risking a bad fit that
  // poisons the growing scan-match target). Applies to both fill windows and
  // has no effect when fill_start and fill_end are both false. Default 0.7
  // matches ScanMatchParams' loose-init gate. Require 0 < x <= 1.
  double fill_min_inlier_fraction = 0.7;

  // Keyframes accumulated before GLIM finalizes a submap (GLIM SubMappingParams
  // max_num_keyframes). Smaller = more, smaller submaps: finer loop-closure
  // granularity and (via more GNSS-covered submaps) can unblock GNSS priors, but
  // super-linearly more sub-mapping work per submap and thinner, weaker submaps.
  // Larger = fewer, larger submaps: a cheaper global graph but coarser correction.
  // Default 15 == GLIM stock, so the default trajectory is unchanged. Must be > 0.
  int submap_max_keyframes = 15;

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

  // Number of CPU threads passed to GLIM and to the scan-matching endpoint
  // fill's per-registration work (covariance estimation + GICP
  // correspondences). Must be positive. 1 is the deterministic path.
  int num_threads = 4;

  // Route GLIM through its CUDA backends and use GPU export voxelization. With
  // t_lidar_imu set this selects OdometryEstimationGPU; without it odometry stays
  // CT (no GPU LiDAR-only backend exists) but sub/global mapping still enable
  // GPU VGICP. Honored only in a BAGWIZ_WITH_SLAM_CUDA build; constructing with
  // use_gpu=true in a non-CUDA build throws. The per-frame host stash is int16-
  // quantized in this mode to roughly halve host memory on large bags. Outside
  // the CPU reproducibility guarantee.
  bool use_gpu = false;

  // When false (default), insert() runs a two-stage pipeline: the caller thread
  // does bag read + GLIM preprocess (producer), and an internal worker thread runs
  // GLIM odometry + sub/global mapping (consumer). The consumer processes events
  // in strict bag order, so the trajectory and map are bit-identical to the serial
  // path — it only overlaps the CPU preprocess (and bag read) with the GPU
  // odometry/mapping to cut wall-clock. Set true to force the fully synchronous
  // single-thread path (e.g. for A/B timing or a strictly serial run).
  bool disable_pipeline = false;
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
// caller can hand them straight to write_pcd / write_tum.
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

  // Number of start-window poses filled in by scan-matching and prepended to
  // `trajectory` (config.fill_start). 0 when the fill was off or there were no
  // pre-init scans to fill.
  std::size_t filled_start_pose_count = 0;

  // Number of end-window poses filled in by scan-matching and appended to
  // `trajectory` (config.fill_end). 0 when the fill was off or there were no
  // trailing scans past the last estimated frame.
  std::size_t filled_end_pose_count = 0;

  // True when the start-window fill was abandoned because the pre-init scan buffer
  // overflowed before odometry converged (a very long static/slow start) — lets
  // the caller distinguish "nothing to fill" from "gave up", which have the
  // same filled_start_pose_count of 0.
  bool warmup_overflowed = false;

  // Wall-clock breakdown of finish(), for the command layer's log line: the
  // global iSAM2 optimization, the scan-matching endpoint fill (start + end
  // windows together), and the export map fill. Diagnostic only — without the
  // split, a long endpoint fill (the end window alone can hold a full
  // odometry smoother window of scans) is indistinguishable from a slow
  // optimizer in the finalization total.
  double optimize_seconds = 0.0;
  double window_fill_seconds = 0.0;
  double export_seconds = 0.0;
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
