// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__SCAN_MATCH_RECOVERY_HPP_
#define BAGWIZ__CORE__SLAM__SCAN_MATCH_RECOVERY_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
#include <vector>

// Scan-to-map registration for the SLAM warmup / cooldown windows.
//
// GLIM leaves two windows unestimated: the LiDAR-IMU initialization window at
// the START (odometry stays silent until IMU init converges) and the smoother
// window at the END (the newest scans never marginalize into a finalized
// submap). warmup_recovery.hpp reconstructs those poses by integrating the IMU
// away from the boundary frame, but that path (a) needs an IMU and (b) cannot
// cover a scan that falls outside the integrated IMU span (e.g. the very first
// scan, whose stamp precedes the first IMU sample). This module recovers the
// same poses geometrically instead: it registers each unestimated scan against
// a target built from the already-optimized neighboring frames, so it needs no
// IMU and leaves no gap.
//
// It is the GLIM-free, gtsam_points-only registration core (an iVox target + a
// GICP factor optimized with Levenberg-Marquardt, mirroring GLIM's own
// loose_initial_state_estimation). It knows nothing about GLIM, the bag, or the
// map: the SLAM wrapper (cloud_mapper.cpp) seeds the target from the optimized
// frames near the boundary, then chain-registers each window scan outward,
// using the IMU propagation result (when available) as the initial guess and a
// fallback. Keeping the registration here lets it be unit-tested with synthetic
// clouds without the GLIM stack.
namespace bagwiz::core::slam
{

// Tuning for the scan-to-map registration. Defaults follow GLIM's loose-init
// (iVox leaf 1.0 m) and stay single-threaded so a `--threads 1` run is
// run-to-run deterministic.
struct ScanMatchParams
{
  double voxel_resolution = 1.0;             // iVox target leaf size [m]
  double max_correspondence_distance = 1.0;  // GICP correspondence trimming [m]
  double min_inlier_fraction = 0.7;          // convergence gate (0..1)
  int k_neighbors = 10;                      // neighbors for covariance estimation
  int max_iterations = 20;                   // Levenberg-Marquardt cap
  int num_threads = 1;                       // 1 => deterministic registration
  // A cloud with fewer points than this is not registered (too sparse for a
  // stable covariance / correspondence estimate); mirrors GLIM's loose-init
  // 50-point floor.
  int min_points = 50;
};

// Outcome of one registration. `converged` is the gate the caller keys on:
// false means the fit was rejected (too few points, empty target, or
// inlier_fraction below the threshold) and the pose must not be trusted.
struct ScanMatchResult
{
  bool converged = false;
  double inlier_fraction = 0.0;
  Eigen::Isometry3d T_world_lidar = Eigen::Isometry3d::Identity();
};

// Incremental scan-to-map registrator. The target is an iVox of world-frame
// points grown via insert_target(); register_scan() aligns a LiDAR-frame source
// scan to it without mutating the target (the caller decides, from the result's
// gate, whether to then insert_target the accepted scan and chain onward).
class ScanMatchRecoverer
{
public:
  explicit ScanMatchRecoverer(ScanMatchParams params = {});
  ~ScanMatchRecoverer();

  ScanMatchRecoverer(const ScanMatchRecoverer &) = delete;
  ScanMatchRecoverer & operator=(const ScanMatchRecoverer &) = delete;
  ScanMatchRecoverer(ScanMatchRecoverer &&) noexcept;
  ScanMatchRecoverer & operator=(ScanMatchRecoverer &&) noexcept;

  // Add already-optimized world-frame points to the registration target.
  // Points carry per-point covariances estimated from their own neighborhood
  // (as GLIM does before inserting a frame into its iVox). A batch below
  // min_points is ignored.
  void insert_target(const std::vector<Eigen::Vector3d> & world_points);

  [[nodiscard]] bool target_empty() const noexcept;

  // Register `source_lidar_points` (LiDAR frame) against the current target,
  // starting the optimization from `init_T_world_lidar`. Does not modify the
  // target. Returns converged=false (with T_world_lidar left at the init guess)
  // when the target is empty, the source is below min_points, or the fit's
  // inlier fraction is below params.min_inlier_fraction.
  [[nodiscard]] ScanMatchResult register_scan(
    const std::vector<Eigen::Vector3d> & source_lidar_points,
    const Eigen::Isometry3d & init_T_world_lidar) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__SCAN_MATCH_RECOVERY_HPP_
