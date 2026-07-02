// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__MAP_SLAM_HPP_
#define BAGWIZ__COMMANDS__MAP_SLAM_HPP_

#include <filesystem>
#include <string>
#include <vector>

namespace bagwiz::commands
{

// Arguments for `bagwiz map slam`. Populated by MapCommand's CLI wiring
// (src/commands/map.cpp) and consumed by run_map_slam. Kept in a header so the
// run function can be exercised directly from tests without driving the CLI
// parser.
struct MapSlamArgs
{
  std::filesystem::path input_path;
  // PointCloud2 topic to run SLAM on.
  std::string cloud_topic;
  // Optional Imu topic. Empty: LiDAR-only odometry. Set: switches to GLIM's
  // LiDAR-IMU OdometryEstimationCPU, resolving the LiDAR<-IMU extrinsic from the
  // bag's static TF using the cloud's and the IMU's header frame_ids.
  std::string imu_topic;
  // Optional NavSatFix topic. Empty: no GNSS. Set: adds GNSS global constraints
  // (horizontal translation priors on submap poses) during global mapping,
  // pinning the world frame to GNSS and curbing drift. The fixes are projected
  // to a local ENU frame internally. The antenna lever-arm (T_cloud_gnss) is
  // resolved from the bag's static TF so the prior constrains the sensor origin,
  // not the antenna; if that TF is absent the run still proceeds (warned) using
  // the raw antenna position.
  std::string gnss_topic;
  // Output root directory; receives traj.tum and map.pcd.
  std::filesystem::path output_root;
  // Exported-map voxel size in meters. Controls only the exported map's density,
  // never the optimization or trajectory.
  double map_resolution = 0.2;
  // Overwrite the output file(s) if they already exist.
  bool overwrite = false;
  // After writing map.pcd, serve it over a loopback HTTP server and open the
  // default browser to a Three.js viewer. Blocks until interrupted (Ctrl-C).
  bool viewer = false;
  // Disable the live progress bar. The bar is also auto-suppressed when stderr
  // is not a TTY or NO_COLOR is set, so this flag is only needed to silence it
  // on an interactive terminal.
  bool no_progress = false;
  // Optional trajectory up-sampling spec; affects traj.tum ONLY, never the map.
  // Empty: disabled (output unchanged). Otherwise a positive magnitude with an
  // optional, case-insensitive suffix: 'x'/'X' = multiple of the trajectory's
  // native rate (e.g. "2x"); 'hz'/'HZ'/'Hz' or no suffix = absolute frequency in
  // Hz (e.g. "20" or "20hz"). Parsed by core::parse_upsample_spec; resampling
  // interpolates position linearly and orientation by SLERP within the original
  // time span only (no extrapolation). A target at or below the native rate
  // leaves the trajectory unchanged (warned, never down-sampled).
  std::string upsample_traj;

  // Recover poses for the initialization ("start") window (LiDAR-IMU only;
  // automatically disabled without --imu). GLIM's LiDAR-IMU odometry emits no
  // frame until IMU init converges (~1 s), leaving traj.tum without samples over
  // its opening window. When enabled, the pre-init IMU + scan stamps are buffered
  // and, once the first frame's converged state is known, the IMU is integrated
  // backward from it to recover per-scan poses for that window, re-anchored onto
  // the optimized map.
  bool recover_start = true;

  // Recover poses for the cooldown ("end") window (LiDAR-IMU only; automatically
  // disabled without --imu) — the symmetric counterpart of recover_start. The
  // newest scans stay inside the odometry smoother window at end-of-sequence, so
  // traj.tum otherwise stops one window short of the last input scan. When
  // enabled, a trailing ring of IMU + scan stamps is buffered and the IMU is
  // integrated forward from the last estimated frame to recover per-scan poses for
  // those trailing scans, re-anchored onto the optimized map.
  bool recover_end = true;

  // Number of CPU threads for GLIM. 0 or a negative value falls back to the
  // default (4).
  int num_threads = 4;

  // SLAM backend selection: "auto" (default), "cpu", or "gpu".
  //  - auto: use the CUDA GPU backend when this binary was built with
  //    -DBAGWIZ_WITH_SLAM_CUDA AND a CUDA device is visible; otherwise CPU.
  //  - cpu: force the CPU backend (the reproducibility-guaranteed path).
  //  - gpu: force the CUDA GPU backend; error on a non-CUDA build or no device.
  // The GPU backend = GLIM's GPU LiDAR-IMU odometry (with --imu; CT without it,
  // since GLIM has no GPU LiDAR-only backend), GPU VGICP registration in
  // sub/global mapping, and GPU export-map voxelization. It is OUTSIDE the CPU
  // reproducibility guarantee. The effective choice is resolved in run_map_slam.
  std::string backend = "auto";
};

// Run LiDAR SLAM over a single PointCloud2 topic entirely in-process: bagwiz
// reads + decodes the bag and feeds GLIM's modules directly, with no ROS node /
// pub-sub. By default the marginalized frames flow through GLIM's
// SubMapping -> GlobalMapping so the output is the globally-optimized 6-DoF
// trajectory (traj.tum) plus an optimized world-frame point-cloud map (map.pcd),
// both under args.output_root. With args.imu_topic set, odometry switches to
// GLIM's LiDAR-IMU OdometryEstimationCPU.
//
// Returns a process exit code: 0 on success, 1 on any error (input open
// failure, topic/type mismatch, an absent LiDAR<-IMU static-TF chain, output
// collision, or a read/write error).
int run_map_slam(const MapSlamArgs & args);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__MAP_SLAM_HPP_
