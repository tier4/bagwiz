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
  // Frame the output trajectory is expressed in. Empty (the default) keeps the
  // trajectory in the PointCloud2 topic's frame_id. A different value is resolved
  // through the bag's static TF and each output pose is transformed accordingly.
  std::string output_frame;
  // Voxel size in meters used for BOTH the GLIM LiDAR input downsample and the
  // exported-map merge — the single "map resolution" knob. Default 0.15 matches
  // GLIM's stock downsample, so the default trajectory is unchanged. Because it
  // feeds the optimizer it affects the trajectory too, not only the map.
  double input_resolution = 0.15;
  // Range crop in meters applied before downsampling: returns closer than
  // range_min or farther than range_max never enter the trajectory or the map.
  // Defaults match GLIM's stock 1.0 / 100.0. Require 0 < range_min < range_max.
  double range_min = 1.0;
  double range_max = 100.0;
  // Inlier-fraction acceptance gate (0..1) for warmup/cooldown pose-recovery
  // scan-matching. Higher = stricter (endpoints may stay unrecovered); lower =
  // looser (a bad fit can degrade recovery). No effect when both recoveries are
  // off. Default 0.7 matches the recovery scan-matcher's loose-init gate.
  double recovery_min_inlier_fraction = 0.7;
  // Keyframes accumulated before GLIM finalizes a submap (GLIM max_num_keyframes).
  // Smaller = more, smaller submaps (finer loop-closure granularity, more
  // GNSS-covered submaps) at super-linearly more sub-mapping cost; larger = fewer,
  // larger submaps (cheaper global graph, coarser correction). Default 15 == GLIM
  // stock, so the default trajectory is unchanged. Must be > 0.
  int submap_max_keyframes = 15;
  // Overwrite the output file(s) if they already exist.
  bool overwrite = false;
  // After writing map.pcd, serve it over a loopback HTTP server and open the
  // default browser to a Three.js viewer. Blocks until interrupted (Ctrl-C).
  bool viewer = false;
  // Disable the live progress bar. The bar is also auto-suppressed when stderr
  // is not a TTY or NO_COLOR is set, so this flag is only needed to silence it
  // on an interactive terminal.
  bool no_progress = false;

  // Recover poses for the initialization ("start") window. GLIM's odometry emits
  // no frame over its opening window (the LiDAR-IMU init, ~1 s), leaving traj.tum
  // without samples there. When enabled, the pre-init scans are buffered and
  // recovered by scan-matching each against the optimized map (works in LiDAR-only
  // mode too); with --imu the buffered IMU additionally seeds each registration's
  // initial guess and is the fallback when a registration fails its gate.
  bool recover_start = true;

  // Recover poses for the cooldown ("end") window — the symmetric counterpart of
  // recover_start. The newest scans stay inside the odometry smoother window at
  // end-of-sequence, so traj.tum otherwise stops one window short of the last
  // input scan. When enabled, the trailing scans are buffered and recovered by
  // scan-matching each against the optimized map (LiDAR-only included); with --imu
  // the buffered IMU additionally seeds each initial guess and is the fallback.
  bool recover_end = true;

  // Number of CPU threads for GLIM. 0 or a negative value falls back to the
  // default (4).
  int num_threads = 4;

  // SLAM backend selection: "auto" (default), "cpu", or "cuda".
  //  - auto: use the CUDA GPU backend when this binary was built with
  //    -DBAGWIZ_WITH_SLAM_CUDA AND a CUDA device is visible; otherwise CPU.
  //  - cpu: force the CPU backend (the reproducibility-guaranteed path).
  //  - cuda: force the CUDA GPU backend; error on a non-CUDA build or no device.
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
