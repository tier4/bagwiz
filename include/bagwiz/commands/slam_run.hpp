// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__SLAM_RUN_HPP_
#define BAGWIZ__COMMANDS__SLAM_RUN_HPP_

#include <filesystem>
#include <string>
#include <vector>

namespace bagwiz::commands
{

// Arguments for `bagwiz slam run`. Populated by SlamCommand's CLI wiring
// (src/commands/slam.cpp) and consumed by run_slam_run. Kept in a header so the
// run function can be exercised directly from tests without driving the CLI
// parser.
struct SlamRunArgs
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
  // Remove dynamic (moving-object) points from the exported map with an
  // original Removert-style filter. Affects only map.pcd contents, never the
  // optimization or trajectory. The remaining removert_* fields tune the filter
  // and are consulted only when this is set.
  bool removert = false;
  // Enable the multi-resolution consensus revert pass.
  bool removert_revert = true;
  // Vertical and horizontal field of view for the Removert range images (deg).
  double removert_vertical_fov_deg = 50.0;
  double removert_horizontal_fov_deg = 360.0;
  // Resolution magnifier lists (pixels per degree) for the remove and revert
  // passes. Processed in order. Defaults use a single resolution each to keep
  // the filter fast; add finer/coarser values for more aggressive cleaning.
  std::vector<double> removert_remove_resolutions = {2.0};
  std::vector<double> removert_revert_resolutions = {1.0};
  // Adaptive discrepancy coefficient and upper-bound for valid pixel differences.
  double removert_adaptive_coeff = 0.05;
  double removert_valid_diff_upper_bound = 200.0;
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
int run_slam_run(const SlamRunArgs & args);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__SLAM_RUN_HPP_
