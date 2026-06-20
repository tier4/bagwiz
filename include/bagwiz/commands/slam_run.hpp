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
  // pinning the world frame to GNSS and curbing drift. Requires global mapping,
  // so it is rejected together with without_global_optim. The fixes are projected
  // to a local ENU frame internally. The antenna lever-arm (T_cloud_gnss) is
  // resolved from the bag's static TF so the prior constrains the sensor origin,
  // not the antenna; if that TF is absent the run still proceeds (warned) using
  // the raw antenna position.
  std::string gnss_topic;
  // Output root directory; receives traj.tum and, unless without_global_optim,
  // map.pcd.
  std::filesystem::path output_root;
  // Exported-map voxel size in meters. Controls only the exported map's density,
  // never the optimization or trajectory.
  double map_resolution = 0.2;
  // Overwrite the output file(s) if they already exist.
  bool overwrite = false;
  // Skip global mapping and write only the raw odometry trajectory; no map.
  bool without_global_optim = false;
  // After writing map.pcd, serve it over a loopback HTTP server and open the
  // default browser to a Three.js viewer. Blocks until interrupted (Ctrl-C).
  // Mutually exclusive with without_global_optim (which produces no map).
  bool vis = false;
};

// Run LiDAR SLAM over a single PointCloud2 topic entirely in-process: bagwiz
// reads + decodes the bag and feeds GLIM's modules directly, with no ROS node /
// pub-sub. By default the marginalized frames flow through GLIM's
// SubMapping -> GlobalMapping so the output is the globally-optimized 6-DoF
// trajectory (traj.tum) plus an optimized world-frame point-cloud map (map.pcd),
// both under args.output_root. With args.imu_topic set, odometry switches to
// GLIM's LiDAR-IMU OdometryEstimationCPU. With args.without_global_optim the
// global optimization is skipped and only the raw odometry trajectory (traj.tum)
// is written; no map is produced.
//
// Returns a process exit code: 0 on success, 1 on any error (input open
// failure, topic/type mismatch, an absent LiDAR<-IMU static-TF chain, output
// collision, or a read/write error).
int run_slam_run(const SlamRunArgs & args);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__SLAM_RUN_HPP_
