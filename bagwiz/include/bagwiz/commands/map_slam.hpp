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
  // Optional camera image topics (sensor_msgs/msg/Image or CompressedImage).
  // Empty: no colorization. Set: after the global optimization, the map points
  // are colorized by splatting them into each camera's images and map.pcd
  // gains an rgb field. Each point's color is a robust weighted average over
  // its observations; a point observed by several cameras gets a weighted
  // blend of them after per-camera gain alignment against the FIRST listed
  // topic. Points no image observed inherit the color of the nearest observed
  // neighbor (see color_propagate). Intrinsics come from each camera's
  // CameraInfo topic (camera_info_topics, or auto-resolved from the image
  // topic name); each camera extrinsic is resolved from the bag's static TF
  // (cloud frame <- CameraInfo frame_id) and its absence is an error. Images
  // are assumed RAW (unrectified): the CameraInfo distortion model is applied
  // during projection.
  std::vector<std::string> image_topics;
  // Explicit CameraInfo topics for image_topics. Either empty (auto-resolve
  // every camera from its image topic name using the standard suffix rules)
  // or exactly one entry per image topic, in the same order.
  std::vector<std::string> camera_info_topics;
  // Keyframe thinning gate for the colorize pass, in meters. 0 (the default)
  // feeds every decoded image to the colorizer; > 0 feeds an image only when
  // the interpolated body pose moved at least this far — or rotated at least
  // kKeyframeMinRotationDeg degrees — since the image that opened the current
  // bucket on that topic. Consecutive vehicle-camera frames are
  // near-duplicates (a stopped platform contributes hundreds of identical
  // frames), so thinning cuts colorize cost roughly linearly with the frames
  // dropped while the per-point observation reservoirs stay well fed.
  // Deterministic. Has no effect without image_topics.
  double cam_min_dist = 0.0;
  // Blur refinement of the keyframe gate (requires cam_min_dist > 0): instead
  // of keeping the FIRST image of each gate bucket, decode and score every
  // image in the bucket (mean Sobel gradient magnitude) and keep the
  // sharpest, so motion-blurred frames are dropped rather than merely
  // down-weighted. Costs a decode per candidate image; the rasterize/sweep
  // work is still saved. Deterministic.
  bool cam_keyframe_blur = false;
  // Fill map points no camera observed with the color of the nearest observed
  // neighbor within an automatic radius (4x the median local point spacing,
  // clamped to [0.05, 5] m), so map.pcd comes out fully colored. Disabled by
  // --no-color-propagate, in which case unobserved points keep a neutral
  // gray. Has no effect without image_topics.
  bool color_propagate = true;
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
  // Inlier-fraction acceptance gate (0..1) for warmup/cooldown pose-fill
  // scan-matching. Higher = stricter (endpoints may stay unfilled); lower =
  // looser (a bad fit can degrade the fill). No effect when both fills are off.
  // Default 0.7 matches the fill scan-matcher's loose-init gate.
  double fill_min_inlier_fraction = 0.7;
  // Keyframes accumulated before GLIM finalizes a submap (GLIM max_num_keyframes).
  // Smaller = more, smaller submaps (finer loop-closure granularity, more
  // GNSS-covered submaps) at super-linearly more sub-mapping cost; larger = fewer,
  // larger submaps (cheaper global graph, coarser correction). Default 15 == GLIM
  // stock, so the default trajectory is unchanged. Must be > 0.
  int submap_max_keyframes = 15;
  // Post-process radius outlier removal on the finished map, applied right
  // before colorization and export: when enabled, a map point is dropped when
  // fewer than outlier_min_neighbors other map points lie within
  // outlier_radius meters. Off by default, so the exported map is unchanged
  // unless requested. Filters the map only; the trajectory is untouched.
  bool remove_outliers = false;
  // Neighborhood radius in meters for remove_outliers. Tune together with
  // input_resolution: the exported map is voxel-merged at that resolution, so
  // the radius should span a few voxels (default 0.5 ~ 3 voxels at the stock
  // 0.15 resolution).
  double outlier_radius = 0.5;
  // Minimum number of OTHER map points within outlier_radius a point needs to
  // survive remove_outliers.
  int outlier_min_neighbors = 5;
  // Remove ghost points left by moving objects from the exported map (DUFOMap-
  // style void-region ray casting): after the global optimization, every scan's
  // rays mark the voxels they traverse as seen-free, and a scan point falling in
  // a voxel that was ever seen free is dropped before the map merge. Off by
  // default. Filters the map only; the trajectory is untouched.
  bool remove_dynamic = false;
  // Voxel side in meters of the free-space grid for remove_dynamic. Independent
  // of input_resolution: coarser costs less memory and absorbs more pose noise;
  // finer separates ghosts closer to static surfaces.
  double dynamic_resolution = 0.2;
  // d_s in meters for remove_dynamic: each ray stops this far short of its hit
  // so range noise cannot mark the hit surface's neighborhood as free.
  double dynamic_sensor_offset = 0.15;
  // d_p in voxels for remove_dynamic: a voxel counts as void only when it and
  // every voxel within this Chebyshev radius were seen free, so pose error up
  // to ~d_p * dynamic_resolution cannot delete static points. 0 disables the
  // guard; higher is more conservative.
  int dynamic_neighborhood = 1;
  // Overwrite the output file(s) if they already exist.
  bool overwrite = false;
  // After writing map.pcd, serve it over a loopback HTTP server and open the
  // default browser to a Three.js viewer. Blocks until interrupted (Ctrl-C).
  bool viewer = false;
  // Disable the live progress bar. The bar is also auto-suppressed when stderr
  // is not a TTY or NO_COLOR is set, so this flag is only needed to silence it
  // on an interactive terminal.
  bool no_progress = false;

  // Fill in poses for the initialization ("start") window. GLIM's odometry emits
  // no frame over its opening window (the LiDAR-IMU init, ~1 s), leaving traj.tum
  // without samples there. When enabled, the pre-init scans are buffered and
  // filled in by scan-matching each against the optimized map (works in LiDAR-only
  // mode too); with --imu the buffered IMU additionally seeds each registration's
  // initial guess and is the fallback when a registration fails its gate.
  bool fill_start = true;

  // Fill in poses for the cooldown ("end") window — the symmetric counterpart of
  // fill_start. The newest scans stay inside the odometry smoother window at
  // end-of-sequence, so traj.tum otherwise stops one window short of the last
  // input scan. When enabled, the trailing scans are buffered and filled in by
  // scan-matching each against the optimized map (LiDAR-only included); with --imu
  // the buffered IMU additionally seeds each initial guess and is the fallback.
  bool fill_end = true;

  // Number of CPU threads for GLIM and trajectory endpoint (warmup/cooldown)
  // fill. 0 resolves to the host's hardware concurrency.
  int num_threads = 8;

  // SLAM backend selection: "auto" (default), "cpu", or "cuda".
  //  - auto: use the CUDA GPU backend when this binary was built with
  //    -DBAGWIZ_WITH_SLAM_CUDA AND a CUDA device is visible; otherwise CPU.
  //  - cpu: force the CPU backend (the reproducibility-guaranteed path).
  //  - cuda: force the CUDA GPU backend; error on a non-CUDA build or no device.
  // The GPU backend = GLIM's GPU LiDAR-IMU odometry (with --imu; CT without it,
  // since GLIM has no GPU LiDAR-only backend), GPU VGICP registration in
  // sub/global mapping, GPU export-map voxelization, and GPU --cam colorization
  // (the CUDA colorize rasterizer). It is OUTSIDE the CPU reproducibility
  // guarantee. The effective choice is resolved in run_map_slam.
  std::string backend = "auto";
};

// Run LiDAR SLAM over a single PointCloud2 topic entirely in-process: bagwiz
// reads + decodes the bag and feeds GLIM's modules directly, with no ROS node /
// pub-sub. By default the marginalized frames flow through GLIM's
// SubMapping -> GlobalMapping so the output is the globally-optimized 6-DoF
// trajectory (traj.tum) plus an optimized world-frame point-cloud map (map.pcd),
// both under args.output_root. With args.imu_topic set, odometry switches to
// GLIM's LiDAR-IMU estimator (OdometryEstimationCPU, or OdometryEstimationGPU
// on the GPU backend — see backend).
//
// Returns a process exit code: 0 on success, 1 on any error (input open
// failure, topic/type mismatch, an absent LiDAR<-IMU static-TF chain, output
// collision, or a read/write error).
int run_map_slam(const MapSlamArgs & args);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__MAP_SLAM_HPP_
