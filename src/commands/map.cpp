// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/commands/map_slam.hpp"
#include "bagwiz/commands/map_viewer.hpp"
#include "bagwiz/core/logging.hpp"

#include <string_view>
#include <thread>

namespace bagwiz::commands
{

namespace
{
constexpr const char * kLogger = "bagwiz.cmd.map";
}  // namespace

// `bagwiz map` is a command group for map-producing and map-processing tools.
// Its actions are:
//   slam    - in-process LiDAR SLAM over a rosbag (estimate trajectory + map)
//   viewer  - open the browser map viewer for an existing map.pcd
class MapCommand : public Command
{
public:
  [[nodiscard]] std::string_view name() const override { return "map"; }
  [[nodiscard]] std::string_view description() const override
  {
    return "LiDAR map generation and viewing";
  }

  void configure(CLI::App & app) override
  {
    app.require_subcommand(1);
    configure_slam(app);
    configure_viewer(app);
  }

  int run() override
  {
    switch (selected_) {
      case Subcommand::kSlam:
        return run_map_slam(slam_args_);
      case Subcommand::kViewer:
        return run_map_viewer(viewer_args_);
      case Subcommand::kNone:
        BAGWIZ_LOG_ERROR(kLogger, "no subcommand selected");
        return 1;
    }
    return 1;
  }

private:
  enum class Subcommand { kNone, kSlam, kViewer };
  Subcommand selected_ = Subcommand::kNone;

  MapSlamArgs slam_args_;
  MapViewerArgs viewer_args_;

  void configure_slam(CLI::App & app)
  {
    auto * sub = app.add_subcommand(
      "slam", "Estimate a trajectory from a LiDAR PointCloud2 topic (GLIM, in-process)");
    sub->add_option("input", slam_args_.input_path, "Bag path (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    sub->add_option("pcd_topic", slam_args_.cloud_topic, "PointCloud2 topic to run SLAM on")
      ->required();
    sub
      ->add_option(
        "output_root", slam_args_.output_root, "Output root directory; writes traj.tum and map.pcd")
      ->required();
    sub->add_option(
      "--imu", slam_args_.imu_topic,
      "Optional Imu topic; switches odometry to LiDAR-IMU. The LiDAR<-IMU extrinsic is "
      "resolved from the bag's static TF using the cloud and IMU header frame_ids "
      "(errors if that chain is absent).");
    sub->add_option(
      "--gnss", slam_args_.gnss_topic,
      "Optional NavSatFix topic; adds GNSS global constraints during global mapping "
      "(horizontal translation priors on submap poses) to pin the world frame to GNSS "
      "and curb drift. The antenna lever-arm is resolved from the bag's static TF (cloud "
      "<- NavSatFix frame_id) and removed (a missing TF only warns). Each prior is "
      "weighted by the fix's reported position covariance (falling back to a fixed "
      "precision when unknown).");
    auto * cam_opt = sub->add_option(
      "--cam", slam_args_.image_topics,
      "Camera image topic(s) (sensor_msgs/msg/Image or CompressedImage); list several "
      "after one flag and/or repeat the flag. After the global optimization, map points "
      "are colorized by splatting them into each camera's images against a per-pixel "
      "depth buffer, and map.pcd gains an rgb field. Each point's color is a robust "
      "weighted average over its observations (the weights favor close, frontal, sharp, "
      "non-border views, and the lit-mode cluster wins over shadows) with per-image "
      "exposure/gain compensation; map points sitting behind a vehicle or pedestrian "
      "caught by the LiDAR scan nearest an image are skipped for that image, so moving "
      "traffic does not stain the map. A point seen by several cameras gets a weighted "
      "blend of them, gain-aligned to the first listed "
      "topic. Points no image observed inherit the nearest observed neighbor's color "
      "(see --no-color-propagate) or keep a neutral gray. Intrinsics come from each "
      "camera's CameraInfo topic (see --cam-info); each camera extrinsic is resolved "
      "from the bag's static TF (cloud <- camera frame), erroring if that chain is "
      "absent. Images are assumed raw (unrectified): the CameraInfo distortion model "
      "is applied during projection.");
    sub
      ->add_option(
        "--cam-info", slam_args_.camera_info_topics,
        "Explicit CameraInfo topic(s) for --cam: either omit entirely (auto-resolve every "
        "camera from its image topic name using the standard suffix rules) or pass exactly "
        "one per --cam topic, in the same order (several after one flag and/or repeated).")
      ->needs(cam_opt);
    sub->add_flag(
      "!--no-color-propagate", slam_args_.color_propagate,
      "Do not propagate colors to map points no camera observed. By default, unobserved "
      "points inherit the color of the nearest observed neighbor within an automatic "
      "radius (4x the median point spacing, clamped to [0.05, 5] m), so map.pcd is "
      "fully colored; with this flag they keep a neutral gray instead.");
    sub->add_option(
      "--frame", slam_args_.output_frame,
      "Output trajectory frame. Defaults to the PointCloud2 topic's frame_id; a "
      "different value is resolved through the bag's static TF and the trajectory is "
      "transformed so each pose expresses the requested frame in the SLAM world.");
    sub
      ->add_option(
        "--input-res", slam_args_.input_resolution,
        "Voxel size in meters (default 0.15) for BOTH the LiDAR input downsample and the "
        "exported-map merge — the single map-resolution knob. Smaller = denser map and finer "
        "SLAM detail, at more points and runtime. Unlike a pure export voxel it feeds the "
        "optimizer, so it also changes the trajectory. The range crop still bounds which returns "
        "enter the pipeline.")
      ->check(CLI::PositiveNumber);
    sub
      ->add_option(
        "--min-range", slam_args_.range_min,
        "Discard LiDAR returns closer than this many meters before SLAM (default 1.0). Points "
        "dropped here never enter the trajectory or the map. Must be > 0 and < --max-range.")
      ->check(CLI::PositiveNumber);
    sub
      ->add_option(
        "--max-range", slam_args_.range_max,
        "Discard LiDAR returns farther than this many meters before SLAM (default 100.0). Points "
        "dropped here never enter the trajectory or the map. Must be > --min-range.")
      ->check(CLI::PositiveNumber);
    sub
      ->add_option(
        "--fill-min-inliers", slam_args_.fill_min_inlier_fraction,
        "Inlier-fraction acceptance gate (0..1, default 0.7) for warmup/cooldown pose-fill "
        "scan-matching. Higher = stricter (endpoints may stay unfilled); lower = looser (a bad "
        "fit can degrade the fill). No effect when both fills are disabled.")
      ->check(CLI::Range(0.0, 1.0));
    sub
      ->add_option(
        "--submap-keyframes", slam_args_.submap_max_keyframes,
        "Keyframes per GLIM submap before it is finalized (default 15). Smaller = more, smaller "
        "submaps: finer loop-closure granularity and more GNSS-covered submaps (can unblock GNSS "
        "priors on short runs), but super-linearly more sub-mapping work per submap and thinner, "
        "weaker submaps. Larger = fewer, larger submaps: a cheaper global graph but coarser "
        "correction. Feeds the optimizer, so it also changes the trajectory.")
      ->check(CLI::PositiveNumber);
    sub->add_flag(
      "-w,--overwrite", slam_args_.overwrite, "Overwrite the output file(s) if they already exist");
    sub->add_flag(
      "--viewer", slam_args_.viewer,
      "After writing map.pcd, open the default browser to a Three.js point-cloud viewer "
      "served over a loopback HTTP server. Runs until interrupted (Ctrl-C).");
    sub->add_flag(
      "--no-progress", slam_args_.no_progress,
      "Disable the live progress bars. They are also auto-suppressed when stderr is not a "
      "terminal or NO_COLOR is set, so this is only needed to silence them interactively.");

    sub->add_flag(
      "!--no-warmup-fill", slam_args_.fill_start,
      "Disable initialization-window ('start') pose fill (default on). GLIM's odometry emits "
      "no "
      "pose over its opening window (the LiDAR-IMU init, ~1 s), so traj.tum otherwise has no "
      "samples there. By default those pre-init scans are buffered and filled in by scan-matching "
      "each against the optimized map (so it works in LiDAR-only mode too); with --imu the "
      "buffered "
      "IMU additionally seeds each registration's initial guess and is the fallback if a "
      "registration fails. Affects traj.tum's opening window only.");
    sub->add_flag(
      "!--no-cooldown-fill", slam_args_.fill_end,
      "Disable cooldown-window ('end') pose fill (default on) — the symmetric counterpart of "
      "--no-warmup-fill. The newest scans stay inside the odometry smoother window at "
      "end-of-sequence, so traj.tum otherwise stops one window short of the last input scan. By "
      "default those trailing scans are buffered and filled in by scan-matching each against the "
      "optimized map (LiDAR-only included); with --imu the buffered IMU additionally seeds each "
      "initial guess and is the fallback. Affects traj.tum's closing window only.");
    const int max_threads = static_cast<int>(std::thread::hardware_concurrency());
    sub
      ->add_option(
        "-j,--threads", slam_args_.num_threads,
        "Number of CPU threads for GLIM and trajectory endpoint fill (default: 4). The "
        "host's hardware concurrency is the effective maximum.")
      ->check(CLI::Range(0, max_threads > 0 ? max_threads : 256));
    sub
      ->add_option(
        "--backend", slam_args_.backend,
        "SLAM backend (default 'auto'). 'auto' uses the CUDA GPU backend when this binary was "
        "built with -DBAGWIZ_WITH_SLAM_CUDA (pixi run -e humble-cuda build-full) AND a CUDA device "
        "is "
        "visible, else CPU. 'cuda' forces it (errors on a non-CUDA build / no device). 'cpu' "
        "forces the CPU backend (the reproducibility-guaranteed path). The CUDA backend = GPU "
        "LiDAR-IMU odometry with --imu (CT without it), GPU VGICP mapping, and GPU export "
        "voxelization; it is OUTSIDE the CPU reproducibility guarantee.")
      ->check(CLI::IsMember({"auto", "cpu", "cuda"}));
    sub->callback([this]() { selected_ = Subcommand::kSlam; });
  }

  void configure_viewer(CLI::App & app)
  {
    auto * sub = app.add_subcommand(
      "viewer", "Open the browser map viewer for an existing map.pcd (no SLAM run)");
    sub
      ->add_option(
        "map", viewer_args_.map_path,
        "Path to a map.pcd file, or a directory containing map.pcd (e.g. a map slam "
        "output root). Served over a loopback HTTP server with the Three.js viewer; "
        "runs until interrupted (Ctrl-C).")
      ->required()
      ->check(CLI::ExistingPath);
    sub->callback([this]() { selected_ = Subcommand::kViewer; });
  }
};

BAGWIZ_REGISTER_COMMAND(MapCommand)

}  // namespace bagwiz::commands
