// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/commands/map_filter.hpp"
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
//   filter  - post-process an existing map (e.g. Removert dynamic-point removal)
class MapCommand : public Command
{
public:
  [[nodiscard]] std::string_view name() const override { return "map"; }
  [[nodiscard]] std::string_view description() const override
  {
    return "LiDAR map generation and filtering";
  }

  void configure(CLI::App & app) override
  {
    app.require_subcommand(1);
    configure_slam(app);
    configure_viewer(app);
    configure_filter(app);
  }

  int run() override
  {
    switch (selected_) {
      case Subcommand::kSlam:
        return run_map_slam(slam_args_);
      case Subcommand::kViewer:
        return run_map_viewer(viewer_args_);
      case Subcommand::kFilterRemovert:
        return run_map_filter_removert(filter_removert_args_);
      case Subcommand::kNone:
        BAGWIZ_LOG_ERROR(kLogger, "no subcommand selected");
        return 1;
    }
    return 1;
  }

private:
  enum class Subcommand { kNone, kSlam, kViewer, kFilterRemovert };
  Subcommand selected_ = Subcommand::kNone;

  MapSlamArgs slam_args_;
  MapViewerArgs viewer_args_;
  MapFilterRemovertArgs filter_removert_args_;

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
      "precision when unknown). Requires global mapping.");
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
        "--recovery-min-inliers", slam_args_.recovery_min_inlier_fraction,
        "Inlier-fraction acceptance gate (0..1, default 0.7) for warmup/cooldown pose-recovery "
        "scan-matching. Higher = stricter (endpoints may stay unrecovered); lower = looser (a bad "
        "fit can degrade recovery). No effect when both recoveries are disabled.")
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
      "!--no-warmup-recovery", slam_args_.recover_start,
      "Disable initialization-window ('start') pose recovery (default on). GLIM's odometry emits "
      "no "
      "pose over its opening window (the LiDAR-IMU init, ~1 s), so traj.tum otherwise has no "
      "samples there. By default those pre-init scans are buffered and recovered by scan-matching "
      "each against the optimized map (so it works in LiDAR-only mode too); with --imu the "
      "buffered "
      "IMU additionally seeds each registration's initial guess and is the fallback if a "
      "registration fails. Affects traj.tum's opening window only.");
    sub->add_flag(
      "!--no-cooldown-recovery", slam_args_.recover_end,
      "Disable cooldown-window ('end') pose recovery (default on) — the symmetric counterpart of "
      "--no-warmup-recovery. The newest scans stay inside the odometry smoother window at "
      "end-of-sequence, so traj.tum otherwise stops one window short of the last input scan. By "
      "default those trailing scans are buffered and recovered by scan-matching each against the "
      "optimized map (LiDAR-only included); with --imu the buffered IMU additionally seeds each "
      "initial guess and is the fallback. Affects traj.tum's closing window only.");
    const int max_threads = static_cast<int>(std::thread::hardware_concurrency());
    sub
      ->add_option(
        "-t,--threads", slam_args_.num_threads,
        "Number of CPU threads for GLIM (default: 4). The host's hardware concurrency is the "
        "effective maximum.")
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

  void configure_filter(CLI::App & app)
  {
    auto * group = app.add_subcommand("filter", "Post-process an existing point-cloud map");
    group->require_subcommand(1);
    configure_filter_removert(*group);
  }

  void configure_filter_removert(CLI::App & group)
  {
    auto * sub = group.add_subcommand(
      "removert",
      "Remove dynamic (moving-object) points from a map using an original Removert-style "
      "filter. The optimized trajectory is used to reproject each raw scan into the world "
      "frame so map points can be classified against the scan views.");
    sub
      ->add_option(
        "map", filter_removert_args_.map_path,
        "Map to filter: a map.pcd file or a directory containing map.pcd")
      ->required()
      ->check(CLI::ExistingPath);
    sub->add_option("input", filter_removert_args_.input_path, "Bag path (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    sub
      ->add_option(
        "pcd_topic", filter_removert_args_.cloud_topic,
        "PointCloud2 topic whose scans will be reprojected into the world frame")
      ->required();
    sub
      ->add_option(
        "traj", filter_removert_args_.traj_path,
        "TUM trajectory produced by `bagwiz map slam` (one pose per scan)")
      ->required()
      ->check(CLI::ExistingPath);
    sub
      ->add_option(
        "output", filter_removert_args_.output_path,
        "Output map path (.pcd) or directory (receives map.pcd)")
      ->required();
    sub->add_flag(
      "--revert/--no-revert", filter_removert_args_.enable_revert,
      "Enable the multi-resolution consensus revert pass (default on). Removed points are "
      "re-checked at coarser resolutions and recovered if they are not dynamic at any of them. "
      "Use --no-revert to disable.");
    sub
      ->add_option(
        "--vfov", filter_removert_args_.vertical_fov_deg,
        "Total vertical field of view in degrees for the Removert range image (default 50.0)")
      ->check(CLI::PositiveNumber);
    sub
      ->add_option(
        "--hfov", filter_removert_args_.horizontal_fov_deg,
        "Horizontal field of view in degrees for the Removert range image (default 360.0)")
      ->check(CLI::PositiveNumber);
    sub
      ->add_option(
        "--remove-resolutions", filter_removert_args_.remove_resolutions,
        "Comma-separated magnifier ratios for the remove pass (default 2.0). Processed in "
        "order; each resolution operates on the map left by the previous one.")
      ->delimiter(',')
      ->check(CLI::Range(0.01, 10.0));
    sub
      ->add_option(
        "--revert-resolutions", filter_removert_args_.revert_resolutions,
        "Comma-separated magnifier ratios for the consensus revert pass (default 1.0)")
      ->delimiter(',')
      ->check(CLI::Range(0.01, 10.0));
    sub
      ->add_option(
        "--adaptive-coeff", filter_removert_args_.adaptive_coeff,
        "Adaptive discrepancy coefficient: a map point is dynamic when abs(scan_range - "
        "map_range) > coeff * scan_range (default 0.05)")
      ->check(CLI::PositiveNumber);
    sub
      ->add_option(
        "--valid-diff-max", filter_removert_args_.valid_diff_upper_bound,
        "Upper bound on range difference for a valid pixel comparison (default 200.0). "
        "Pixels with larger differences are treated as no-point pixels.")
      ->check(CLI::PositiveNumber);
    sub->add_flag(
      "-w,--overwrite", filter_removert_args_.overwrite,
      "Overwrite the output file(s) if they already exist");
    sub->add_flag(
      "--no-progress", filter_removert_args_.no_progress,
      "Disable the live progress bar. The bar is also auto-suppressed when stderr is not a "
      "terminal or NO_COLOR is set, so this is only needed to silence them interactively.");
    sub->callback([this]() { selected_ = Subcommand::kFilterRemovert; });
  }
};

BAGWIZ_REGISTER_COMMAND(MapCommand)

}  // namespace bagwiz::commands
