// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/commands/slam_run.hpp"
#include "bagwiz/commands/slam_viewer.hpp"
#include "bagwiz/core/logging.hpp"

#include <string_view>

namespace bagwiz::commands
{

namespace
{
constexpr const char * kLogger = "bagwiz.cmd.slam";
}  // namespace

// `bagwiz slam` is a command group for in-process LiDAR SLAM over a rosbag. Its
// actions are `run` (estimate a trajectory, and by default an optimized
// point-cloud map, from a single PointCloud2 topic — see run_slam_run for the
// full behavior, IMU mode, and outputs) and `viewer` (open the browser map
// viewer for an already-written map.pcd without re-running SLAM — see
// run_slam_viewer).
// Modeling it as a group (require_subcommand(1)) leaves room for further SLAM
// tools without reshaping the CLI, the same way `topic` and `tf` group their
// actions.
class SlamCommand : public Command
{
public:
  [[nodiscard]] std::string_view name() const override { return "slam"; }
  [[nodiscard]] std::string_view description() const override { return "LiDAR SLAM over a rosbag"; }

  void configure(CLI::App & app) override
  {
    app.require_subcommand(1);
    configure_run(app);
    configure_viewer(app);
  }

  int run() override
  {
    switch (selected_) {
      case Subcommand::kRun:
        return run_slam_run(run_args_);
      case Subcommand::kViewer:
        return run_slam_viewer(viewer_args_);
      case Subcommand::kNone:
        BAGWIZ_LOG_ERROR(kLogger, "no subcommand selected");
        return 1;
    }
    return 1;
  }

private:
  enum class Subcommand { kNone, kRun, kViewer };
  Subcommand selected_ = Subcommand::kNone;

  SlamRunArgs run_args_;
  SlamViewerArgs viewer_args_;

  void configure_run(CLI::App & app)
  {
    auto * sub = app.add_subcommand(
      "run", "Estimate a trajectory from a LiDAR PointCloud2 topic (GLIM, in-process)");
    sub->add_option("input", run_args_.input_path, "Bag path (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    sub->add_option("pcd_topic", run_args_.cloud_topic, "PointCloud2 topic to run SLAM on")
      ->required();
    sub
      ->add_option(
        "output_root", run_args_.output_root, "Output root directory; writes traj.tum and map.pcd")
      ->required();
    sub->add_option(
      "--imu", run_args_.imu_topic,
      "Optional Imu topic; switches odometry to LiDAR-IMU. The LiDAR<-IMU extrinsic is "
      "resolved from the bag's static TF using the cloud and IMU header frame_ids "
      "(errors if that chain is absent).");
    sub->add_option(
      "--gnss", run_args_.gnss_topic,
      "Optional NavSatFix topic; adds GNSS global constraints during global mapping "
      "(horizontal translation priors on submap poses) to pin the world frame to GNSS "
      "and curb drift. Fixes are projected to a local ENU frame internally; the antenna "
      "lever-arm is resolved from the bag's static TF (cloud <- NavSatFix frame_id) and "
      "removed (a missing TF only warns). Each prior is weighted by the fix's reported "
      "position covariance (falling back to a fixed precision when unknown). Requires "
      "global mapping.");
    sub
      ->add_option(
        "--map-resolution", run_args_.map_resolution,
        "Exported map voxel size in meters (smaller = denser; default 0.2). Controls "
        "only the exported map's density, never the optimization or trajectory. The "
        "LiDAR preprocessor's ~0.15 m input voxel bounds the real resolution.")
      ->check(CLI::PositiveNumber);
    sub->add_flag(
      "--removert", run_args_.removert,
      "Remove dynamic (moving-object) points from the exported map using an original "
      "Removert-style filter. Each optimized scan and the merged map are projected into dense "
      "range images using the configured FOV; a map point is dynamic when "
      "abs(scan_range - map_range) exceeds the adaptive coefficient times scan_range (and is "
      "below --removert-valid-diff-max). Affects map.pcd only, never the optimization or "
      "trajectory.");
    sub->add_flag(
      "--removert-revert", run_args_.removert_revert,
      "Enable the multi-resolution consensus revert pass (default on). Removed points are "
      "re-checked at coarser resolutions and recovered if they are not dynamic at any of them. "
      "Use --removert-revert=false to disable. Only used with --removert.");
    sub
      ->add_option(
        "--removert-vfov", run_args_.removert_vertical_fov_deg,
        "Vertical field of view in degrees for the Removert range image (default 50.0). "
        "Only used with --removert.")
      ->check(CLI::PositiveNumber);
    sub
      ->add_option(
        "--removert-hfov", run_args_.removert_horizontal_fov_deg,
        "Horizontal field of view in degrees for the Removert range image (default 360.0). "
        "Only used with --removert.")
      ->check(CLI::PositiveNumber);
    sub
      ->add_option(
        "--removert-remove-resolutions", run_args_.removert_remove_resolutions,
        "Comma-separated magnifier ratios for the Removert remove pass "
        "(default 2.0). Processed in order; each resolution operates on the map left by "
        "the previous one. Each value must be between 0.01 and 10.0 pixels per degree. "
        "Only used with --removert.")
      ->delimiter(',')
      ->check(CLI::Range(0.01, 10.0));
    sub
      ->add_option(
        "--removert-revert-resolutions", run_args_.removert_revert_resolutions,
        "Comma-separated magnifier ratios for the Removert consensus revert pass "
        "(default 1.0). Each value must be between 0.01 and 10.0 pixels per degree. "
        "Only used with --removert.")
      ->delimiter(',')
      ->check(CLI::Range(0.01, 10.0));
    sub
      ->add_option(
        "--removert-adaptive-coeff", run_args_.removert_adaptive_coeff,
        "Adaptive discrepancy coefficient for Removert: a map point is dynamic when "
        "abs(scan_range - map_range) > coeff * scan_range (default 0.05). "
        "Only used with --removert.")
      ->check(CLI::PositiveNumber);
    sub
      ->add_option(
        "--removert-valid-diff-max", run_args_.removert_valid_diff_upper_bound,
        "Upper bound on range difference for a valid pixel comparison in Removert "
        "(default 200.0). Pixels with larger differences are treated as no-point pixels. "
        "Only used with --removert.")
      ->check(CLI::PositiveNumber);
    sub->add_flag(
      "-w,--overwrite", run_args_.overwrite, "Overwrite the output(s) if they already exist");
    sub->add_flag(
      "--viewer", run_args_.viewer,
      "After writing map.pcd, open the default browser to a Three.js point-cloud viewer "
      "served over a loopback HTTP server. Runs until interrupted (Ctrl-C).");
    sub->add_option(
      "--upsample-traj", run_args_.upsample_traj,
      "Densify traj.tum only (the map is unaffected) to a higher rate. Every original pose is "
      "kept verbatim and interpolated samples are inserted between consecutive poses. Accepts "
      "an absolute frequency ('20' or '20hz') or a multiple of the trajectory's native rate "
      "('2x'). Position is interpolated linearly and orientation by SLERP, only within the "
      "original time span (no extrapolation). A target at or below the native rate writes the "
      "trajectory unchanged (warned; never down-sampled); gaps between poses wider than a few "
      "times the median spacing are left un-interpolated.");
    sub->add_flag(
      "--no-progress", run_args_.no_progress,
      "Disable the live progress bars. They are also auto-suppressed when stderr is not a "
      "terminal or NO_COLOR is set, so this is only needed to silence them interactively.");
    sub->callback([this]() { selected_ = Subcommand::kRun; });
  }

  void configure_viewer(CLI::App & app)
  {
    auto * sub = app.add_subcommand(
      "viewer", "Open the browser map viewer for an existing map.pcd (no SLAM run)");
    sub
      ->add_option(
        "map", viewer_args_.map_path,
        "Path to a map.pcd file, or a directory containing map.pcd (e.g. a slam run "
        "output root). Served over a loopback HTTP server with the Three.js viewer; "
        "runs until interrupted (Ctrl-C).")
      ->required()
      ->check(CLI::ExistingPath);
    sub->callback([this]() { selected_ = Subcommand::kViewer; });
  }
};

BAGWIZ_REGISTER_COMMAND(SlamCommand)

}  // namespace bagwiz::commands
