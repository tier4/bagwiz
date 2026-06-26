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
      "--remove-dynamic", run_args_.remove_dynamic,
      "Remove dynamic (moving-object) points from the exported map using a Removert-style "
      "visibility filter: a map point is dropped when enough optimized scans see a farther "
      "surface along its line of sight (the space it occupies was observed as free). Affects "
      "map.pcd only, never the optimization or trajectory.");
    sub
      ->add_option(
        "--dynamic-ratio", run_args_.dynamic_ratio,
        "See-through ratio for --remove-dynamic: drop a map point when this fraction of the "
        "scans looking along its line of sight see a farther surface (default 0.3; higher = "
        "keeps more).")
      ->check(CLI::Range(0.0, 1.0));
    sub
      ->add_option(
        "--dynamic-min-range", run_args_.dynamic_min_range,
        "Minimum range in meters a scan must observe a map point at before it constrains it "
        "(default 1.0); drops ego/near-body returns. Only used with --remove-dynamic.")
      ->check(CLI::PositiveNumber);
    sub
      ->add_option(
        "--dynamic-max-range", run_args_.dynamic_max_range,
        "Maximum range in meters a scan considers a map point at (default 60.0); bounds the "
        "search and ignores far, sparse returns. Only used with --remove-dynamic.")
      ->check(CLI::PositiveNumber);
    sub->add_flag(
      "-w,--overwrite", run_args_.overwrite, "Overwrite the output(s) if they already exist");
    sub->add_flag(
      "--viewer", run_args_.viewer,
      "After writing map.pcd, open the default browser to a Three.js point-cloud viewer "
      "served over a loopback HTTP server. Runs until interrupted (Ctrl-C).");
    sub->add_option(
      "--upsample-traj", run_args_.upsample_traj,
      "Resample the output trajectory (traj.tum only; the map is unaffected) onto a uniform, "
      "denser time grid. Accepts an absolute frequency ('20' or '20hz') or a multiple of the "
      "trajectory's native rate ('2x'). Position is interpolated linearly and orientation "
      "by SLERP, only within the original time span (no extrapolation). A target at or below the "
      "native rate writes the trajectory unchanged (warned; never down-sampled); gaps between "
      "poses wider than a few times the median spacing are left un-interpolated.");
    sub->add_flag(
      "--no-progress", run_args_.no_progress,
      "Disable the live progress bar. It is also auto-suppressed when stderr is not a "
      "terminal or NO_COLOR is set, so this is only needed to silence it interactively.");
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
