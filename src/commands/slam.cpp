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
#include "bagwiz/core/logging.hpp"

#include <string_view>

namespace bagwiz::commands
{

namespace
{
constexpr const char * kLogger = "bagwiz.cmd.slam";
}  // namespace

// `bagwiz slam` is a command group for in-process LiDAR SLAM over a rosbag. Its
// sole action today is `run` (estimate a trajectory, and by default an optimized
// point-cloud map, from a single PointCloud2 topic — see run_slam_run for the
// full behavior, IMU mode, and outputs). Modeling it as a group
// (require_subcommand(1)) leaves room for further SLAM tools — e.g. map
// post-processing or trajectory evaluation — without reshaping the CLI, the same
// way `topic` and `tf` group their actions.
class SlamCommand : public Command
{
public:
  [[nodiscard]] std::string_view name() const override { return "slam"; }
  [[nodiscard]] std::string_view description() const override { return "LiDAR SLAM over a rosbag"; }

  void configure(CLI::App & app) override
  {
    app.require_subcommand(1);
    configure_run(app);
  }

  int run() override
  {
    switch (selected_) {
      case Subcommand::kRun:
        return run_slam_run(run_args_);
      case Subcommand::kNone:
        BAGWIZ_LOG_ERROR(kLogger, "no subcommand selected");
        return 1;
    }
    return 1;
  }

private:
  enum class Subcommand { kNone, kRun };
  Subcommand selected_ = Subcommand::kNone;

  SlamRunArgs run_args_;

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
        "output_root", run_args_.output_root,
        "Output root directory; writes traj.tum and, unless --without-global-optim, "
        "map.ply")
      ->required();
    sub->add_option(
      "--imu", run_args_.imu_topic,
      "Optional Imu topic; switches odometry to LiDAR-IMU. The LiDAR<-IMU extrinsic is "
      "resolved from the bag's static TF using the cloud and IMU header frame_ids "
      "(errors if that chain is absent).");
    sub
      ->add_option(
        "--map-resolution", run_args_.map_resolution,
        "Exported map voxel size in meters (smaller = denser; default 0.2). Controls "
        "only the exported map's density, never the optimization or trajectory. The "
        "LiDAR preprocessor's ~0.15 m input voxel bounds the real resolution.")
      ->check(CLI::PositiveNumber);
    sub->add_flag(
      "--without-global-optim", run_args_.without_global_optim,
      "Skip global mapping and write only the raw odometry trajectory (traj.tum); "
      "no point-cloud map is produced");
    sub->add_flag(
      "-w,--overwrite", run_args_.overwrite, "Overwrite the output(s) if they already exist");
    sub->callback([this]() { selected_ = Subcommand::kRun; });
  }
};

BAGWIZ_REGISTER_COMMAND(SlamCommand)

}  // namespace bagwiz::commands
