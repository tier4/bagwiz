// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__SLAM_VIS_HPP_
#define BAGWIZ__COMMANDS__SLAM_VIS_HPP_

#include <filesystem>

namespace bagwiz::commands
{

// Arguments for `bagwiz slam vis`. Populated by SlamCommand's CLI wiring
// (src/commands/slam.cpp) and consumed by run_slam_vis.
struct SlamVisArgs
{
  // Path to an existing map.ply file, or a directory containing map.ply (e.g. a
  // `slam run` output root). run_slam_vis resolves the directory form to
  // <dir>/map.ply.
  std::filesystem::path map_path;
};

// Open the same browser viewer as `slam run --vis` for an ALREADY written
// point-cloud map, without re-running SLAM: resolve args.map_path (a .ply file
// or a directory holding map.ply), then serve it over a loopback HTTP server and
// open the host's default browser, blocking until interrupted (Ctrl-C). This is
// the cheap, repeatable way to revisit a map produced by an earlier `slam run`.
//
// Returns a process exit code: 0 on success, 1 on error (map not found, no
// loopback port could be bound, or a binary built without the map viewer).
int run_slam_vis(const SlamVisArgs & args);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__SLAM_VIS_HPP_
