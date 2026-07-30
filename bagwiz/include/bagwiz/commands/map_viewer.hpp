// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__MAP_VIEWER_HPP_
#define BAGWIZ__COMMANDS__MAP_VIEWER_HPP_

#include <filesystem>

namespace bagwiz::commands
{

// Arguments for `bagwiz map viewer`. Populated by MapCommand's CLI wiring
// (src/commands/map.cpp) and consumed by run_map_viewer.
struct MapViewerArgs
{
  // Path to an existing map.pcd file, or a directory containing map.pcd (e.g. a
  // `map slam` output root). run_map_viewer resolves the directory form to
  // <dir>/map.pcd.
  std::filesystem::path input_path;
};

// Open the same browser viewer as `map slam --viewer` for an ALREADY written
// point-cloud map, without re-running SLAM: resolve args.input_path (a .pcd file
// or a directory holding map.pcd), then serve it over a loopback HTTP server and
// open the host's default browser, blocking until interrupted (Ctrl-C). This is
// the cheap, repeatable way to revisit a map produced by an earlier `map slam`.
//
// Returns a process exit code: 0 on success, 1 on error (map not found, no
// loopback port could be bound, or a binary built without the map viewer).
int run_map_viewer(const MapViewerArgs & args);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__MAP_VIEWER_HPP_
