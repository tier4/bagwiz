// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/slam_vis.hpp"

#include "bagwiz/core/logging.hpp"
#ifdef BAGWIZ_WITH_MAP_VIEWER
#include "bagwiz/core/slam/map_viewer.hpp"
#endif

#include <filesystem>
#include <system_error>

namespace bagwiz::commands
{
namespace
{
constexpr const char * kLogger = "bagwiz.cmd.slam";
constexpr const char * kMapFileName = "map.pcd";
}  // namespace

int run_slam_vis(const SlamVisArgs & args)
{
#ifdef BAGWIZ_WITH_MAP_VIEWER
  std::filesystem::path map_path = args.map_path;
  std::error_code ec;
  // Accept either the map.pcd file directly or the output directory that holds
  // it, so `slam vis <output_root>` mirrors `slam run … <output_root>`.
  if (std::filesystem::is_directory(map_path, ec)) {
    map_path /= kMapFileName;
  }
  if (!std::filesystem::is_regular_file(map_path, ec)) {
    BAGWIZ_LOG_ERROR(kLogger, "Map file not found: %s", map_path.c_str());
    return 1;
  }
  return core::slam::serve_map_viewer(map_path);
#else
  (void)args;
  BAGWIZ_LOG_ERROR(
    kLogger, "slam vis is unavailable: this binary was built without the map viewer");
  return 1;
#endif
}

}  // namespace bagwiz::commands
