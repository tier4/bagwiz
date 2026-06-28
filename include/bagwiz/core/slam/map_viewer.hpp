// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__MAP_VIEWER_HPP_
#define BAGWIZ__CORE__SLAM__MAP_VIEWER_HPP_

#include <filesystem>

// Loopback HTTP server backing `bagwiz map slam --viewer`: it serves an embedded
// Three.js viewer page plus the freshly written map.pcd, and opens the default
// browser at it. Built only when BAGWIZ_WITH_MAP_VIEWER is on (which defaults to
// BAGWIZ_WITH_SLAM), so normal builds pull in neither cpp-httplib nor this code.
// httplib is forward-declared so this header stays dependency-light for callers.
namespace httplib
{
class Server;
}

namespace bagwiz::core::slam
{

// Register the viewer routes on `server`:
//   GET /                      -> the embedded Three.js viewer page (text/html)
//   GET /map_viewer.js         -> the embedded viewer module (text/javascript)
//   GET /map_colormaps.js      -> the embedded colormap LUTs (text/javascript)
//   GET /map_viewer_overlay.js -> the embedded gizmo + scale bar (text/javascript)
//   GET /map.pcd               -> the binary PCD at `map_path`, streamed from disk
// Exposed separately from serve_map_viewer so the routing is unit-testable with
// httplib's own client (no browser launch, no blocking listen).
void register_map_viewer_routes(httplib::Server & server, const std::filesystem::path & map_path);

// Serve `map_path` over a loopback-only (127.0.0.1) HTTP server on an
// OS-assigned port, open the host's default browser to it, then block until
// interrupted (SIGINT / Ctrl-C), at which point the server is stopped and this
// returns 0. Returns 1 if no loopback port could be bound. `map_path` must
// already exist (the caller writes map.pcd first).
int serve_map_viewer(const std::filesystem::path & map_path);

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__MAP_VIEWER_HPP_
