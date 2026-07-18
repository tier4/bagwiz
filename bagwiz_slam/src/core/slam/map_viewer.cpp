// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/map_viewer.hpp"

#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/slam/map_viewer_assets.hpp"  // generated: kMapViewerHtml / kMapViewerJs
#include "bagwiz/core/slam/map_viewer_open.hpp"

#include <fmt/core.h>
#include <httplib.h>

#include <atomic>
#include <csignal>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <ios>
#include <memory>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

namespace bagwiz::core::slam
{
namespace
{
constexpr const char * kLogger = "bagwiz.cmd.map.viewer";
constexpr std::size_t kStreamChunkBytes = 1U << 20;  // 1 MiB per content-provider call

// listen() blocks, so a SIGINT handler stops the running server to unblock it.
// Only one viewer server runs per process, so a single namespace-scope pointer
// suffices. std::atomic keeps the handler's load/store well-defined.
std::atomic<httplib::Server *> g_viewer_server{nullptr};

extern "C" void on_viewer_sigint([[maybe_unused]] int signum)
{
  httplib::Server * server = g_viewer_server.load();
  if (server != nullptr) {
    server->stop();
  }
}
}  // namespace

void register_map_viewer_routes(httplib::Server & server, const std::filesystem::path & map_path)
{
  server.Get("/", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(std::string(kMapViewerHtml), "text/html; charset=utf-8");
  });

  server.Get("/map_viewer.js", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(std::string(kMapViewerJs), "text/javascript; charset=utf-8");
  });

  server.Get("/map_colormaps.js", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(std::string(kMapViewerColormaps), "text/javascript; charset=utf-8");
  });

  server.Get("/map_viewer_overlay.js", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(std::string(kMapViewerOverlay), "text/javascript; charset=utf-8");
  });

  server.Get("/map.pcd", [map_path](const httplib::Request &, httplib::Response & res) {
    std::error_code ec;
    const auto size = std::filesystem::file_size(map_path, ec);
    if (ec) {
      res.status = 404;
      res.set_content("map.pcd not found", "text/plain");
      return;
    }
    auto stream = std::make_shared<std::ifstream>(map_path, std::ios::binary);
    if (!stream->good()) {
      res.status = 500;
      res.set_content("could not open map.pcd", "text/plain");
      return;
    }
    // Stream from disk in chunks rather than buffering the (potentially very
    // large) cloud into memory. The captured stream is reused across calls.
    res.set_content_provider(
      static_cast<std::size_t>(size), "application/octet-stream",
      [stream](std::size_t offset, std::size_t length, httplib::DataSink & sink) {
        const std::size_t want = length < kStreamChunkBytes ? length : kStreamChunkBytes;
        std::vector<char> buf(want);
        stream->seekg(static_cast<std::streamoff>(offset));
        stream->read(buf.data(), static_cast<std::streamsize>(buf.size()));
        const auto got = stream->gcount();
        if (got <= 0) {
          return false;  // short read before the declared length: abort the response
        }
        return sink.write(buf.data(), static_cast<std::size_t>(got));
      });
  });

  // TUM trajectories are one short ASCII line per pose, so even a long SLAM run
  // is at most a few tens of MB; unlike /map.pcd this reads the whole file into
  // memory rather than chunk-streaming it. Absent when `map_path` was viewed
  // without a matching `map slam` trajectory (or any other reason its sibling
  // traj.tum does not exist) -- the client treats 404 as "no trajectory to
  // offer" and hides the toggle rather than treating it as an error.
  const std::filesystem::path traj_path = map_path.parent_path() / "traj.tum";
  server.Get("/traj.tum", [traj_path](const httplib::Request &, httplib::Response & res) {
    std::ifstream in(traj_path, std::ios::binary);
    if (!in.good()) {
      res.status = 404;
      res.set_content("traj.tum not found", "text/plain");
      return;
    }
    std::ostringstream buf;
    buf << in.rdbuf();
    res.set_content(buf.str(), "text/plain; charset=utf-8");
  });
}

int serve_map_viewer(const std::filesystem::path & map_path)
{
  httplib::Server server;
  register_map_viewer_routes(server, map_path);

  const int port = server.bind_to_any_port("127.0.0.1");
  if (port < 0) {
    BAGWIZ_LOG_ERROR(kLogger, "could not bind a loopback port for the map viewer");
    return 1;
  }
  const std::string url = fmt::format("http://127.0.0.1:{}/", port);

  BAGWIZ_LOG_INFO(kLogger, "Serving map viewer at %s (press Ctrl-C to stop)", url.c_str());

  // Open the browser as a best-effort step; a failure is non-fatal because the
  // URL is logged above for manual use.
  if (std::system(browser_open_command(url).c_str()) != 0) {
    BAGWIZ_LOG_INFO(kLogger, "Could not auto-open a browser; open %s manually.", url.c_str());
  }

  g_viewer_server.store(&server);
  auto * const previous = std::signal(SIGINT, on_viewer_sigint);
  server.listen_after_bind();  // blocks until on_viewer_sigint() calls stop()
  std::signal(SIGINT, previous);
  g_viewer_server.store(nullptr);

  BAGWIZ_LOG_INFO(kLogger, "Map viewer stopped.");
  return 0;
}

}  // namespace bagwiz::core::slam
