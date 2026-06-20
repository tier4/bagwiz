// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/map_viewer.hpp"

#include "bagwiz/core/logging.hpp"
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
#include <string>
#include <system_error>
#include <vector>

namespace bagwiz::core::slam
{
namespace
{
constexpr const char * kLogger = "bagwiz.cmd.slam";
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

  server.Get("/map.ply", [map_path](const httplib::Request &, httplib::Response & res) {
    std::error_code ec;
    const auto size = std::filesystem::file_size(map_path, ec);
    if (ec) {
      res.status = 404;
      res.set_content("map.ply not found", "text/plain");
      return;
    }
    auto stream = std::make_shared<std::ifstream>(map_path, std::ios::binary);
    if (!stream->good()) {
      res.status = 500;
      res.set_content("could not open map.ply", "text/plain");
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

  fmt::print(stdout, "Serving map viewer at {} (press Ctrl-C to stop)\n", url);
  std::fflush(stdout);

  // Open the browser as a best-effort step; a failure is non-fatal because the
  // URL is printed above for manual use.
  if (std::system(browser_open_command(url).c_str()) != 0) {
    fmt::print(stdout, "Could not auto-open a browser; open {} manually.\n", url);
    std::fflush(stdout);
  }

  g_viewer_server.store(&server);
  auto * const previous = std::signal(SIGINT, on_viewer_sigint);
  server.listen_after_bind();  // blocks until on_viewer_sigint() calls stop()
  std::signal(SIGINT, previous);
  g_viewer_server.store(nullptr);

  fmt::print(stdout, "Map viewer stopped.\n");
  return 0;
}

}  // namespace bagwiz::core::slam
