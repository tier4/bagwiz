// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/map_viewer.hpp"
#include "bagwiz/core/slam/point_cloud_io.hpp"

#include <gtest/gtest.h>
#include <httplib.h>

#include <array>
#include <chrono>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <ios>
#include <string>
#include <thread>
#include <vector>

namespace
{
namespace slam = bagwiz::core::slam;

// All 256 byte values, so an exact echo proves the PCD is streamed verbatim
// (binary-safe, embedded NULs included) rather than treated as text.
std::string all_byte_values()
{
  std::string bytes;
  bytes.reserve(256);
  for (int i = 0; i < 256; ++i) {
    bytes.push_back(static_cast<char>(i));
  }
  return bytes;
}

class MapViewerServer : public ::testing::Test
{
protected:
  void SetUp() override
  {
    map_bytes_ = all_byte_values();
    map_path_ = std::filesystem::temp_directory_path() / "bagwiz_map_viewer_server_test.pcd";
    std::ofstream out(map_path_, std::ios::binary);
    out.write(map_bytes_.data(), static_cast<std::streamsize>(map_bytes_.size()));
    out.close();

    slam::register_map_viewer_routes(server_, map_path_);
    port_ = server_.bind_to_any_port("127.0.0.1");
    ASSERT_GE(port_, 0);
    server_thread_ = std::thread([this] { server_.listen_after_bind(); });
    for (int i = 0; i < 200 && !server_.is_running(); ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ASSERT_TRUE(server_.is_running());
  }

  void TearDown() override
  {
    server_.stop();
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
    std::error_code ec;
    std::filesystem::remove(map_path_, ec);
  }

  std::string map_bytes_;
  std::filesystem::path map_path_;
  httplib::Server server_;
  int port_ = -1;
  std::thread server_thread_;
};

TEST_F(MapViewerServer, ServesViewerPageAtRoot)
{
  httplib::Client client("127.0.0.1", port_);
  const auto res = client.Get("/");
  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 200);
  EXPECT_NE(res->body.find("map_viewer.js"), std::string::npos);
  // The scale-bar element must be present for the overlay script to populate it.
  EXPECT_NE(res->body.find("scaleBarLine"), std::string::npos);
}

TEST_F(MapViewerServer, ServesViewerModule)
{
  httplib::Client client("127.0.0.1", port_);
  const auto res = client.Get("/map_viewer.js");
  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 200);
  EXPECT_NE(res->body.find("PCDLoader"), std::string::npos);
  // The viewer module pulls the overlay widgets from a sibling module, so the
  // compiled JS must reference it (and the server must serve it; see below).
  EXPECT_NE(res->body.find("map_viewer_overlay.js"), std::string::npos);
}

TEST_F(MapViewerServer, ServesColormapsModule)
{
  httplib::Client client("127.0.0.1", port_);
  const auto res = client.Get("/map_colormaps.js");
  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 200);
  EXPECT_NE(res->body.find("sampleColormap"), std::string::npos);
}

TEST_F(MapViewerServer, ServesOverlayModule)
{
  httplib::Client client("127.0.0.1", port_);
  const auto res = client.Get("/map_viewer_overlay.js");
  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 200);
  // The orientation gizmo is built on three.js's ViewHelper, so its identifier
  // proves the compiled overlay (gizmo + scale bar) is embedded and served.
  EXPECT_NE(res->body.find("ViewHelper"), std::string::npos);
}

TEST_F(MapViewerServer, StreamsMapPcdBytesVerbatim)
{
  httplib::Client client("127.0.0.1", port_);
  const auto res = client.Get("/map.pcd");
  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 200);
  EXPECT_EQ(res->body, map_bytes_);
}

// A map larger than the 1 MiB content-provider chunk exercises the multi-call
// streaming path (seek + read + write across several chunks), which the small
// fixture above does not.
TEST(MapViewerServerLarge, StreamsAcrossMultipleChunks)
{
  std::string big;
  big.resize((2U << 20) + 12345U);  // > 2 MiB, not a chunk multiple
  for (std::size_t i = 0; i < big.size(); ++i) {
    big[i] = static_cast<char>((i * 31U + 7U) & 0xFFU);  // deterministic pattern
  }

  const auto path =
    std::filesystem::temp_directory_path() / "bagwiz_map_viewer_server_large_test.pcd";
  {
    std::ofstream out(path, std::ios::binary);
    out.write(big.data(), static_cast<std::streamsize>(big.size()));
  }

  httplib::Server server;
  slam::register_map_viewer_routes(server, path);
  const int port = server.bind_to_any_port("127.0.0.1");
  ASSERT_GE(port, 0);
  std::thread server_thread([&server] { server.listen_after_bind(); });
  for (int i = 0; i < 200 && !server.is_running(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_TRUE(server.is_running());

  httplib::Client client("127.0.0.1", port);
  const auto res = client.Get("/map.pcd");

  server.stop();
  server_thread.join();
  std::error_code ec;
  std::filesystem::remove(path, ec);

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 200);
  ASSERT_EQ(res->body.size(), big.size());
  EXPECT_EQ(res->body, big);
}

// Spin up a viewer server on `path`, GET /map.pcd once, and return the body.
std::string fetch_map_pcd(const std::filesystem::path & path)
{
  httplib::Server server;
  slam::register_map_viewer_routes(server, path);
  const int port = server.bind_to_any_port("127.0.0.1");
  EXPECT_GE(port, 0);
  std::thread server_thread([&server] { server.listen_after_bind(); });
  for (int i = 0; i < 200 && !server.is_running(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_TRUE(server.is_running());

  httplib::Client client("127.0.0.1", port);
  const auto res = client.Get("/map.pcd");

  server.stop();
  server_thread.join();

  EXPECT_TRUE(res);
  return res ? res->body : std::string{};
}

// Regression for the `--viewer` "Offset is outside the bounds of the DataView"
// crash: the map writer must flush/close its ofstream before the viewer serves
// the file. While the producing ofstream is still open, its final partial
// (<BUFSIZ) block sits in the user-space buffer and has not reached the OS, so
// file_size() and the served body are short of the PCD header's POINTS count and
// the browser's loader reads past the end. Once closed, the file is complete.
//
// The cloud is sized so its binary PCD exceeds the stdio buffer and its total
// length is not a buffer-block multiple, which guarantees an unflushed tail on
// libstdc++.
TEST(MapViewerServerFlush, ServesCompleteMapOnlyAfterWriterCloses)
{
  constexpr std::size_t kPoints = 5000;
  std::vector<std::array<float, 3>> points;
  std::vector<float> intensities;
  points.reserve(kPoints);
  intensities.reserve(kPoints);
  for (std::size_t i = 0; i < kPoints; ++i) {
    const auto f = static_cast<float>(i);
    points.push_back({f, f * 2.0F, f * 3.0F});
    intensities.push_back(f * 0.5F);
  }

  const auto path = std::filesystem::temp_directory_path() / "bagwiz_map_viewer_flush_test.pcd";
  std::error_code ec;
  std::filesystem::remove(path, ec);

  std::ofstream out(path, std::ios::binary);
  ASSERT_TRUE(out.good());
  slam::write_pcd(out, points, intensities);
  ASSERT_TRUE(out.good());

  // Served while the writer's stream is still open: this is the buggy state, and
  // the body is shorter than the eventual file.
  const std::string while_open = fetch_map_pcd(path);

  out.close();  // flush to disk, exactly as the --viewer write path now does
  ASSERT_TRUE(out.good());

  const auto full_size = std::filesystem::file_size(path, ec);
  ASSERT_FALSE(ec);

  // Served after close: the full, self-consistent file.
  const std::string after_close = fetch_map_pcd(path);

  std::filesystem::remove(path, ec);

  EXPECT_LT(while_open.size(), full_size);   // the bug: truncated body
  EXPECT_EQ(after_close.size(), full_size);  // the fix: complete body
}

}  // namespace
