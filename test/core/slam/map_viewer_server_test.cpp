// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/map_viewer.hpp"

#include <gtest/gtest.h>
#include <httplib.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <ios>
#include <string>
#include <thread>

namespace
{
namespace slam = bagwiz::core::slam;

// All 256 byte values, so an exact echo proves the PLY is streamed verbatim
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
    map_path_ = std::filesystem::temp_directory_path() / "bagwiz_map_viewer_server_test.ply";
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
}

TEST_F(MapViewerServer, ServesViewerModule)
{
  httplib::Client client("127.0.0.1", port_);
  const auto res = client.Get("/map_viewer.js");
  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 200);
  EXPECT_NE(res->body.find("PLYLoader"), std::string::npos);
}

TEST_F(MapViewerServer, StreamsMapPlyBytesVerbatim)
{
  httplib::Client client("127.0.0.1", port_);
  const auto res = client.Get("/map.ply");
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
    std::filesystem::temp_directory_path() / "bagwiz_map_viewer_server_large_test.ply";
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
  const auto res = client.Get("/map.ply");

  server.stop();
  server_thread.join();
  std::error_code ec;
  std::filesystem::remove(path, ec);

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 200);
  ASSERT_EQ(res->body.size(), big.size());
  EXPECT_EQ(res->body, big);
}

}  // namespace
