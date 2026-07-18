// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/map_viewer_open.hpp"

#include <gtest/gtest.h>

#include <string>

namespace
{
namespace slam = bagwiz::core::slam;

TEST(MapViewerOpen, CommandEmbedsTheUrl)
{
  const std::string url = "http://127.0.0.1:54321/";
  const std::string cmd = slam::browser_open_command(url);
  EXPECT_NE(cmd.find(url), std::string::npos);
}

TEST(MapViewerOpen, CommandUsesThePlatformDefaultOpener)
{
  const std::string cmd = slam::browser_open_command("http://127.0.0.1:54321/");
#if defined(_WIN32)
  EXPECT_NE(cmd.find("start"), std::string::npos);
#elif defined(__APPLE__)
  EXPECT_NE(cmd.find("open"), std::string::npos);
#else
  EXPECT_NE(cmd.find("xdg-open"), std::string::npos);
#endif
}

#if !defined(_WIN32)
TEST(MapViewerOpen, BackgroundsTheLaunchOnPosixSoTheServerCanBlock)
{
  const std::string cmd = slam::browser_open_command("http://127.0.0.1:54321/");
  EXPECT_NE(cmd.find('&'), std::string::npos);
}
#endif

}  // namespace
