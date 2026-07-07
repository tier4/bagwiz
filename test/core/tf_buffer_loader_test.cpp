// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tf_buffer_loader.hpp"

#include "bagwiz/core/tf_message_wire.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <tf2/buffer_core.hpp>
#include <tf2/time.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <filesystem>
#include <span>
#include <string>
#include <vector>

namespace
{

using bagwiz::core::load_static_tf_buffer;

bagwiz::io::CreateOptions mcap_options()
{
  bagwiz::io::CreateOptions o;
  o.format = bagwiz::io::Format::Mcap;
  o.layout = bagwiz::io::Layout::SingleFile;
  o.mcap_compression = "none";
  return o;
}

geometry_msgs::msg::TransformStamped make_edge(
  const std::string & parent, const std::string & child, double tx)
{
  geometry_msgs::msg::TransformStamped ts;
  ts.header.frame_id = parent;
  ts.child_frame_id = child;
  ts.transform.translation.x = tx;
  ts.transform.translation.y = 0.0;
  ts.transform.translation.z = 0.0;
  ts.transform.rotation.x = 0.0;
  ts.transform.rotation.y = 0.0;
  ts.transform.rotation.z = 0.0;
  ts.transform.rotation.w = 1.0;
  return ts;
}

// Writes a single-topic bag with one /tf_static TFMessage: base_link -> lidar,
// translation (1,0,0).
void write_tf_static_bag(const std::filesystem::path & path)
{
  auto w = bagwiz::io::open_write(path, mcap_options());
  w->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf_static"));
  std::vector<geometry_msgs::msg::TransformStamped> edges;
  edges.push_back(make_edge("base_link", "lidar", 1.0));
  const auto payload = bagwiz::core::serialize_tf_message(edges);
  w->write("/tf_static", 0, std::span<const std::byte>(payload.data(), payload.size()));
  w->close();
}

TEST(LoadStaticTfBuffer, ResolvesStaticEdge)
{
  const std::filesystem::path bag =
    std::filesystem::temp_directory_path() / "bagwiz_tfstatic_test.mcap";
  std::filesystem::remove(bag);
  write_tf_static_bag(bag);

  tf2::BufferCore buffer{std::chrono::hours(24 * 365)};
  ASSERT_FALSE(load_static_tf_buffer(bag, buffer).has_value());
  const auto ts = buffer.lookupTransform("base_link", "lidar", tf2::TimePointZero);
  EXPECT_NEAR(ts.transform.translation.x, 1.0, 1e-9);

  std::filesystem::remove(bag);
}

TEST(LoadStaticTfBuffer, ErrorsWhenNoStaticTfTopic)
{
  const std::filesystem::path bag =
    std::filesystem::temp_directory_path() / "bagwiz_tfstatic_missing_test.mcap";
  std::filesystem::remove(bag);
  {
    auto w = bagwiz::io::open_write(bag, mcap_options());
    w->close();
  }

  tf2::BufferCore buffer{std::chrono::hours(24 * 365)};
  const auto error = load_static_tf_buffer(bag, buffer);
  ASSERT_TRUE(error.has_value());
  EXPECT_NE(error->find("tf_static"), std::string::npos);

  std::filesystem::remove(bag);
}

}  // namespace
