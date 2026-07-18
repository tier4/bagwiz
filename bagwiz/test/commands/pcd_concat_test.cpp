// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/pcd_concat.hpp"

#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <span>
#include <string>
#include <vector>

namespace
{

using bagwiz::commands::PcdConcatArgs;
using bagwiz::commands::run_pcd_concat;
namespace pc = bagwiz::core::pointcloud;

constexpr std::int64_t kMs = 1'000'000;

bagwiz::io::TopicInfo pcd_topic_info(const std::string & name)
{
  bagwiz::io::TopicInfo t;
  t.name = name;
  t.type = "sensor_msgs/msg/PointCloud2";
  t.serialization_format = "cdr";
  return t;
}

bagwiz::io::CreateOptions mcap_options()
{
  bagwiz::io::CreateOptions o;
  o.format = bagwiz::io::Format::Mcap;
  o.layout = bagwiz::io::Layout::SingleFile;
  o.mcap_compression = "none";
  return o;
}

// A base_link-framed xyz-only cloud (point_step 12) with one point per x in `xs`.
// The base_link frame means concat resolves an identity extrinsic and needs no TF.
std::vector<std::byte> serialize_cloud(std::int64_t stamp_ns, const std::vector<float> & xs)
{
  pc::PointCloud2 c;
  c.timestamp_ns = stamp_ns;
  c.frame_id = "base_link";
  c.height = 1;
  c.width = static_cast<std::uint32_t>(xs.size());
  c.fields = {
    {"x", 0, pc::PointFieldType::kFloat32, 1},
    {"y", 4, pc::PointFieldType::kFloat32, 1},
    {"z", 8, pc::PointFieldType::kFloat32, 1},
  };
  c.point_step = 12;
  c.row_step = 12 * c.width;
  c.is_dense = true;
  c.data.assign(static_cast<std::size_t>(c.width) * 12, std::byte{0});
  for (std::size_t i = 0; i < xs.size(); ++i) {
    std::memcpy(c.data.data() + i * 12, &xs[i], sizeof(float));
  }
  return pc::serialize_pointcloud2(c);
}

// Write /front and /rear, two messages each at 1000/1100 ms (front x=1, rear x=2).
// `add_collision` also writes a /concat topic that the output would collide with.
void write_input(const std::filesystem::path & path, bool add_collision = false)
{
  auto w = bagwiz::io::open_write(path, mcap_options());
  w->declare_topic(pcd_topic_info("/front"));
  w->declare_topic(pcd_topic_info("/rear"));
  if (add_collision) {
    w->declare_topic(pcd_topic_info("/concat"));
  }
  for (const std::int64_t stamp : {1000 * kMs, 1100 * kMs}) {
    const auto f = serialize_cloud(stamp, {1.0f});
    const auto r = serialize_cloud(stamp, {2.0f});
    w->write("/front", stamp, std::span<const std::byte>(f.data(), f.size()));
    w->write("/rear", stamp, std::span<const std::byte>(r.data(), r.size()));
    if (add_collision) {
      const auto x = serialize_cloud(stamp, {9.0f});
      w->write("/concat", stamp, std::span<const std::byte>(x.data(), x.size()));
    }
  }
  w->close();
}

struct TopicRead
{
  bool present = false;
  int message_count = 0;
  std::uint32_t last_width = 0;
  std::string last_frame;
};

TopicRead read_topic(const std::filesystem::path & path, const std::string & topic)
{
  TopicRead out;
  auto reader = bagwiz::io::open_read(path);
  for (const auto & t : reader->topics()) {
    if (t.name == topic) {
      out.present = true;
    }
  }
  if (!out.present) {
    return out;
  }
  bagwiz::io::ReadFilter filter;
  filter.topics = {topic};
  reader->set_filter(filter);
  bagwiz::io::RawMessage raw;
  while (reader->next(raw)) {
    if (raw.topic->name != topic) {
      continue;
    }
    ++out.message_count;
    const auto parsed = pc::parse_pointcloud2(raw.payload);
    if (parsed.ok()) {
      out.last_width = parsed.cloud->width;
      out.last_frame = parsed.cloud->frame_id;
    }
  }
  return out;
}

PcdConcatArgs base_args(const std::filesystem::path & in, const std::filesystem::path & out)
{
  PcdConcatArgs a;
  a.input_path = in;
  a.output_topic = "/concat";
  a.pcd_topics = {"/front", "/rear"};
  a.frame = "base_link";
  a.output_path = out;
  a.overwrite = true;
  return a;
}

class PcdConcatTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_ = std::filesystem::temp_directory_path() /
           ("bagwiz_pcd_concat_" +
            std::to_string(::testing::UnitTest::GetInstance()->current_test_info()->line()));
    std::filesystem::remove_all(tmp_);
    std::filesystem::create_directories(tmp_);
  }
  void TearDown() override { std::filesystem::remove_all(tmp_); }

  std::filesystem::path tmp_;
};

}  // namespace

// Reference-driven concat writes one merged message per reference (/front) message
// in the target frame, with width = sum of the matched clouds' widths; input
// topics are kept by default.
TEST_F(PcdConcatTest, ConcatenatesAndKeepsInputs)
{
  const auto in = tmp_ / "in.mcap";
  const auto out = tmp_ / "out.mcap";
  write_input(in);
  ASSERT_EQ(run_pcd_concat(base_args(in, out)), 0);

  const auto concat = read_topic(out, "/concat");
  ASSERT_TRUE(concat.present);
  EXPECT_EQ(concat.message_count, 2);  // one per reference message
  EXPECT_EQ(concat.last_width, 2u);    // 1 front point + 1 rear point
  EXPECT_EQ(concat.last_frame, "base_link");
  EXPECT_TRUE(read_topic(out, "/front").present);  // kept by default
  EXPECT_TRUE(read_topic(out, "/rear").present);
}

TEST_F(PcdConcatTest, DropInputsRemovesSourceTopics)
{
  const auto in = tmp_ / "in.mcap";
  const auto out = tmp_ / "out.mcap";
  write_input(in);
  auto args = base_args(in, out);
  args.drop_inputs = true;
  ASSERT_EQ(run_pcd_concat(args), 0);

  EXPECT_TRUE(read_topic(out, "/concat").present);
  EXPECT_FALSE(read_topic(out, "/front").present);
  EXPECT_FALSE(read_topic(out, "/rear").present);
}

TEST_F(PcdConcatTest, OutputTopicCollisionNeedsForce)
{
  const auto in = tmp_ / "in.mcap";
  const auto out = tmp_ / "out.mcap";
  write_input(in, /*add_collision=*/true);  // input already carries a /concat topic

  auto args = base_args(in, out);
  args.force = false;
  EXPECT_EQ(run_pcd_concat(args), 1);  // rejected without --force

  args.force = true;
  ASSERT_EQ(run_pcd_concat(args), 0);  // replaced with --force
  EXPECT_TRUE(read_topic(out, "/concat").present);
}

TEST_F(PcdConcatTest, MissingInputTopicIsError)
{
  const auto in = tmp_ / "in.mcap";
  const auto out = tmp_ / "out.mcap";
  write_input(in);
  auto args = base_args(in, out);
  args.pcd_topics = {"/front", "/nope"};  // /nope is not in the bag
  EXPECT_EQ(run_pcd_concat(args), 1);
}
