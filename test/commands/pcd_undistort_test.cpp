// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/pcd_undistort.hpp"

#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/core/tf_message_wire.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace
{

using bagwiz::commands::PcdUndistortArgs;
using bagwiz::commands::run_pcd_undistort;
namespace pc = bagwiz::core::pointcloud;

constexpr std::int64_t kMs = 1'000'000;
// map->base_link translates +1m in x over these 100ms; the /points message
// below samples a per-point time exactly 0.1s after its header stamp, so the
// deskewed point lands on the base_link pose recorded at kPoseT1Ns with no
// interpolation ambiguity.
constexpr std::int64_t kPoseT0Ns = 1000 * kMs;
constexpr std::int64_t kPoseT1Ns = 1100 * kMs;

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

geometry_msgs::msg::TransformStamped make_map_to_base_link(std::int64_t stamp_ns, double tx)
{
  geometry_msgs::msg::TransformStamped ts;
  ts.header.frame_id = "map";
  ts.header.stamp.sec = static_cast<std::int32_t>(stamp_ns / 1'000'000'000LL);
  ts.header.stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % 1'000'000'000LL);
  ts.child_frame_id = "base_link";
  ts.transform.translation.x = tx;
  ts.transform.rotation.w = 1.0;
  return ts;
}

// One point [x y z] (+ optional "t" relative-seconds field), all float32.
std::vector<std::byte> serialize_cloud(
  std::int64_t stamp_ns, const std::string & frame_id, float x, std::optional<float> t_sec)
{
  pc::PointCloud2 c;
  c.timestamp_ns = stamp_ns;
  c.frame_id = frame_id;
  c.height = 1;
  c.width = 1;
  c.fields = {
    {"x", 0, pc::PointFieldType::kFloat32, 1},
    {"y", 4, pc::PointFieldType::kFloat32, 1},
    {"z", 8, pc::PointFieldType::kFloat32, 1},
  };
  c.point_step = 12;
  if (t_sec.has_value()) {
    c.fields.push_back({"t", 12, pc::PointFieldType::kFloat32, 1});
    c.point_step = 16;
  }
  c.row_step = c.point_step;
  c.is_dense = true;
  c.data.assign(c.point_step, std::byte{0});
  const float zero = 0.0f;
  std::memcpy(c.data.data() + 0, &x, sizeof(float));
  std::memcpy(c.data.data() + 4, &zero, sizeof(float));
  std::memcpy(c.data.data() + 8, &zero, sizeof(float));
  if (t_sec.has_value()) {
    const float t = *t_sec;
    std::memcpy(c.data.data() + 12, &t, sizeof(float));
  }
  return pc::serialize_pointcloud2(c);
}

// Writes:
//   /pose_tf   tf2_msgs/msg/TFMessage, map->base_link, tx 0.0 (t0) -> 1.0 (t1)
//   /tf_static tf2_msgs/msg/TFMessage, present but carries no edges (base_link
//              already equals --to for /points, so no extrinsic hop is needed;
//              load_static_tf_buffer only requires the topic to exist)
//   /points    sensor_msgs/msg/PointCloud2, frame base_link, one point at
//              local x=0 with per-point relative time 0.1s (when
//              with_time_field); header.stamp = t0
//   /other     sensor_msgs/msg/PointCloud2, an unrelated topic (not in --pcd)
//              used to check verbatim copy-through
void write_undistort_input(const std::filesystem::path & path, bool with_time_field)
{
  auto w = bagwiz::io::open_write(path, mcap_options());
  w->declare_topic(bagwiz::core::make_tf_message_topic_info("/pose_tf"));
  w->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf_static"));
  w->declare_topic(pcd_topic_info("/points"));
  w->declare_topic(pcd_topic_info("/other"));

  {
    const std::vector<geometry_msgs::msg::TransformStamped> edges0{
      make_map_to_base_link(kPoseT0Ns, 0.0)};
    const std::vector<geometry_msgs::msg::TransformStamped> edges1{
      make_map_to_base_link(kPoseT1Ns, 1.0)};
    const auto p0 = bagwiz::core::serialize_tf_message(edges0);
    const auto p1 = bagwiz::core::serialize_tf_message(edges1);
    w->write("/pose_tf", kPoseT0Ns, std::span<const std::byte>(p0.data(), p0.size()));
    w->write("/pose_tf", kPoseT1Ns, std::span<const std::byte>(p1.data(), p1.size()));
  }
  {
    const std::vector<geometry_msgs::msg::TransformStamped> no_edges;
    const auto s = bagwiz::core::serialize_tf_message(no_edges);
    w->write("/tf_static", 0, std::span<const std::byte>(s.data(), s.size()));
  }
  {
    const auto pts = with_time_field ? serialize_cloud(kPoseT0Ns, "base_link", 0.0f, 0.1f)
                                     : serialize_cloud(kPoseT0Ns, "base_link", 0.0f, std::nullopt);
    w->write("/points", kPoseT0Ns, std::span<const std::byte>(pts.data(), pts.size()));
  }
  {
    const auto other = serialize_cloud(kPoseT0Ns, "some_other_frame", 42.0f, std::nullopt);
    w->write("/other", kPoseT0Ns, std::span<const std::byte>(other.data(), other.size()));
  }
  w->close();
}

// Same as write_undistort_input, but omits /tf_static entirely: a realistic
// minimal SLAM-free bag that never published any static TF at all (as
// opposed to write_undistort_input's /tf_static, which is present but
// carries zero edges). Used to exercise load_static_tf_buffer's "no static TF
// topic" failure without a --pcd-topic-frame extrinsic hop muddying it.
void write_undistort_input_no_static_topic(const std::filesystem::path & path)
{
  auto w = bagwiz::io::open_write(path, mcap_options());
  w->declare_topic(bagwiz::core::make_tf_message_topic_info("/pose_tf"));
  w->declare_topic(pcd_topic_info("/points"));

  const std::vector<geometry_msgs::msg::TransformStamped> edges0{
    make_map_to_base_link(kPoseT0Ns, 0.0)};
  const std::vector<geometry_msgs::msg::TransformStamped> edges1{
    make_map_to_base_link(kPoseT1Ns, 1.0)};
  const auto p0 = bagwiz::core::serialize_tf_message(edges0);
  const auto p1 = bagwiz::core::serialize_tf_message(edges1);
  w->write("/pose_tf", kPoseT0Ns, std::span<const std::byte>(p0.data(), p0.size()));
  w->write("/pose_tf", kPoseT1Ns, std::span<const std::byte>(p1.data(), p1.size()));

  const auto pts = serialize_cloud(kPoseT0Ns, "base_link", 0.0f, 0.1f);
  w->write("/points", kPoseT0Ns, std::span<const std::byte>(pts.data(), pts.size()));
  w->close();
}

// A malformed cloud: declares a "t" field whose offset+size runs past
// point_step (data buffer sized to the too-small point_step). Used to
// exercise the upfront per-`--pcd`-topic bounds check on the per-point time
// field: an out-of-bounds field must be rejected the same way an absent one
// is (deskew_pointcloud2 applies the identical bounds check and would
// otherwise silently pass the cloud through un-deskewed).
std::vector<std::byte> serialize_cloud_oob_time_field(
  std::int64_t stamp_ns, const std::string & frame_id)
{
  pc::PointCloud2 c;
  c.timestamp_ns = stamp_ns;
  c.frame_id = frame_id;
  c.height = 1;
  c.width = 1;
  c.fields = {
    {"x", 0, pc::PointFieldType::kFloat32, 1},
    {"y", 4, pc::PointFieldType::kFloat32, 1},
    {"z", 8, pc::PointFieldType::kFloat32, 1},
    {"t", 12, pc::PointFieldType::kFloat32, 1},  // offset 12 + 4 bytes = 16, past point_step below
  };
  c.point_step = 12;  // deliberately too small: "t" doesn't fit
  c.row_step = c.point_step;
  c.is_dense = true;
  c.data.assign(c.point_step, std::byte{0});
  return pc::serialize_pointcloud2(c);
}

// Writes a two --pcd topic bag:
//   /points_a and /points_b both have per-point time and require deskew;
//   /other is a non-target PointCloud2 topic interleaved between them;
//   /pose_tf and /tf_static supply the map->base_link trajectory.
// Both /points_a and /points_b contain one point at local x=0 with relative
// per-point time 0.1s, so deskewing to the header stamp moves each to x=+1.
void write_two_pcd_topics_input(const std::filesystem::path & path)
{
  auto w = bagwiz::io::open_write(path, mcap_options());
  w->declare_topic(bagwiz::core::make_tf_message_topic_info("/pose_tf"));
  w->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf_static"));
  w->declare_topic(pcd_topic_info("/points_a"));
  w->declare_topic(pcd_topic_info("/points_b"));
  w->declare_topic(pcd_topic_info("/other"));

  {
    const std::vector<geometry_msgs::msg::TransformStamped> edges0{
      make_map_to_base_link(kPoseT0Ns, 0.0)};
    const std::vector<geometry_msgs::msg::TransformStamped> edges1{
      make_map_to_base_link(kPoseT1Ns, 1.0)};
    const auto p0 = bagwiz::core::serialize_tf_message(edges0);
    const auto p1 = bagwiz::core::serialize_tf_message(edges1);
    w->write("/pose_tf", kPoseT0Ns, std::span<const std::byte>(p0.data(), p0.size()));
    w->write("/pose_tf", kPoseT1Ns, std::span<const std::byte>(p1.data(), p1.size()));
  }
  {
    const std::vector<geometry_msgs::msg::TransformStamped> no_edges;
    const auto s = bagwiz::core::serialize_tf_message(no_edges);
    w->write("/tf_static", 0, std::span<const std::byte>(s.data(), s.size()));
  }
  {
    const auto a = serialize_cloud(kPoseT0Ns, "base_link", 0.0f, 0.1f);
    const auto b = serialize_cloud(kPoseT0Ns, "base_link", 0.0f, 0.1f);
    const auto other = serialize_cloud(kPoseT0Ns, "some_other_frame", 42.0f, std::nullopt);
    // Interleave the topics so order preservation is non-trivial.
    w->write("/points_a", kPoseT0Ns, std::span<const std::byte>(a.data(), a.size()));
    w->write("/other", kPoseT0Ns, std::span<const std::byte>(other.data(), other.size()));
    w->write("/points_b", kPoseT0Ns, std::span<const std::byte>(b.data(), b.size()));
  }
  w->close();
}

// Same as write_undistort_input(path, /*with_time_field=*/true), but /points'
// "t" field is out-of-bounds (see serialize_cloud_oob_time_field) instead of
// simply absent.
void write_undistort_input_oob_time_field(const std::filesystem::path & path)
{
  auto w = bagwiz::io::open_write(path, mcap_options());
  w->declare_topic(bagwiz::core::make_tf_message_topic_info("/pose_tf"));
  w->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf_static"));
  w->declare_topic(pcd_topic_info("/points"));

  const std::vector<geometry_msgs::msg::TransformStamped> edges0{
    make_map_to_base_link(kPoseT0Ns, 0.0)};
  const std::vector<geometry_msgs::msg::TransformStamped> edges1{
    make_map_to_base_link(kPoseT1Ns, 1.0)};
  const auto p0 = bagwiz::core::serialize_tf_message(edges0);
  const auto p1 = bagwiz::core::serialize_tf_message(edges1);
  w->write("/pose_tf", kPoseT0Ns, std::span<const std::byte>(p0.data(), p0.size()));
  w->write("/pose_tf", kPoseT1Ns, std::span<const std::byte>(p1.data(), p1.size()));

  const std::vector<geometry_msgs::msg::TransformStamped> no_edges;
  const auto s = bagwiz::core::serialize_tf_message(no_edges);
  w->write("/tf_static", 0, std::span<const std::byte>(s.data(), s.size()));

  const auto pts = serialize_cloud_oob_time_field(kPoseT0Ns, "base_link");
  w->write("/points", kPoseT0Ns, std::span<const std::byte>(pts.data(), pts.size()));
  w->close();
}

std::optional<float> read_first_point_x(
  const std::filesystem::path & path, const std::string & topic)
{
  auto reader = bagwiz::io::open_read(path);
  bagwiz::io::ReadFilter filter;
  filter.topics = {topic};
  reader->set_filter(filter);
  bagwiz::io::RawMessage raw;
  if (!reader->next(raw)) {
    return std::nullopt;
  }
  const auto parsed = pc::parse_pointcloud2(raw.payload);
  if (!parsed.ok() || parsed.cloud->width == 0) {
    return std::nullopt;
  }
  float x = 0.0f;
  std::memcpy(&x, parsed.cloud->data.data(), sizeof(float));
  return x;
}

// Raw per-message payload bytes for a topic, in bag order. Used to assert
// verbatim (byte-identical) copy-through.
std::vector<std::vector<std::byte>> read_raw_payloads(
  const std::filesystem::path & path, const std::string & topic)
{
  std::vector<std::vector<std::byte>> out;
  auto reader = bagwiz::io::open_read(path);
  bagwiz::io::ReadFilter filter;
  filter.topics = {topic};
  reader->set_filter(filter);
  bagwiz::io::RawMessage raw;
  while (reader->next(raw)) {
    out.emplace_back(raw.payload.begin(), raw.payload.end());
  }
  return out;
}

PcdUndistortArgs base_args(const std::filesystem::path & in, const std::filesystem::path & out)
{
  PcdUndistortArgs a;
  a.input_path = in;
  a.pose_topic = "/pose_tf";
  a.pcd_topics = {"/points"};
  a.from_frame = "map";
  a.to_frame = "base_link";
  a.output_path = out;
  a.overwrite = true;
  return a;
}

class PcdUndistortTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_ = std::filesystem::temp_directory_path() /
           ("bagwiz_pcd_undistort_" +
            std::to_string(::testing::UnitTest::GetInstance()->current_test_info()->line()));
    std::filesystem::remove_all(tmp_);
    std::filesystem::create_directories(tmp_);
    in_ = tmp_ / "in.mcap";
    out_ = tmp_ / "out.mcap";
  }
  void TearDown() override { std::filesystem::remove_all(tmp_); }

  std::filesystem::path tmp_;
  std::filesystem::path in_;
  std::filesystem::path out_;
};

}  // namespace

// map->base_link moves +1m in x over 100ms. The lone /points point sits at
// the sensor origin with a per-point relative time of 0.1s, so deskewing to
// the cloud's header stamp (t0) carries it to the base_link pose recorded at
// t0+0.1s = t1, i.e. local x=0 -> x=+1. See deskew_test.cpp's
// PureTranslationMovesPointToRefPose for the same scenario at the kernel level.
TEST_F(PcdUndistortTest, DeskewsTargetTopicAndPreservesOthers)
{
  write_undistort_input(in_, /*with_time_field=*/true);
  const auto other_before = read_raw_payloads(in_, "/other");
  const auto pose_before = read_raw_payloads(in_, "/pose_tf");
  ASSERT_EQ(other_before.size(), 1u);
  ASSERT_EQ(pose_before.size(), 2u);

  ASSERT_EQ(run_pcd_undistort(base_args(in_, out_)), 0);

  const auto x = read_first_point_x(out_, "/points");
  ASSERT_TRUE(x.has_value());
  EXPECT_NEAR(*x, 1.0f, 1e-4f);

  EXPECT_EQ(read_raw_payloads(out_, "/other"), other_before);
  EXPECT_EQ(read_raw_payloads(out_, "/pose_tf"), pose_before);
}

TEST_F(PcdUndistortTest, MissingPerPointTimeIsFatal)
{
  write_undistort_input(in_, /*with_time_field=*/false);
  EXPECT_EQ(run_pcd_undistort(base_args(in_, out_)), 1);
}

// A "t" field declared past point_step must be rejected the same way an
// absent one is, not silently passed through un-deskewed (see
// cloud_has_usable_point_time in the runner).
TEST_F(PcdUndistortTest, OutOfBoundsTimeFieldIsFatal)
{
  write_undistort_input_oob_time_field(in_);
  EXPECT_EQ(run_pcd_undistort(base_args(in_, out_)), 1);
}

TEST_F(PcdUndistortTest, UnresolvableToIsFatal)
{
  write_undistort_input(in_, /*with_time_field=*/true);
  auto a = base_args(in_, out_);
  a.to_frame = "no_such_frame";
  EXPECT_EQ(run_pcd_undistort(a), 1);
}

// A bag with no static TF topic at all (a realistic minimal SLAM-free bag)
// must fail fast. This is a regression test for a defect where the surfaced
// error forwarded load_static_tf_buffer's message verbatim, which ends in
// "...--frame" (pcd concat's flag; pcd undistort has none). The exact log
// text isn't asserted here (this suite only checks exit codes, matching
// every other test in this file); it was verified by hand — see the task-4
// report's "Fix + test results" section for the exact printed line.
TEST_F(PcdUndistortTest, MissingStaticTfTopicIsFatal)
{
  write_undistort_input_no_static_topic(in_);
  EXPECT_EQ(run_pcd_undistort(base_args(in_, out_)), 1);
}

// In-place mode (output_path unset) is the default entry point. Run it on a
// copy of the fixture bag so `in_` stays untouched as the "before" reference
// that other_before / pose_before are read from.
TEST_F(PcdUndistortTest, InPlaceRewritesTargetTopicAndPreservesOthers)
{
  write_undistort_input(in_, /*with_time_field=*/true);
  const auto other_before = read_raw_payloads(in_, "/other");
  const auto pose_before = read_raw_payloads(in_, "/pose_tf");
  ASSERT_EQ(other_before.size(), 1u);
  ASSERT_EQ(pose_before.size(), 2u);

  const auto inplace_path = tmp_ / "inplace.mcap";
  std::filesystem::copy_file(in_, inplace_path);

  PcdUndistortArgs a;
  a.input_path = inplace_path;
  a.pose_topic = "/pose_tf";
  a.pcd_topics = {"/points"};
  a.from_frame = "map";
  a.to_frame = "base_link";
  // a.output_path is left unset -> in-place.
  ASSERT_EQ(run_pcd_undistort(a), 0);

  const auto x = read_first_point_x(inplace_path, "/points");
  ASSERT_TRUE(x.has_value());
  EXPECT_NEAR(*x, 1.0f, 1e-4f);

  EXPECT_EQ(read_raw_payloads(inplace_path, "/other"), other_before);
  EXPECT_EQ(read_raw_payloads(inplace_path, "/pose_tf"), pose_before);
}

TEST_F(PcdUndistortTest, DeskewsMultiplePcdTopics)
{
  write_two_pcd_topics_input(in_);
  auto a = base_args(in_, out_);
  a.pcd_topics = {"/points_a", "/points_b"};
  a.threads = 2;
  ASSERT_EQ(run_pcd_undistort(a), 0);

  const auto xa = read_first_point_x(out_, "/points_a");
  const auto xb = read_first_point_x(out_, "/points_b");
  ASSERT_TRUE(xa.has_value());
  ASSERT_TRUE(xb.has_value());
  EXPECT_NEAR(*xa, 1.0f, 1e-4f);
  EXPECT_NEAR(*xb, 1.0f, 1e-4f);

  // Verify order preservation on non-pcd topic
  EXPECT_EQ(read_raw_payloads(out_, "/other"), read_raw_payloads(in_, "/other"));
}

TEST_F(PcdUndistortTest, SyncAndParallelOutputsAreIdentical)
{
  write_two_pcd_topics_input(in_);
  const auto sync_out = tmp_ / "sync.mcap";
  const auto par_out = tmp_ / "par.mcap";

  auto sync_args = base_args(in_, sync_out);
  sync_args.pcd_topics = {"/points_a", "/points_b"};
  sync_args.threads = 1;
  ASSERT_EQ(run_pcd_undistort(sync_args), 0);

  auto par_args = base_args(in_, par_out);
  par_args.pcd_topics = {"/points_a", "/points_b"};
  par_args.threads = 2;
  ASSERT_EQ(run_pcd_undistort(par_args), 0);

  EXPECT_EQ(read_raw_payloads(sync_out, "/points_a"), read_raw_payloads(par_out, "/points_a"));
  EXPECT_EQ(read_raw_payloads(sync_out, "/points_b"), read_raw_payloads(par_out, "/points_b"));
  EXPECT_EQ(read_raw_payloads(sync_out, "/other"), read_raw_payloads(par_out, "/other"));
}
