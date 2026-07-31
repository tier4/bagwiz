// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/tf_static_cp.hpp"

#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/tf/tf_message_wire.hpp"
#include "bagwiz/core/tf/tf_value_extract.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <gtest/gtest.h>
#include <sqlite3.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace
{

constexpr const char * kTfMessageType = "tf2_msgs/msg/TFMessage";

geometry_msgs::msg::TransformStamped make_edge(
  const std::string & parent, const std::string & child, double tx)
{
  geometry_msgs::msg::TransformStamped ts;
  ts.header.frame_id = parent;
  ts.header.stamp.sec = 42;  // a non-destination stamp, to prove it is rewritten
  ts.header.stamp.nanosec = 7;
  ts.child_frame_id = child;
  ts.transform.translation.x = tx;
  ts.transform.rotation.w = 1.0;
  return ts;
}

bagwiz::io::TopicInfo tf_topic_info(const std::string & name)
{
  bagwiz::io::TopicInfo t;
  t.name = name;
  t.type = kTfMessageType;
  t.serialization_format = "cdr";
  t.schema_encoding = "ros2msg";
  t.schema_text = bagwiz::core::kTfMessageWireSchema;
  return t;
}

void write_tf_message(
  bagwiz::io::BagWriter & writer, const std::string & topic, std::int64_t stamp_ns,
  const std::vector<geometry_msgs::msg::TransformStamped> & transforms)
{
  const auto cdr = bagwiz::core::serialize_tf_message(
    std::span<const geometry_msgs::msg::TransformStamped>(transforms.data(), transforms.size()));
  writer.write(topic, stamp_ns, std::span<const std::byte>(cdr.data(), cdr.size()));
}

bagwiz::io::CreateOptions mcap_options()
{
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Mcap;
  options.layout = bagwiz::io::Layout::SingleFile;
  options.mcap_compression = "none";
  return options;
}

// Source bag: one static topic (/tf_static: map->odom, odom->base_link) plus a
// dynamic /tf topic that must NOT be copied.
void write_src_bag(const std::filesystem::path & path)
{
  auto writer = bagwiz::io::open_write(path, mcap_options());
  writer->declare_topic(tf_topic_info("/tf_static"));
  writer->declare_topic(tf_topic_info("/tf"));
  write_tf_message(
    *writer, "/tf_static", 1'000'000'000LL,
    {make_edge("map", "odom", 1.0), make_edge("odom", "base_link", 2.0)});
  write_tf_message(*writer, "/tf", 2'000'000'000LL, {make_edge("odom", "base_link", 9.0)});
  writer->close();
}

// Destination bag: a single non-TF topic whose earliest message fixes the bag's
// start time at `start_ns`.
void write_dst_bag(const std::filesystem::path & path, std::int64_t start_ns)
{
  bagwiz::io::TopicInfo clock;
  clock.name = "/clock";
  clock.type = "std_msgs/msg/String";
  clock.serialization_format = "cdr";

  constexpr std::array<std::byte, 4> kPayload{
    std::byte{0xDE}, std::byte{0xAD}, std::byte{0xBE}, std::byte{0xEF}};
  const auto bytes = std::span<const std::byte>(kPayload.data(), kPayload.size());

  auto writer = bagwiz::io::open_write(path, mcap_options());
  writer->declare_topic(clock);
  writer->write("/clock", start_ns, bytes);
  writer->write("/clock", start_ns + 1'000'000'000LL, bytes);
  writer->close();
}

struct ReadStaticResult
{
  bool present = false;
  std::int64_t stamp_ns = 0;
  int message_count = 0;
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
};

// Read back a TF topic from `path`, decoding its (expected single) message.
ReadStaticResult read_tf_topic(const std::filesystem::path & path, const std::string & topic)
{
  ReadStaticResult result;
  auto reader = bagwiz::io::open_read(path);
  reader->populate_schemas();

  const bagwiz::io::TopicInfo * info = nullptr;
  for (const auto & t : reader->topics()) {
    if (t.name == topic) {
      info = &t;
      break;
    }
  }
  if (info == nullptr) {
    return result;
  }
  result.present = true;

  auto open = bagwiz::core::decoder::open_decoder(*info);
  EXPECT_TRUE(open.ok()) << open.error;

  bagwiz::io::ReadFilter filter;
  filter.topics = {topic};
  reader->set_filter(filter);

  bagwiz::io::RawMessage raw;
  while (reader->next(raw)) {
    if (raw.topic->name != topic) {
      continue;
    }
    ++result.message_count;
    result.stamp_ns = raw.timestamp_ns;
    const auto decoded = open.decoder->decode(raw.payload);
    EXPECT_TRUE(decoded.ok()) << decoded.error;
    result.transforms = bagwiz::core::extract_tf_message(*decoded.value);
  }
  return result;
}

bool topic_present(const std::filesystem::path & path, const std::string & topic)
{
  auto reader = bagwiz::io::open_read(path);
  for (const auto & t : reader->topics()) {
    if (t.name == topic) {
      return true;
    }
  }
  return false;
}

class TfStaticCpTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("bagwiz_tf_static_cp_" +
                std::to_string(::testing::UnitTest::GetInstance()->current_test_info()->line()));
    std::filesystem::remove_all(tmp_dir_);
    std::filesystem::create_directories(tmp_dir_);
  }
  void TearDown() override { std::filesystem::remove_all(tmp_dir_); }

  std::filesystem::path tmp_dir_;
};

// -o mode: the static topic is copied into the output, stamped at the
// destination's start time, with the dynamic /tf left behind.
TEST_F(TfStaticCpTest, CopiesStaticTfToOutputStampedAtDstStart)
{
  const auto src = tmp_dir_ / "src.mcap";
  const auto dst = tmp_dir_ / "dst.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  constexpr std::int64_t kDstStart = 5'000'000'000LL;
  write_src_bag(src);
  write_dst_bag(dst, kDstStart);

  const int rc =
    bagwiz::commands::run_tf_static_cp(src, dst, out, /*force=*/false, /*overwrite=*/false);
  ASSERT_EQ(rc, 0);

  const auto copied = read_tf_topic(out, "/tf_static");
  ASSERT_TRUE(copied.present);
  EXPECT_EQ(copied.message_count, 1);
  EXPECT_EQ(copied.stamp_ns, kDstStart);
  ASSERT_EQ(copied.transforms.size(), 2U);
  // header.stamp is rewritten to the destination start time (5s, 0ns), not the
  // source's 42s/7ns.
  for (const auto & t : copied.transforms) {
    EXPECT_EQ(t.header.stamp.sec, 5);
    EXPECT_EQ(t.header.stamp.nanosec, 0U);
  }
  EXPECT_EQ(copied.transforms[0].header.frame_id, "map");
  EXPECT_EQ(copied.transforms[0].child_frame_id, "odom");
  EXPECT_EQ(copied.transforms[1].child_frame_id, "base_link");

  // The dynamic /tf from the source is not copied; the destination's own topic
  // survives.
  EXPECT_FALSE(topic_present(out, "/tf"));
  EXPECT_TRUE(topic_present(out, "/clock"));
  // The destination bag itself is untouched in -o mode.
  EXPECT_FALSE(topic_present(dst, "/tf_static"));
}

TEST_F(TfStaticCpTest, InjectedStaticTfIsEmittedInTimestampOrder)
{
  // The synthesized message is stamped at the destination's start time, so it
  // belongs at the front of the bag. Appending it after the stream copy would
  // instead give it the highest rowid while holding the lowest timestamp — the
  // one row whose storage order disagrees with its time, which any consumer
  // reading in physical order (a .db3 full-table scan, for instance) would
  // deliver last.
  const auto src = tmp_dir_ / "src_order.mcap";
  const auto dst = tmp_dir_ / "dst_order.db3";
  const auto out = tmp_dir_ / "out_order.db3";
  constexpr std::int64_t kDstStart = 5'000'000'000LL;
  write_src_bag(src);

  {
    bagwiz::io::TopicInfo clock;
    clock.name = "/clock";
    clock.type = "std_msgs/msg/String";
    clock.serialization_format = "cdr";
    constexpr std::array<std::byte, 4> kPayload{
      std::byte{0xDE}, std::byte{0xAD}, std::byte{0xBE}, std::byte{0xEF}};
    const auto bytes = std::span<const std::byte>(kPayload.data(), kPayload.size());

    bagwiz::io::CreateOptions options;
    options.format = bagwiz::io::Format::Sqlite3;
    options.layout = bagwiz::io::Layout::SingleFile;
    auto writer = bagwiz::io::open_write(dst, options);
    writer->declare_topic(clock);
    writer->write("/clock", kDstStart, bytes);
    writer->write("/clock", kDstStart + 1'000'000'000LL, bytes);
    writer->close();
  }

  ASSERT_EQ(
    bagwiz::commands::run_tf_static_cp(src, dst, out, /*force=*/false, /*overwrite=*/false), 0);

  // Storage order must be non-decreasing in timestamp, so a reader that walks
  // the rows physically sees the same sequence as one that sorts by time.
  sqlite3 * db = nullptr;
  ASSERT_EQ(sqlite3_open(out.string().c_str(), &db), SQLITE_OK);
  sqlite3_stmt * stmt = nullptr;
  ASSERT_EQ(
    sqlite3_prepare_v2(
      db,
      "SELECT t.name, m.timestamp FROM messages m JOIN topics t ON t.id = m.topic_id "
      "ORDER BY m.id",
      -1, &stmt, nullptr),
    SQLITE_OK);
  std::vector<std::pair<std::string, std::int64_t>> rows;
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    rows.emplace_back(
      reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0)), sqlite3_column_int64(stmt, 1));
  }
  sqlite3_finalize(stmt);
  sqlite3_close(db);

  ASSERT_EQ(rows.size(), 3U);
  for (std::size_t i = 1; i < rows.size(); ++i) {
    EXPECT_LE(rows[i - 1].second, rows[i].second)
      << "row " << i << " (" << rows[i].first << ") is stored out of timestamp order";
  }
  EXPECT_EQ(rows.front().first, "/tf_static")
    << "the injected message shares the bag's start time and must lead";
}

// In-place mode (no -o): the destination bag gains the static topic.
TEST_F(TfStaticCpTest, InPlaceAddsStaticTfToDestination)
{
  const auto src = tmp_dir_ / "src.mcap";
  const auto dst = tmp_dir_ / "dst.mcap";
  constexpr std::int64_t kDstStart = 3'000'000'000LL;
  write_src_bag(src);
  write_dst_bag(dst, kDstStart);

  const int rc = bagwiz::commands::run_tf_static_cp(
    src, dst, std::nullopt, /*force=*/false, /*overwrite=*/false);
  ASSERT_EQ(rc, 0);

  const auto copied = read_tf_topic(dst, "/tf_static");
  ASSERT_TRUE(copied.present);
  EXPECT_EQ(copied.message_count, 1);
  EXPECT_EQ(copied.stamp_ns, kDstStart);
  EXPECT_EQ(copied.transforms.size(), 2U);
  EXPECT_TRUE(topic_present(dst, "/clock"));
}

// A source without any static TF is a hard error.
TEST_F(TfStaticCpTest, NoStaticTfInSourceFails)
{
  const auto src = tmp_dir_ / "src.mcap";
  const auto dst = tmp_dir_ / "dst.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  // Source with only a dynamic /tf topic.
  {
    auto writer = bagwiz::io::open_write(src, mcap_options());
    writer->declare_topic(tf_topic_info("/tf"));
    write_tf_message(*writer, "/tf", 1'000'000'000LL, {make_edge("odom", "base_link", 1.0)});
    writer->close();
  }
  write_dst_bag(dst, 1'000'000'000LL);

  EXPECT_EQ(
    bagwiz::commands::run_tf_static_cp(src, dst, out, /*force=*/false, /*overwrite=*/false), 1);
  EXPECT_FALSE(std::filesystem::exists(out));
}

// A static topic already present in the destination aborts without --force and is
// replaced with it. -w/--overwrite must NOT stand in for it: the two conflicts are
// separate permissions (as in `traj join` / `tf static join`), so clearing a path
// does not also authorise replacing a bag's real static TF.
TEST_F(TfStaticCpTest, CollidingTopicHonoursForceAndNotOverwrite)
{
  const auto src = tmp_dir_ / "src.mcap";
  const auto dst = tmp_dir_ / "dst.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  constexpr std::int64_t kDstStart = 4'000'000'000LL;
  write_src_bag(src);
  // Destination already carries a /tf_static with a single, different edge.
  {
    auto writer = bagwiz::io::open_write(dst, mcap_options());
    writer->declare_topic(tf_topic_info("/tf_static"));
    write_tf_message(*writer, "/tf_static", kDstStart, {make_edge("a", "b", 99.0)});
    writer->close();
  }

  // Without --force the collision aborts and no output is produced.
  EXPECT_EQ(
    bagwiz::commands::run_tf_static_cp(src, dst, out, /*force=*/false, /*overwrite=*/false), 1);
  EXPECT_FALSE(std::filesystem::exists(out));

  // -w alone does not permit it either.
  EXPECT_EQ(
    bagwiz::commands::run_tf_static_cp(src, dst, out, /*force=*/false, /*overwrite=*/true), 1);
  EXPECT_FALSE(std::filesystem::exists(out));

  // With --force the source's static TF replaces the destination's.
  ASSERT_EQ(
    bagwiz::commands::run_tf_static_cp(src, dst, out, /*force=*/true, /*overwrite=*/false), 0);
  const auto copied = read_tf_topic(out, "/tf_static");
  ASSERT_TRUE(copied.present);
  EXPECT_EQ(copied.message_count, 1);
  ASSERT_EQ(copied.transforms.size(), 2U);
  EXPECT_EQ(copied.transforms[0].child_frame_id, "odom");
}

// An existing output path aborts without -w/--overwrite and is replaced with it.
// --force does not stand in for it, the mirror of the case above.
TEST_F(TfStaticCpTest, ExistingOutputHonoursOverwriteAndNotForce)
{
  const auto src = tmp_dir_ / "src.mcap";
  const auto dst = tmp_dir_ / "dst.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_src_bag(src);
  write_dst_bag(dst, 1'000'000'000LL);
  write_dst_bag(out, 8'000'000'000LL);  // pre-existing output

  EXPECT_EQ(
    bagwiz::commands::run_tf_static_cp(src, dst, out, /*force=*/false, /*overwrite=*/false), 1);
  EXPECT_EQ(
    bagwiz::commands::run_tf_static_cp(src, dst, out, /*force=*/true, /*overwrite=*/false), 1);
  ASSERT_EQ(
    bagwiz::commands::run_tf_static_cp(src, dst, out, /*force=*/false, /*overwrite=*/true), 0);
  EXPECT_TRUE(topic_present(out, "/tf_static"));
}

}  // namespace
