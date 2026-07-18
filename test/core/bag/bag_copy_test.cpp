// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/bag/bag_copy.hpp"

#include "bagwiz/io/bag_io.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <span>
#include <string>
#include <system_error>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace
{

constexpr std::array<std::uint8_t, 4> kPayload{0x01, 0x02, 0x03, 0x04};

bagwiz::io::TopicInfo make_topic(std::string name, std::string type)
{
  bagwiz::io::TopicInfo t;
  t.name = std::move(name);
  t.type = std::move(type);
  t.serialization_format = "cdr";
  return t;
}

std::span<const std::byte> payload_view()
{
  static_assert(sizeof(std::uint8_t) == sizeof(std::byte));
  return std::span<const std::byte>(
    reinterpret_cast<const std::byte *>(
      kPayload.data()),  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    kPayload.size());
}

bagwiz::io::CreateOptions mcap_dir_opts()
{
  bagwiz::io::CreateOptions opts;
  opts.format = bagwiz::io::Format::Mcap;
  opts.layout = bagwiz::io::Layout::Directory;
  opts.mcap_compression = "none";
  return opts;
}

class BagCopyFilteredTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("bagwiz_bag_copy_" +
                std::to_string(::testing::UnitTest::GetInstance()->random_seed()) + "_" +
                std::to_string(
                  reinterpret_cast<std::uintptr_t>(
                    this)));  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    std::filesystem::create_directories(tmp_dir_);
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }

  std::filesystem::path tmp_dir_;
};

// Build a small MCAP bag with three messages across two topics.
std::filesystem::path build_input(const std::filesystem::path & dir)
{
  const auto path = dir / "input";
  auto writer = bagwiz::io::open_write(path, mcap_dir_opts());
  writer->declare_topic(make_topic("/foo", "std_msgs/msg/String"));
  writer->declare_topic(make_topic("/bar", "std_msgs/msg/Int32"));
  writer->write("/foo", 1'000'000'000LL, payload_view());
  writer->write("/bar", 2'000'000'000LL, payload_view());
  writer->write("/foo", 3'000'000'000LL, payload_view());
  writer->close();
  return path;
}

}  // namespace

TEST_F(BagCopyFilteredTest, CopiesAllMessagesWhenSuppressIsEmpty)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "output";

  auto reader = bagwiz::io::open_read(in_path);
  auto writer = bagwiz::io::open_write(out_path, mcap_dir_opts());
  for (const auto & t : reader->topics()) {
    writer->declare_topic(t);
  }

  const auto counts =
    bagwiz::core::bag_copy_filtered(*reader, *writer, std::unordered_set<std::string>{});
  writer->close();

  EXPECT_EQ(counts.copied, 3U);
  EXPECT_EQ(counts.suppressed, 0U);

  auto verify = bagwiz::io::open_read(out_path);
  int total = 0;
  bagwiz::io::RawMessage raw;
  while (verify->next(raw)) {
    ++total;
  }
  EXPECT_EQ(total, 3);
}

TEST_F(BagCopyFilteredTest, SuppressesNamedTopic)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "output";

  auto reader = bagwiz::io::open_read(in_path);
  auto writer = bagwiz::io::open_write(out_path, mcap_dir_opts());
  for (const auto & t : reader->topics()) {
    writer->declare_topic(t);
  }

  const std::unordered_set<std::string> suppress{"/foo"};
  const auto counts = bagwiz::core::bag_copy_filtered(*reader, *writer, suppress);
  writer->close();

  EXPECT_EQ(counts.copied, 1U);
  EXPECT_EQ(counts.suppressed, 2U);

  auto verify = bagwiz::io::open_read(out_path);
  int foo = 0;
  int bar = 0;
  bagwiz::io::RawMessage raw;
  while (verify->next(raw)) {
    if (raw.topic->name == "/foo") {
      ++foo;
    } else if (raw.topic->name == "/bar") {
      ++bar;
    }
  }
  EXPECT_EQ(foo, 0);
  EXPECT_EQ(bar, 1);
}

TEST_F(BagCopyFilteredTest, SuppressesAllWhenEveryTopicIsListed)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "output";

  auto reader = bagwiz::io::open_read(in_path);
  auto writer = bagwiz::io::open_write(out_path, mcap_dir_opts());
  for (const auto & t : reader->topics()) {
    writer->declare_topic(t);
  }

  const std::unordered_set<std::string> suppress{"/foo", "/bar"};
  const auto counts = bagwiz::core::bag_copy_filtered(*reader, *writer, suppress);
  writer->close();

  EXPECT_EQ(counts.copied, 0U);
  EXPECT_EQ(counts.suppressed, 3U);
}

// Reuses the BagCopyFilteredTest fixture helpers (same tmp-dir setup and
// build_input); the rename path is exercised against the same /foo + /bar bag.
using BagCopyRenamedTest = BagCopyFilteredTest;

TEST_F(BagCopyRenamedTest, RewritesMessagesUnderTheNewName)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "output";

  auto reader = bagwiz::io::open_read(in_path);
  auto writer = bagwiz::io::open_write(out_path, mcap_dir_opts());
  // Declare /foo under its new name; everything else verbatim. The writer
  // rejects writes to undeclared topics, so the destination must be declared
  // before bag_copy_renamed runs.
  for (const auto & t : reader->topics()) {
    if (t.name == "/foo") {
      auto renamed = t;
      renamed.name = "/renamed";
      writer->declare_topic(renamed);
    } else {
      writer->declare_topic(t);
    }
  }

  const std::unordered_map<std::string, std::string> rename{{"/foo", "/renamed"}};
  const auto counts = bagwiz::core::bag_copy_renamed(*reader, *writer, rename);
  writer->close();

  EXPECT_EQ(counts.copied, 3U);
  EXPECT_EQ(counts.renamed, 2U);

  auto verify = bagwiz::io::open_read(out_path);
  int foo = 0;
  int renamed_count = 0;
  int bar = 0;
  bagwiz::io::RawMessage raw;
  while (verify->next(raw)) {
    if (raw.topic->name == "/foo") {
      ++foo;
    } else if (raw.topic->name == "/renamed") {
      ++renamed_count;
    } else if (raw.topic->name == "/bar") {
      ++bar;
    }
  }
  EXPECT_EQ(foo, 0);            // the old name is gone
  EXPECT_EQ(renamed_count, 2);  // both /foo messages now carry the new name
  EXPECT_EQ(bar, 1);            // untouched topic copied verbatim
}

TEST_F(BagCopyRenamedTest, LeavesNamesUnchangedWhenRenameIsEmpty)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "output";

  auto reader = bagwiz::io::open_read(in_path);
  auto writer = bagwiz::io::open_write(out_path, mcap_dir_opts());
  for (const auto & t : reader->topics()) {
    writer->declare_topic(t);
  }

  const auto counts = bagwiz::core::bag_copy_renamed(
    *reader, *writer, std::unordered_map<std::string, std::string>{});
  writer->close();

  EXPECT_EQ(counts.copied, 3U);
  EXPECT_EQ(counts.renamed, 0U);

  auto verify = bagwiz::io::open_read(out_path);
  int foo = 0;
  int bar = 0;
  bagwiz::io::RawMessage raw;
  while (verify->next(raw)) {
    if (raw.topic->name == "/foo") {
      ++foo;
    } else if (raw.topic->name == "/bar") {
      ++bar;
    }
  }
  EXPECT_EQ(foo, 2);
  EXPECT_EQ(bar, 1);
}
