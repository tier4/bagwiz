// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pipeline/rewrite_backend.hpp"

#include "bag_equal.hpp"  // NOLINT(build/include_subdir)  sibling test header, resolves relative
#include "bagwiz/core/pipeline/sequential_backend.hpp"
#include "bagwiz/core/pipeline/topic_router.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <span>
#include <string>
#include <system_error>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace
{

namespace pipeline = bagwiz::core::pipeline;

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

class RewriteBackendTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("bagwiz_rewrite_backend_" +
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

}  // namespace

TEST_F(RewriteBackendTest, SequentialSuppressDropsNamedTopic)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "output";

  auto reader = bagwiz::io::open_read(in_path);
  auto writer = bagwiz::io::open_write(out_path, mcap_dir_opts());
  const std::unordered_set<std::string> suppress{"/foo"};
  for (const auto & t : reader->topics()) {
    if (suppress.count(t.name) == 0) {
      writer->declare_topic(t);
    }
  }

  pipeline::SuppressRouter router(suppress);
  pipeline::SequentialBackend backend;
  const auto counts = pipeline::run_pipeline(*reader, *writer, router, backend, "test suppress");
  writer->close();

  EXPECT_EQ(counts.copied, 1U);
  EXPECT_EQ(counts.dropped, 2U);
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
  EXPECT_EQ(foo, 0);
  EXPECT_EQ(bar, 1);
}

TEST_F(RewriteBackendTest, SequentialRenameRewritesName)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "output";

  auto reader = bagwiz::io::open_read(in_path);
  auto writer = bagwiz::io::open_write(out_path, mcap_dir_opts());
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
  pipeline::RenameRouter router(rename);
  pipeline::SequentialBackend backend;
  const auto counts = pipeline::run_pipeline(*reader, *writer, router, backend, "test rename");
  writer->close();

  EXPECT_EQ(counts.copied, 3U);
  EXPECT_EQ(counts.renamed, 2U);
  EXPECT_EQ(counts.dropped, 0U);

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
  EXPECT_EQ(foo, 0);
  EXPECT_EQ(renamed_count, 2);
  EXPECT_EQ(bar, 1);
}

TEST_F(RewriteBackendTest, SequentialPassthroughIsByteIdentical)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "output";

  auto reader = bagwiz::io::open_read(in_path);
  auto writer = bagwiz::io::open_write(out_path, mcap_dir_opts());
  for (const auto & t : reader->topics()) {
    writer->declare_topic(t);
  }

  const std::unordered_set<std::string> empty;
  pipeline::SuppressRouter router(empty);
  pipeline::SequentialBackend backend;
  const auto counts = pipeline::run_pipeline(*reader, *writer, router, backend, "");
  writer->close();

  EXPECT_EQ(counts.copied, 3U);
  EXPECT_EQ(counts.dropped, 0U);
  EXPECT_EQ(counts.renamed, 0U);

  // A full-passthrough rewrite reproduces the input message stream exactly.
  bagwiz::test::expect_bags_equal(in_path, out_path);
}
