// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/topic_drop.hpp"

#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/metadata_yaml.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

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
    reinterpret_cast<const std::byte *>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
      kPayload.data()),
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

// Build an MCAP directory bag with three topics:
//   /sensing/camera     (2 messages)
//   /sensing/lidar      (1 message)
//   /perception/objects (1 message)
std::filesystem::path build_input(const std::filesystem::path & dir)
{
  const auto path = dir / "input";
  auto writer = bagwiz::io::open_write(path, mcap_dir_opts());
  writer->declare_topic(make_topic("/sensing/camera", "sensor_msgs/msg/Image"));
  writer->declare_topic(make_topic("/sensing/lidar", "sensor_msgs/msg/PointCloud2"));
  writer->declare_topic(make_topic("/perception/objects", "std_msgs/msg/String"));
  writer->write("/sensing/camera", 1'000'000'000LL, payload_view());
  writer->write("/sensing/lidar", 2'000'000'000LL, payload_view());
  writer->write("/perception/objects", 3'000'000'000LL, payload_view());
  writer->write("/sensing/camera", 4'000'000'000LL, payload_view());
  writer->close();
  return path;
}

// Map of declared-topic -> message count for the bag at `path`. A topic that
// was dropped is absent from the map entirely (neither declared nor carrying
// messages); a kept topic maps to its message count.
std::map<std::string, int> collect(const std::filesystem::path & path)
{
  auto reader = bagwiz::io::open_read(path);
  std::map<std::string, int> counts;
  for (const auto & t : reader->topics()) {
    counts[t.name];  // ensure declared topics appear, even at zero messages
  }
  bagwiz::io::RawMessage raw;
  while (reader->next(raw)) {
    if (raw.topic != nullptr) {
      ++counts[raw.topic->name];
    }
  }
  return counts;
}

class TopicDropTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ =
      std::filesystem::temp_directory_path() /
      ("bagwiz_topic_drop_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) +
       "_" +
       std::to_string(
         reinterpret_cast<std::uintptr_t>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
           this)));
    std::filesystem::create_directories(tmp_dir_);
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }

  std::filesystem::path tmp_dir_;
};

TEST_F(TopicDropTest, DropExactTopicToOutput)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";

  bagwiz::commands::TopicDropArgs args;
  args.input_path = in_path;
  args.topics = {"/sensing/lidar"};
  args.output_path = out_path;

  ASSERT_EQ(bagwiz::commands::run_topic_drop(args), 0);

  const auto out = collect(out_path);
  EXPECT_EQ(out.count("/sensing/lidar"), 0U);  // dropped: not even declared
  EXPECT_EQ(out.at("/sensing/camera"), 2);
  EXPECT_EQ(out.at("/perception/objects"), 1);

  // The input bag is untouched in -o mode.
  const auto in = collect(in_path);
  EXPECT_EQ(in.count("/sensing/lidar"), 1U);
  EXPECT_EQ(in.at("/sensing/lidar"), 1);
}

TEST_F(TopicDropTest, DropWildcardDropsMatchingSubtree)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";

  bagwiz::commands::TopicDropArgs args;
  args.input_path = in_path;
  args.topics = {"/sensing/*"};
  args.output_path = out_path;

  ASSERT_EQ(bagwiz::commands::run_topic_drop(args), 0);

  const auto out = collect(out_path);
  EXPECT_EQ(out.count("/sensing/camera"), 0U);
  EXPECT_EQ(out.count("/sensing/lidar"), 0U);
  ASSERT_EQ(out.count("/perception/objects"), 1U);
  EXPECT_EQ(out.at("/perception/objects"), 1);
}

TEST_F(TopicDropTest, DropMultipleSelectors)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";

  bagwiz::commands::TopicDropArgs args;
  args.input_path = in_path;
  args.topics = {"/sensing/camera", "*/objects"};
  args.output_path = out_path;

  ASSERT_EQ(bagwiz::commands::run_topic_drop(args), 0);

  const auto out = collect(out_path);
  EXPECT_EQ(out.count("/sensing/camera"), 0U);
  EXPECT_EQ(out.count("/perception/objects"), 0U);
  ASSERT_EQ(out.count("/sensing/lidar"), 1U);
  EXPECT_EQ(out.at("/sensing/lidar"), 1);
}

TEST_F(TopicDropTest, DropInPlaceRewritesInput)
{
  const auto in_path = build_input(tmp_dir_);

  bagwiz::commands::TopicDropArgs args;
  args.input_path = in_path;
  args.topics = {"/sensing/lidar"};
  // No output_path -> in-place.

  ASSERT_EQ(bagwiz::commands::run_topic_drop(args), 0);

  const auto result = collect(in_path);
  EXPECT_EQ(result.count("/sensing/lidar"), 0U);
  EXPECT_EQ(result.at("/sensing/camera"), 2);
  EXPECT_EQ(result.at("/perception/objects"), 1);
}

TEST_F(TopicDropTest, UnmatchedSelectorFailsWithoutWriting)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";

  bagwiz::commands::TopicDropArgs args;
  args.input_path = in_path;
  args.topics = {"/does/not/exist"};
  args.output_path = out_path;

  EXPECT_EQ(bagwiz::commands::run_topic_drop(args), 1);
  EXPECT_FALSE(std::filesystem::exists(out_path));

  // The input is left fully intact.
  const auto in = collect(in_path);
  EXPECT_EQ(in.size(), 3U);
}

TEST_F(TopicDropTest, EmptySelectorListFailsWithoutWriting)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";

  bagwiz::commands::TopicDropArgs args;
  args.input_path = in_path;
  args.topics = {};  // the CLI forbids this, but the API must not silently no-op
  args.output_path = out_path;

  EXPECT_EQ(bagwiz::commands::run_topic_drop(args), 1);
  EXPECT_FALSE(std::filesystem::exists(out_path));

  // The input is left fully intact.
  const auto in = collect(in_path);
  EXPECT_EQ(in.size(), 3U);
}

TEST_F(TopicDropTest, DropAllTopicsProducesEmptyBag)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";

  bagwiz::commands::TopicDropArgs args;
  args.input_path = in_path;
  args.topics = {"*"};
  args.output_path = out_path;

  ASSERT_EQ(bagwiz::commands::run_topic_drop(args), 0);

  const auto out = collect(out_path);
  EXPECT_TRUE(out.empty());
}

TEST_F(TopicDropTest, ExistingOutputWithoutOverwriteFails)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";
  std::filesystem::create_directories(out_path);  // pre-existing collision

  bagwiz::commands::TopicDropArgs args;
  args.input_path = in_path;
  args.topics = {"/sensing/lidar"};
  args.output_path = out_path;
  args.overwrite = false;

  EXPECT_EQ(bagwiz::commands::run_topic_drop(args), 1);
}

TEST_F(TopicDropTest, OverwriteReplacesExistingOutput)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";
  std::filesystem::create_directories(out_path);

  bagwiz::commands::TopicDropArgs args;
  args.input_path = in_path;
  args.topics = {"/sensing/lidar"};
  args.output_path = out_path;
  args.overwrite = true;

  ASSERT_EQ(bagwiz::commands::run_topic_drop(args), 0);

  const auto out = collect(out_path);
  EXPECT_EQ(out.count("/sensing/lidar"), 0U);
  EXPECT_EQ(out.at("/sensing/camera"), 2);
}

// The default path (chunk pass-through) and the decoded pipeline
// (BAGWIZ_PASSTHROUGH=off) must produce the same bag content — and only the
// pass-through preserves the input's chunk compression.
TEST_F(TopicDropTest, PassthroughMatchesPipelineAndPreservesCompression)
{
  const auto in_path = tmp_dir_ / "input_zstd";
  {
    auto opts = mcap_dir_opts();
    opts.mcap_compression = "zstd";
    auto writer = bagwiz::io::open_write(in_path, opts);
    writer->declare_topic(make_topic("/sensing/camera", "sensor_msgs/msg/Image"));
    writer->declare_topic(make_topic("/sensing/lidar", "sensor_msgs/msg/PointCloud2"));
    // Large compressible payloads: libmcap silently stores chunks whose
    // payload does not shrink as uncompressed, which would defeat the
    // compression-preservation assertion below.
    const std::vector<std::byte> big(2048, std::byte{0x42});
    const std::span<const std::byte> big_view(big.data(), big.size());
    writer->write("/sensing/camera", 1'000'000'000LL, big_view);
    writer->write("/sensing/lidar", 2'000'000'000LL, big_view);
    writer->write("/sensing/camera", 3'000'000'000LL, big_view);
    writer->close();
  }

  bagwiz::commands::TopicDropArgs args;
  args.input_path = in_path;
  args.topics = {"/sensing/lidar"};

  ::setenv("BAGWIZ_PASSTHROUGH", "off", 1);
  args.output_path = tmp_dir_ / "ref";
  ASSERT_EQ(bagwiz::commands::run_topic_drop(args), 0);
  ::unsetenv("BAGWIZ_PASSTHROUGH");
  args.output_path = tmp_dir_ / "out";
  ASSERT_EQ(bagwiz::commands::run_topic_drop(args), 0);

  EXPECT_EQ(collect(tmp_dir_ / "ref"), collect(tmp_dir_ / "out"));

  // The decoded pipeline still forces compression off; the pass-through
  // keeps the input's zstd chunks (visible in the directory metadata).
  EXPECT_EQ(
    bagwiz::io::load_metadata_yaml(tmp_dir_ / "ref" / "metadata.yaml").compression_format, "none");
  EXPECT_EQ(
    bagwiz::io::load_metadata_yaml(tmp_dir_ / "out" / "metadata.yaml").compression_format, "zstd");
}

}  // namespace
