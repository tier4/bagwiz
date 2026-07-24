// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/bag/bag_passthrough.hpp"

#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/metadata_yaml.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <span>
#include <string>
#include <system_error>
#include <utility>

// The orchestrator must gate the chunk pass-through on everything the engine
// cannot see — the BAGWIZ_PASSTHROUGH kill switch, non-mcap inputs and
// outputs, split requests, MESSAGE-mode compression — and, when the engine
// runs on a directory target, produce a metadata.yaml equivalent to what the
// decoded pipeline's directory writer would emit.
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

bagwiz::io::CreateOptions dir_opts(bagwiz::io::Format format, const char * compression = "none")
{
  bagwiz::io::CreateOptions opts;
  opts.format = format;
  opts.layout = bagwiz::io::Layout::Directory;
  opts.mcap_compression = compression;
  return opts;
}

// A 3-message, 2-topic bag in the requested storage format and layout.
void build_input(const std::filesystem::path & path, const bagwiz::io::CreateOptions & opts)
{
  auto writer = bagwiz::io::open_write(path, opts);
  writer->declare_topic(make_topic("/foo", "std_msgs/msg/String"));
  writer->declare_topic(make_topic("/bar", "std_msgs/msg/Int32"));
  writer->write("/foo", 1'000'000'000LL, payload_view());
  writer->write("/bar", 2'000'000'000LL, payload_view());
  writer->write("/foo", 3'000'000'000LL, payload_view());
  writer->close();
}

bagwiz::core::RewriteTarget make_target(
  const std::filesystem::path & path, const bagwiz::io::CreateOptions & opts)
{
  return bagwiz::core::RewriteTarget{path, opts};
}

// Replace one metadata.yaml line with an explicit value, to fabricate
// metadata shapes bagwiz itself never writes.
void patch_metadata_value(
  const std::filesystem::path & yaml_path, const std::string & key, const std::string & old_value,
  const std::string & value)
{
  std::string text;
  {
    std::ifstream f(yaml_path);
    text.assign((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  }
  const std::string needle = key + ": " + old_value;
  const auto pos = text.find(needle);
  ASSERT_NE(pos, std::string::npos) << key;
  text.replace(pos, needle.size(), key + ": " + value);
  std::ofstream f(yaml_path);
  f << text;
}

class BagPassthroughTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("bagwiz_bag_passthrough_" +
                std::to_string(::testing::UnitTest::GetInstance()->random_seed()) + "_" +
                std::to_string(
                  reinterpret_cast<std::uintptr_t>(
                    this)));  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    std::filesystem::create_directories(tmp_dir_);
    ::unsetenv("BAGWIZ_PASSTHROUGH");
  }

  void TearDown() override
  {
    ::unsetenv("BAGWIZ_PASSTHROUGH");
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }

  std::filesystem::path tmp_dir_;
};

}  // namespace

TEST_F(BagPassthroughTest, RunsOnMcapDirectoryInputAndWritesMetadataYaml)
{
  const auto input = tmp_dir_ / "input";
  build_input(input, dir_opts(bagwiz::io::Format::Mcap, "zstd"));
  const auto output = tmp_dir_ / "output";

  bagwiz::core::PassthroughEdit edit;
  edit.suppress_topics = {"/bar"};
  const auto counts = bagwiz::core::try_bag_passthrough_rewrite(
    input, make_target(output, dir_opts(bagwiz::io::Format::Mcap)), edit, "bagwiz.test");
  ASSERT_TRUE(counts.has_value());
  EXPECT_EQ(counts->copied, 2u);
  EXPECT_EQ(counts->renamed, 0u);

  // The output must be a self-sufficient rosbag2 directory bag: shard named
  // like the directory, metadata.yaml with the surviving topic only.
  EXPECT_TRUE(std::filesystem::exists(output / "output_0.mcap"));
  const auto metadata = bagwiz::io::load_metadata_yaml(output / "metadata.yaml");
  EXPECT_EQ(metadata.storage_identifier, "mcap");
  ASSERT_EQ(metadata.topics.size(), 1u);
  EXPECT_EQ(metadata.topics[0].name, "/foo");
  EXPECT_TRUE(metadata.has_summary);
  EXPECT_EQ(metadata.total_messages, 2);
  EXPECT_EQ(metadata.per_topic_counts.at("/foo"), 2);

  auto reader = bagwiz::io::open_read(output);
  bagwiz::io::RawMessage raw;
  int total = 0;
  while (reader->next(raw)) {
    EXPECT_EQ(raw.topic->name, "/foo");
    ++total;
  }
  EXPECT_EQ(total, 2);
}

TEST_F(BagPassthroughTest, RenameCountsRenamedMessages)
{
  const auto input = tmp_dir_ / "input";
  build_input(input, dir_opts(bagwiz::io::Format::Mcap));
  const auto output = tmp_dir_ / "output";

  bagwiz::core::PassthroughEdit edit;
  edit.rename = {{"/foo", "/foo2"}};
  const auto counts = bagwiz::core::try_bag_passthrough_rewrite(
    input, make_target(output, dir_opts(bagwiz::io::Format::Mcap)), edit, "bagwiz.test");
  ASSERT_TRUE(counts.has_value());
  EXPECT_EQ(counts->copied, 3u);
  EXPECT_EQ(counts->renamed, 2u);

  const auto metadata = bagwiz::io::load_metadata_yaml(output / "metadata.yaml");
  ASSERT_EQ(metadata.topics.size(), 2u);
  EXPECT_EQ(metadata.per_topic_counts.at("/foo2"), 2);
}

TEST_F(BagPassthroughTest, EnvKillSwitchDeclines)
{
  const auto input = tmp_dir_ / "input";
  build_input(input, dir_opts(bagwiz::io::Format::Mcap));
  const auto output = tmp_dir_ / "output";

  ::setenv("BAGWIZ_PASSTHROUGH", "off", 1);
  const auto counts = bagwiz::core::try_bag_passthrough_rewrite(
    input, make_target(output, dir_opts(bagwiz::io::Format::Mcap)), {}, "bagwiz.test");
  EXPECT_FALSE(counts.has_value());
  EXPECT_FALSE(std::filesystem::exists(output));
}

TEST_F(BagPassthroughTest, SqliteInputDeclines)
{
  const auto input = tmp_dir_ / "input";
  build_input(input, dir_opts(bagwiz::io::Format::Sqlite3));
  const auto output = tmp_dir_ / "output";

  const auto counts = bagwiz::core::try_bag_passthrough_rewrite(
    input, make_target(output, dir_opts(bagwiz::io::Format::Sqlite3)), {}, "bagwiz.test");
  EXPECT_FALSE(counts.has_value());
  EXPECT_FALSE(std::filesystem::exists(output));
}

TEST_F(BagPassthroughTest, SqliteOutputDeclines)
{
  const auto input = tmp_dir_ / "input";
  build_input(input, dir_opts(bagwiz::io::Format::Mcap));
  const auto output = tmp_dir_ / "output.db3";

  bagwiz::io::CreateOptions opts;
  opts.format = bagwiz::io::Format::Auto;
  opts.layout = bagwiz::io::Layout::Auto;
  const auto counts =
    bagwiz::core::try_bag_passthrough_rewrite(input, make_target(output, opts), {}, "bagwiz.test");
  EXPECT_FALSE(counts.has_value());
  EXPECT_FALSE(std::filesystem::exists(output));
}

TEST_F(BagPassthroughTest, SplitRequestDeclines)
{
  const auto input = tmp_dir_ / "input";
  build_input(input, dir_opts(bagwiz::io::Format::Mcap));
  const auto output = tmp_dir_ / "output";

  auto opts = dir_opts(bagwiz::io::Format::Mcap);
  opts.split_bytes = 1024;
  const auto counts =
    bagwiz::core::try_bag_passthrough_rewrite(input, make_target(output, opts), {}, "bagwiz.test");
  EXPECT_FALSE(counts.has_value());
  EXPECT_FALSE(std::filesystem::exists(output));
}

TEST_F(BagPassthroughTest, MessageModeCompressionDeclines)
{
  const auto input = tmp_dir_ / "input";
  build_input(input, dir_opts(bagwiz::io::Format::Mcap));
  // Stamp MESSAGE-mode compression into the metadata, as rosbag2's
  // per-message compression would. The payloads are not actually
  // compressed, but the gate must decline on the metadata alone.
  patch_metadata_value(input / "metadata.yaml", "compression_format", "none", "zstd");
  patch_metadata_value(input / "metadata.yaml", "compression_mode", "\"\"", "message");

  const auto output = tmp_dir_ / "output";
  const auto counts = bagwiz::core::try_bag_passthrough_rewrite(
    input, make_target(output, dir_opts(bagwiz::io::Format::Mcap)), {}, "bagwiz.test");
  EXPECT_FALSE(counts.has_value());
  EXPECT_FALSE(std::filesystem::exists(output));
}

TEST_F(BagPassthroughTest, SingleFileInputToSingleFileOutput)
{
  const auto input = tmp_dir_ / "input.mcap";
  bagwiz::io::CreateOptions in_opts;
  in_opts.format = bagwiz::io::Format::Mcap;
  in_opts.layout = bagwiz::io::Layout::SingleFile;
  in_opts.mcap_compression = "zstd";
  build_input(input, in_opts);

  const auto output = tmp_dir_ / "output.mcap";
  bagwiz::io::CreateOptions out_opts;
  out_opts.format = bagwiz::io::Format::Auto;
  out_opts.layout = bagwiz::io::Layout::Auto;
  const auto counts = bagwiz::core::try_bag_passthrough_rewrite(
    input, make_target(output, out_opts), {}, "bagwiz.test");
  ASSERT_TRUE(counts.has_value());
  EXPECT_EQ(counts->copied, 3u);
  EXPECT_TRUE(std::filesystem::exists(output));
  EXPECT_FALSE(std::filesystem::exists(output / "metadata.yaml"));  // single file, no dir

  auto reader = bagwiz::io::open_read(output);
  bagwiz::io::RawMessage raw;
  int total = 0;
  while (reader->next(raw)) {
    ++total;
  }
  EXPECT_EQ(total, 3);
}

TEST_F(BagPassthroughTest, EngineDeclineCleansUpCreatedDirectory)
{
  // Corrupt the shard after writing the metadata: the orchestrator's gates
  // pass, the output directory gets created, then the engine declines on
  // the unreadable mcap — and the directory must be removed so the decoded
  // fallback starts from a clean slate.
  const auto input = tmp_dir_ / "input";
  build_input(input, dir_opts(bagwiz::io::Format::Mcap));
  {
    std::ofstream f(input / "input_0.mcap", std::ios::binary | std::ios::trunc);
    f << "not an mcap";
  }

  const auto output = tmp_dir_ / "output";
  const auto counts = bagwiz::core::try_bag_passthrough_rewrite(
    input, make_target(output, dir_opts(bagwiz::io::Format::Mcap)), {}, "bagwiz.test");
  EXPECT_FALSE(counts.has_value());
  EXPECT_FALSE(std::filesystem::exists(output));
}
