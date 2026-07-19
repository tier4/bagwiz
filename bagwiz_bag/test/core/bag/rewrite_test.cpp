// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/bag/rewrite.hpp"

#include "bagwiz/io/bag_io.hpp"

#include <mcap/reader.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

namespace
{

constexpr std::array<std::uint8_t, 4> kPayload{0x11, 0x22, 0x33, 0x44};
constexpr const char * kLogger = "bagwiz.test.rewrite";

std::span<const std::byte> payload_span()
{
  return {
    reinterpret_cast<const std::byte *>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
      kPayload.data()),
    kPayload.size()};
}

bagwiz::io::TopicInfo make_topic(const std::string & name)
{
  bagwiz::io::TopicInfo t;
  t.name = name;
  t.type = "std_msgs/msg/Int32";
  t.serialization_format = "cdr";
  return t;
}

// Materialise a small bag at `path` holding a single "/input" message.
void seed_bag(
  const std::filesystem::path & path, bagwiz::io::Format format, bagwiz::io::Layout layout)
{
  bagwiz::io::CreateOptions opts;
  opts.format = format;
  opts.layout = layout;
  opts.mcap_compression = "none";
  auto writer = bagwiz::io::open_write(path, opts);
  writer->declare_topic(make_topic("/input"));
  writer->write("/input", 1'000'000'000LL, payload_span());
  writer->close();
}

// The stand-in for a command's pass: write a fresh one-topic bag through the
// injected factory and report success. The dispatch under test owns which
// path the factory targets (-o output or in-place tmp).
int write_replacement_pass(const bagwiz::io::WriterFactory & open_writer)
{
  auto writer = open_writer();
  writer->declare_topic(make_topic("/rewritten"));
  writer->write("/rewritten", 2'000'000'000LL, payload_span());
  writer->close();
  return 0;
}

std::string read_file_bytes(const std::filesystem::path & path)
{
  std::ifstream in(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

// A pass writing a payload big and repetitive enough that zstd actually
// shrinks it. libmcap stores a chunk uncompressed when compression would not
// pay for itself, so the few-byte payload the other tests use would report
// "no compression" even with zstd correctly requested.
int write_compressible_replacement_pass(const bagwiz::io::WriterFactory & open_writer)
{
  const std::vector<std::byte> big(64 * 1024, std::byte{0xAB});
  auto writer = open_writer();
  writer->declare_topic(make_topic("/rewritten"));
  writer->write("/rewritten", 2'000'000'000LL, {big.data(), big.size()});
  writer->close();
  return 0;
}

// The compression recorded on each chunk of an MCAP file, deduplicated. An
// empty string is mcap::Compression::None — that is what the writer emits when
// CreateOptions::mcap_compression is "none", so it is the value a
// disable_mcap_compression rewrite must produce.
std::set<std::string> mcap_chunk_compressions(const std::filesystem::path & path)
{
  mcap::McapReader reader;
  const auto open_status = reader.open(path.string());
  EXPECT_TRUE(open_status.ok()) << open_status.message;
  const auto summary_status = reader.readSummary(mcap::ReadSummaryMethod::AllowFallbackScan);
  EXPECT_TRUE(summary_status.ok()) << summary_status.message;

  std::set<std::string> compressions;
  for (const auto & index : reader.chunkIndexes()) {
    compressions.insert(index.compression);
  }
  reader.close();
  return compressions;
}

bool tmp_leftover_in(const std::filesystem::path & dir)
{
  for (const auto & entry : std::filesystem::directory_iterator(dir)) {
    if (entry.path().filename().string().find(".bagwiz-inplace-tmp-") != std::string::npos) {
      return true;
    }
  }
  return false;
}

class RewriteTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ =
      std::filesystem::temp_directory_path() /
      ("bagwiz_rewrite_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()));
    std::filesystem::create_directories(tmp_dir_);

    options_.logger = kLogger;
    options_.format_unknown_error = "test: could not detect storage format of input bag '%s'.";
    options_.pass_failed_error = "test: pass failed; aborting in-place swap";
    options_.inherit_output_format = true;
    options_.disable_mcap_compression = true;
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }

  std::filesystem::path tmp_dir_;
  bagwiz::core::BagRewriteOptions options_;
};

}  // namespace

TEST_F(RewriteTest, OutputModeWritesNewBagAndLeavesInputUntouched)
{
  const auto input = tmp_dir_ / "input.mcap";
  const auto output = tmp_dir_ / "output.mcap";
  seed_bag(input, bagwiz::io::Format::Mcap, bagwiz::io::Layout::SingleFile);
  const auto input_before = read_file_bytes(input);

  int pass_calls = 0;
  const int status = bagwiz::core::run_bag_rewrite(
    input, output, /*overwrite=*/false, options_, [&](const bagwiz::io::WriterFactory & factory) {
      ++pass_calls;
      return write_replacement_pass(factory);
    });

  EXPECT_EQ(status, 0);
  EXPECT_EQ(pass_calls, 1);
  EXPECT_EQ(read_file_bytes(input), input_before);
  const auto reader = bagwiz::io::open_read(output);
  ASSERT_EQ(reader->topics().size(), 1u);
  EXPECT_EQ(reader->topics()[0].name, "/rewritten");
}

TEST_F(RewriteTest, OutputModeFailsWhenOutputExistsWithoutOverwrite)
{
  const auto input = tmp_dir_ / "input.mcap";
  const auto output = tmp_dir_ / "output.mcap";
  seed_bag(input, bagwiz::io::Format::Mcap, bagwiz::io::Layout::SingleFile);
  {
    std::ofstream out(output);
    out << "PRE-EXISTING";
  }

  int pass_calls = 0;
  const int status = bagwiz::core::run_bag_rewrite(
    input, output, /*overwrite=*/false, options_, [&](const bagwiz::io::WriterFactory & factory) {
      ++pass_calls;
      return write_replacement_pass(factory);
    });

  EXPECT_EQ(status, 1);
  EXPECT_EQ(pass_calls, 0);  // prepare_output_path fails before the pass runs
  EXPECT_EQ(read_file_bytes(output), "PRE-EXISTING");
}

TEST_F(RewriteTest, OutputModeOverwriteReplacesExisting)
{
  const auto input = tmp_dir_ / "input.mcap";
  const auto output = tmp_dir_ / "output.mcap";
  seed_bag(input, bagwiz::io::Format::Mcap, bagwiz::io::Layout::SingleFile);
  {
    std::ofstream out(output);
    out << "PRE-EXISTING";
  }

  const int status = bagwiz::core::run_bag_rewrite(
    input, output, /*overwrite=*/true, options_, write_replacement_pass);

  EXPECT_EQ(status, 0);
  const auto reader = bagwiz::io::open_read(output);
  ASSERT_EQ(reader->topics().size(), 1u);
  EXPECT_EQ(reader->topics()[0].name, "/rewritten");
}

TEST_F(RewriteTest, OutputModeDirectoryOutputInheritsInputFormat)
{
  const auto input = tmp_dir_ / "input_db3";
  const auto output = tmp_dir_ / "output_dir";
  seed_bag(input, bagwiz::io::Format::Sqlite3, bagwiz::io::Layout::Directory);

  const int status = bagwiz::core::run_bag_rewrite(
    input, output, /*overwrite=*/false, options_, write_replacement_pass);

  EXPECT_EQ(status, 0);
  ASSERT_TRUE(std::filesystem::is_directory(output));
  EXPECT_EQ(bagwiz::io::detect_format(output), bagwiz::io::Format::Sqlite3);
}

TEST_F(RewriteTest, OutputModeDirectoryOutputWithoutInheritUsesFactoryDefault)
{
  const auto input = tmp_dir_ / "input_db3";
  const auto output = tmp_dir_ / "output_dir";
  seed_bag(input, bagwiz::io::Format::Sqlite3, bagwiz::io::Layout::Directory);
  options_.inherit_output_format = false;

  const int status = bagwiz::core::run_bag_rewrite(
    input, output, /*overwrite=*/false, options_, write_replacement_pass);

  EXPECT_EQ(status, 0);
  ASSERT_TRUE(std::filesystem::is_directory(output));
  // Auto/Auto resolves to the factory default (MCAP) regardless of the input.
  EXPECT_EQ(bagwiz::io::detect_format(output), bagwiz::io::Format::Mcap);
}

TEST_F(RewriteTest, OutputModeSingleFileExtensionWinsOverInherit)
{
  const auto input = tmp_dir_ / "input.mcap";
  const auto output = tmp_dir_ / "output.db3";
  seed_bag(input, bagwiz::io::Format::Mcap, bagwiz::io::Layout::SingleFile);

  const int status = bagwiz::core::run_bag_rewrite(
    input, output, /*overwrite=*/false, options_, write_replacement_pass);

  EXPECT_EQ(status, 0);
  ASSERT_FALSE(std::filesystem::is_directory(output));
  EXPECT_EQ(bagwiz::io::detect_format(output), bagwiz::io::Format::Sqlite3);
}

TEST_F(RewriteTest, InPlaceRewritesInputAndPreservesMcapSingleFile)
{
  const auto input = tmp_dir_ / "input.mcap";
  seed_bag(input, bagwiz::io::Format::Mcap, bagwiz::io::Layout::SingleFile);

  const int status = bagwiz::core::run_bag_rewrite(
    input, std::nullopt, /*overwrite=*/false, options_, write_replacement_pass);

  EXPECT_EQ(status, 0);
  ASSERT_FALSE(std::filesystem::is_directory(input));
  EXPECT_EQ(bagwiz::io::detect_format(input), bagwiz::io::Format::Mcap);
  const auto reader = bagwiz::io::open_read(input);
  ASSERT_EQ(reader->topics().size(), 1u);
  EXPECT_EQ(reader->topics()[0].name, "/rewritten");
  EXPECT_FALSE(tmp_leftover_in(tmp_dir_));
}

TEST_F(RewriteTest, InPlacePreservesSqlite3SingleFile)
{
  const auto input = tmp_dir_ / "input.db3";
  seed_bag(input, bagwiz::io::Format::Sqlite3, bagwiz::io::Layout::SingleFile);

  const int status = bagwiz::core::run_bag_rewrite(
    input, std::nullopt, /*overwrite=*/false, options_, write_replacement_pass);

  EXPECT_EQ(status, 0);
  ASSERT_FALSE(std::filesystem::is_directory(input));
  EXPECT_EQ(bagwiz::io::detect_format(input), bagwiz::io::Format::Sqlite3);
}

TEST_F(RewriteTest, InPlacePassFailureReturnsStatusAndLeavesInputUntouched)
{
  const auto input = tmp_dir_ / "input.mcap";
  seed_bag(input, bagwiz::io::Format::Mcap, bagwiz::io::Layout::SingleFile);
  const auto input_before = read_file_bytes(input);

  // The pass fully writes the replacement bag but still reports failure; the
  // swap must not happen and the pass's exit code must propagate.
  const int status = bagwiz::core::run_bag_rewrite(
    input, std::nullopt, /*overwrite=*/false, options_,
    [](const bagwiz::io::WriterFactory & factory) {
      write_replacement_pass(factory);
      return 3;
    });

  EXPECT_EQ(status, 3);
  EXPECT_EQ(read_file_bytes(input), input_before);
  EXPECT_FALSE(tmp_leftover_in(tmp_dir_));
}

TEST_F(RewriteTest, InPlacePassThrowReturnsOneAndLeavesInputUntouched)
{
  const auto input = tmp_dir_ / "input.mcap";
  seed_bag(input, bagwiz::io::Format::Mcap, bagwiz::io::Layout::SingleFile);
  const auto input_before = read_file_bytes(input);

  const int status = bagwiz::core::run_bag_rewrite(
    input, std::nullopt, /*overwrite=*/false, options_,
    [](const bagwiz::io::WriterFactory &) -> int { throw std::runtime_error("pass exploded"); });

  EXPECT_EQ(status, 1);
  EXPECT_EQ(read_file_bytes(input), input_before);
  EXPECT_FALSE(tmp_leftover_in(tmp_dir_));
}

TEST_F(RewriteTest, InPlaceFormatAutoGuardRejectsNonBag)
{
  const auto input = tmp_dir_ / "notes.txt";
  {
    std::ofstream out(input);
    out << "this is not a bag";
  }

  int pass_calls = 0;
  const int status = bagwiz::core::run_bag_rewrite(
    input, std::nullopt, /*overwrite=*/false, options_,
    [&](const bagwiz::io::WriterFactory & factory) {
      ++pass_calls;
      return write_replacement_pass(factory);
    });

  EXPECT_EQ(status, 1);
  EXPECT_EQ(pass_calls, 0);
  EXPECT_EQ(read_file_bytes(input), "this is not a bag");
  EXPECT_FALSE(tmp_leftover_in(tmp_dir_));
}

// ---------------------------------------------------------------------------
// MCAP compression wiring.
//
// Every rewrite command drives the writer through run_bag_rewrite, and all of
// them but one take the default disable_mcap_compression = true, which pins
// mcap_compression to "none". `pcd undistort` is the sole opt-out: it sets the
// flag false so the writer keeps its zstd default. The option is easy to flip
// by accident and the effect is invisible in a bag's topics/messages, so these
// tests read the produced MCAP's chunk compression directly.
// ---------------------------------------------------------------------------

TEST(BagRewriteOptionsDefaults, DisableMcapCompressionDefaultsToTrue)
{
  // Every rewrite command except `pcd undistort` relies on this default, so a
  // new command that never touches the field still writes uncompressed.
  EXPECT_TRUE(bagwiz::core::BagRewriteOptions{}.disable_mcap_compression);
}

TEST_F(RewriteTest, OutputModeDisablingCompressionWritesUncompressedChunks)
{
  const auto input = tmp_dir_ / "input.mcap";
  const auto output = tmp_dir_ / "output.mcap";
  seed_bag(input, bagwiz::io::Format::Mcap, bagwiz::io::Layout::SingleFile);
  ASSERT_TRUE(options_.disable_mcap_compression);

  ASSERT_EQ(
    bagwiz::core::run_bag_rewrite(
      input, output, /*overwrite=*/false, options_, write_compressible_replacement_pass),
    0);

  EXPECT_EQ(mcap_chunk_compressions(output), (std::set<std::string>{""}));
}

TEST_F(RewriteTest, OutputModeKeepingCompressionWritesZstdChunks)
{
  const auto input = tmp_dir_ / "input.mcap";
  const auto output = tmp_dir_ / "output.mcap";
  seed_bag(input, bagwiz::io::Format::Mcap, bagwiz::io::Layout::SingleFile);
  // The `pcd undistort` configuration: leave the writer's zstd default alone.
  options_.disable_mcap_compression = false;

  ASSERT_EQ(
    bagwiz::core::run_bag_rewrite(
      input, output, /*overwrite=*/false, options_, write_compressible_replacement_pass),
    0);

  EXPECT_EQ(mcap_chunk_compressions(output), (std::set<std::string>{"zstd"}));
}

TEST_F(RewriteTest, InPlaceDisablingCompressionWritesUncompressedChunks)
{
  const auto input = tmp_dir_ / "input.mcap";
  seed_bag(input, bagwiz::io::Format::Mcap, bagwiz::io::Layout::SingleFile);
  ASSERT_TRUE(options_.disable_mcap_compression);

  ASSERT_EQ(
    bagwiz::core::run_bag_rewrite(
      input, std::nullopt, /*overwrite=*/false, options_, write_compressible_replacement_pass),
    0);

  EXPECT_EQ(mcap_chunk_compressions(input), (std::set<std::string>{""}));
}

TEST_F(RewriteTest, InPlaceKeepingCompressionWritesZstdChunks)
{
  const auto input = tmp_dir_ / "input.mcap";
  seed_bag(input, bagwiz::io::Format::Mcap, bagwiz::io::Layout::SingleFile);
  options_.disable_mcap_compression = false;

  ASSERT_EQ(
    bagwiz::core::run_bag_rewrite(
      input, std::nullopt, /*overwrite=*/false, options_, write_compressible_replacement_pass),
    0);

  // create_options_preserving_storage pins format/layout but leaves
  // mcap_compression at the CreateOptions default, so the in-place path keeps
  // zstd exactly like the -o path.
  EXPECT_EQ(mcap_chunk_compressions(input), (std::set<std::string>{"zstd"}));
}
