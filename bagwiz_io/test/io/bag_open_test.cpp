// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/bag_open.hpp"

#include "bagwiz/io/bag_io.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <system_error>

namespace
{

constexpr std::array<std::uint8_t, 4> kPayload{0xDE, 0xAD, 0xBE, 0xEF};

void seed_bag(const std::filesystem::path & path)
{
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Mcap;
  options.layout = bagwiz::io::Layout::SingleFile;
  options.mcap_compression = "none";
  auto writer = bagwiz::io::open_write(path, options);
  bagwiz::io::TopicInfo t;
  t.name = "/probe";
  t.type = "std_msgs/msg/Int32";
  t.serialization_format = "cdr";
  writer->declare_topic(t);
  writer->write(
    "/probe", 1'000'000'000LL,
    std::span<const std::byte>(
      reinterpret_cast<const std::byte *>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        kPayload.data()),
      kPayload.size()));
  writer->close();
}

// Minimal in-memory BagWriter used to drive the factory/close helpers without
// touching the filesystem.
class StubWriter : public bagwiz::io::BagWriter
{
public:
  void declare_topic(const bagwiz::io::TopicInfo & /*topic*/) override {}
  void write(
    std::string_view /*topic*/, int64_t /*timestamp_ns*/,
    std::span<const std::byte> /*payload*/) override
  {
  }
  void close() override
  {
    ++close_calls;
    if (throw_on_close) {
      throw std::runtime_error("stub close failure");
    }
  }

  int close_calls = 0;
  bool throw_on_close = false;
};

class BagOpenTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ =
      std::filesystem::temp_directory_path() /
      ("bagwiz_bag_open_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()));
    std::filesystem::create_directories(tmp_dir_);
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }

  std::filesystem::path tmp_dir_;
};

constexpr const char * kLogger = "bagwiz.test.bag_open";

}  // namespace

TEST_F(BagOpenTest, OpenReadOrLogReturnsReaderForExistingBag)
{
  const auto bag = tmp_dir_ / "probe.mcap";
  seed_bag(bag);

  auto reader = bagwiz::io::open_read_or_log(bag, kLogger);

  ASSERT_NE(reader, nullptr);
  ASSERT_EQ(reader->topics().size(), 1u);
  EXPECT_EQ(reader->topics()[0].name, "/probe");
}

TEST_F(BagOpenTest, OpenReadOrLogReturnsNullForMissingPath)
{
  auto reader = bagwiz::io::open_read_or_log(tmp_dir_ / "missing.mcap", kLogger);
  EXPECT_EQ(reader, nullptr);
}

TEST_F(BagOpenTest, OpenReadOrLogReturnsNullForNonBagFile)
{
  const auto not_a_bag = tmp_dir_ / "notes.txt";
  std::ofstream out(not_a_bag);
  out << "this is not a bag";
  out.close();

  auto reader = bagwiz::io::open_read_or_log(not_a_bag, kLogger);
  EXPECT_EQ(reader, nullptr);
}

TEST_F(BagOpenTest, OpenWriteOrLogReturnsWriterFromFactory)
{
  auto writer =
    bagwiz::io::open_write_or_log([]() { return std::make_unique<StubWriter>(); }, kLogger);
  EXPECT_NE(writer, nullptr);
}

TEST_F(BagOpenTest, OpenWriteOrLogReturnsNullWhenFactoryThrows)
{
  auto writer = bagwiz::io::open_write_or_log(
    []() -> std::unique_ptr<bagwiz::io::BagWriter> { throw std::runtime_error("factory failure"); },
    kLogger);
  EXPECT_EQ(writer, nullptr);
}

TEST_F(BagOpenTest, CloseWriterOrLogTrueOnSuccess)
{
  StubWriter writer;
  EXPECT_TRUE(bagwiz::io::close_writer_or_log(writer, kLogger));
  EXPECT_EQ(writer.close_calls, 1);
}

TEST_F(BagOpenTest, CloseWriterOrLogFalseOnThrow)
{
  StubWriter writer;
  writer.throw_on_close = true;
  EXPECT_FALSE(bagwiz::io::close_writer_or_log(writer, kLogger));
  EXPECT_EQ(writer.close_calls, 1);
}
