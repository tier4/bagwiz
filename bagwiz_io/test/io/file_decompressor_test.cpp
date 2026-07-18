// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/file_decompressor.hpp"

#include <gtest/gtest.h>
#include <zstd.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

// Compress `plain` into a freshly written `.zstd` file at `path`.
void write_zstd_file(const std::filesystem::path & path, const std::vector<std::byte> & plain)
{
  const std::size_t bound = ZSTD_compressBound(plain.size());
  std::vector<std::byte> compressed(bound);
  const std::size_t written = ZSTD_compress(
    compressed.data(), compressed.size(), plain.data(), plain.size(), /*compressionLevel=*/3);
  ASSERT_FALSE(static_cast<bool>(ZSTD_isError(written))) << ZSTD_getErrorName(written);
  compressed.resize(written);

  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  ASSERT_TRUE(out.good());
  out.write(
    reinterpret_cast<const char *>(compressed.data()),
    static_cast<std::streamsize>(compressed.size()));
  out.flush();
  ASSERT_TRUE(out.good());
}

// Read a whole file into a byte vector.
std::vector<std::byte> slurp(const std::filesystem::path & path)
{
  std::ifstream in(path, std::ios::binary | std::ios::ate);
  EXPECT_TRUE(in.good());
  const auto size = in.tellg();
  in.seekg(0);
  std::vector<std::byte> buf(static_cast<std::size_t>(size));
  in.read(reinterpret_cast<char *>(buf.data()), size);
  return buf;
}

std::vector<std::byte> make_payload(std::size_t n)
{
  std::vector<std::byte> v(n);
  for (std::size_t i = 0; i < n; ++i) {
    v[i] = std::byte{static_cast<std::uint8_t>((i * 31 + 7) & 0xFF)};
  }
  return v;
}

class FileDecompressorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("bagwiz_file_decompressor_" +
                std::to_string(::testing::UnitTest::GetInstance()->random_seed()));
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

TEST(IsZstdMagic, MatchesMagicPrefix)
{
  const std::array<std::byte, 6> magic{std::byte{0x28}, std::byte{0xB5}, std::byte{0x2F},
                                       std::byte{0xFD}, std::byte{0x00}, std::byte{0x11}};
  EXPECT_TRUE(bagwiz::io::is_zstd_magic(std::span<const std::byte>(magic.data(), magic.size())));
}

TEST(IsZstdMagic, RejectsShortAndForeign)
{
  const std::array<std::byte, 3> too_short{std::byte{0x28}, std::byte{0xB5}, std::byte{0x2F}};
  EXPECT_FALSE(
    bagwiz::io::is_zstd_magic(std::span<const std::byte>(too_short.data(), too_short.size())));

  const std::array<std::byte, 4> sqlite{
    std::byte{'S'}, std::byte{'Q'}, std::byte{'L'}, std::byte{'i'}};
  EXPECT_FALSE(bagwiz::io::is_zstd_magic(std::span<const std::byte>(sqlite.data(), sqlite.size())));
}

TEST_F(FileDecompressorTest, RoundTripsArbitraryPayload)
{
  const auto plain = make_payload(50'000);
  const auto src = tmp_dir_ / "blob.db3.zstd";
  write_zstd_file(src, plain);

  EXPECT_TRUE(bagwiz::io::is_zstd_file(src));

  std::filesystem::path temp_path;
  {
    bagwiz::io::TempFile temp = bagwiz::io::decompress_zstd_file_to_temp(src);
    ASSERT_TRUE(temp.valid());
    temp_path = temp.path();
    EXPECT_TRUE(std::filesystem::exists(temp_path));
    // The temp file inherits the inner `.db3` extension.
    EXPECT_EQ(temp_path.extension().string(), ".db3");

    const auto decompressed = slurp(temp_path);
    EXPECT_EQ(decompressed, plain);
  }
  // Destructor removed the temp file.
  EXPECT_FALSE(std::filesystem::exists(temp_path));
}

TEST_F(FileDecompressorTest, ThrowsOnNonZstdInput)
{
  const auto src = tmp_dir_ / "not_zstd.db3";
  {
    std::ofstream out(src, std::ios::binary);
    out << "SQLite format 3 not really";
  }
  EXPECT_FALSE(bagwiz::io::is_zstd_file(src));
  EXPECT_THROW(
    { [[maybe_unused]] auto t = bagwiz::io::decompress_zstd_file_to_temp(src); },
    std::runtime_error);
}

TEST_F(FileDecompressorTest, ThrowsOnMissingFile)
{
  EXPECT_THROW(
    {
      [[maybe_unused]] auto t =
        bagwiz::io::decompress_zstd_file_to_temp(tmp_dir_ / "does_not_exist.db3.zstd");
    },
    std::runtime_error);
}

TEST_F(FileDecompressorTest, ThrowsOnTruncatedFrame)
{
  const auto plain = make_payload(20'000);
  const auto src = tmp_dir_ / "truncated.db3.zstd";
  write_zstd_file(src, plain);

  // Chop the file in half so the frame is incomplete.
  const auto full = slurp(src);
  {
    std::ofstream out(src, std::ios::binary | std::ios::trunc);
    out.write(
      reinterpret_cast<const char *>(full.data()), static_cast<std::streamsize>(full.size() / 2));
  }

  EXPECT_THROW(
    { [[maybe_unused]] auto t = bagwiz::io::decompress_zstd_file_to_temp(src); },
    std::runtime_error);
}

TEST_F(FileDecompressorTest, DefaultTempFileOwnsNothing)
{
  bagwiz::io::TempFile empty;
  EXPECT_FALSE(empty.valid());
  EXPECT_TRUE(empty.path().empty());
  // Destruction is a no-op (no throw, nothing to remove).
}
