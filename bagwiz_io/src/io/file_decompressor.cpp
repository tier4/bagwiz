// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/file_decompressor.hpp"

#include <unistd.h>
#include <zstd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <ios>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace bagwiz::io
{

namespace
{

constexpr std::array<std::byte, 4> kZstdMagic{
  std::byte{0x28}, std::byte{0xB5}, std::byte{0x2F}, std::byte{0xFD}};

struct ZstdDCtxDeleter
{
  void operator()(ZSTD_DCtx * ctx) const noexcept
  {
    if (ctx != nullptr) {
      ZSTD_freeDCtx(ctx);
    }
  }
};
using ZstdDCtxPtr = std::unique_ptr<ZSTD_DCtx, ZstdDCtxDeleter>;

// Derive the suffix the temp file should carry from the source filename: the
// inner extension once the trailing `.zstd` is stripped (e.g. `foo.db3.zstd`
// -> `.db3`). Falls back to `.tmp` when the source has no inner extension.
std::string inner_suffix(const std::filesystem::path & src)
{
  std::string name = src.filename().string();
  constexpr std::string_view kZstdExt = ".zstd";
  if (
    name.size() > kZstdExt.size() &&
    name.compare(name.size() - kZstdExt.size(), kZstdExt.size(), kZstdExt) == 0) {
    name.resize(name.size() - kZstdExt.size());
  }
  const auto dot = name.rfind('.');
  if (dot != std::string::npos && dot + 1 < name.size()) {
    return name.substr(dot);
  }
  return ".tmp";
}

// A process-unique temp path. Mirrors the pid + steady-clock naming used by
// core/bag/bag_inplace.cpp, plus an atomic counter so two decompressions in the
// same process (e.g. two shards) never collide.
std::filesystem::path make_temp_path(const std::filesystem::path & src)
{
  static std::atomic<std::uint64_t> counter{0};
  const auto pid = static_cast<std::uint64_t>(::getpid());
  const auto ticks =
    static_cast<std::uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
  const auto n = counter.fetch_add(1);
  const std::string name = "bagwiz-zstd-" + std::to_string(pid) + "-" + std::to_string(ticks) +
                           "-" + std::to_string(n) + inner_suffix(src);
  return std::filesystem::temp_directory_path() / name;
}

}  // namespace

bool is_zstd_magic(std::span<const std::byte> bytes) noexcept
{
  if (bytes.size() < kZstdMagic.size()) {
    return false;
  }
  for (std::size_t i = 0; i < kZstdMagic.size(); ++i) {
    if (bytes[i] != kZstdMagic[i]) {
      return false;
    }
  }
  return true;
}

bool is_zstd_file(const std::filesystem::path & path) noexcept
{
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    return false;
  }
  std::array<std::byte, 4> buf{};
  f.read(reinterpret_cast<char *>(buf.data()), static_cast<std::streamsize>(buf.size()));
  const auto read = f.gcount();
  if (read < 0) {
    return false;
  }
  return is_zstd_magic(std::span<const std::byte>(buf.data(), static_cast<std::size_t>(read)));
}

TempFile decompress_zstd_file_to_temp(const std::filesystem::path & src)
{
  std::ifstream in(src, std::ios::binary);
  if (!in) {
    throw std::runtime_error("failed to open zstd file for reading: " + src.string());
  }

  ZstdDCtxPtr dctx{ZSTD_createDCtx()};
  if (dctx == nullptr) {
    throw std::runtime_error("failed to allocate ZSTD_DCtx for " + src.string());
  }

  const auto temp_path = make_temp_path(src);
  // Own the path immediately so any throw below still unlinks the partial
  // output via TempFile's destructor.
  TempFile temp(temp_path);

  std::ofstream out(temp_path, std::ios::binary | std::ios::trunc);
  if (!out) {
    throw std::runtime_error("failed to open temp file for writing: " + temp_path.string());
  }

  const std::size_t in_chunk = ZSTD_DStreamInSize();
  const std::size_t out_chunk = ZSTD_DStreamOutSize();
  std::vector<std::byte> in_buf(in_chunk);
  std::vector<std::byte> out_buf(out_chunk);

  std::size_t last_ret = 0;
  bool saw_input = false;

  for (;;) {
    in.read(reinterpret_cast<char *>(in_buf.data()), static_cast<std::streamsize>(in_buf.size()));
    const auto got = in.gcount();
    if (got <= 0) {
      break;
    }
    saw_input = true;

    ZSTD_inBuffer input{in_buf.data(), static_cast<std::size_t>(got), 0};
    while (input.pos < input.size) {
      ZSTD_outBuffer output{out_buf.data(), out_buf.size(), 0};
      const auto ret = ZSTD_decompressStream(dctx.get(), &output, &input);
      if (ZSTD_isError(ret) != 0U) {
        throw std::runtime_error(
          std::string("zstd decompress failed for ") + src.string() + ": " +
          ZSTD_getErrorName(ret));
      }
      out.write(
        reinterpret_cast<const char *>(out_buf.data()), static_cast<std::streamsize>(output.pos));
      if (!out) {
        throw std::runtime_error("failed writing decompressed output to " + temp_path.string());
      }
      last_ret = ret;
    }
  }

  if (!saw_input) {
    throw std::runtime_error("zstd file is empty: " + src.string());
  }
  // A non-zero final return means the last frame ended mid-block — the input
  // was truncated.
  if (last_ret != 0) {
    throw std::runtime_error("zstd file ended on an incomplete frame: " + src.string());
  }

  out.flush();
  if (!out) {
    throw std::runtime_error("failed to flush decompressed output to " + temp_path.string());
  }

  return temp;
}

}  // namespace bagwiz::io
