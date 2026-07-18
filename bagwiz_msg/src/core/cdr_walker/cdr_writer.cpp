// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/cdr_walker/cdr_writer.hpp"

#include <array>
#include <bit>
#include <cstring>
#include <type_traits>

namespace bagwiz::core::cdr_walker
{

namespace
{

// Byte-swap a trivially-copyable value (mirrors cdr_reader.cpp::swap_bytes).
template <typename T>
T swap_bytes(T value)
{
  static_assert(std::is_trivially_copyable_v<T>, "swap_bytes requires trivially-copyable");
  std::array<std::byte, sizeof(T)> buf{};
  std::memcpy(buf.data(), &value, sizeof(T));
  std::array<std::byte, sizeof(T)> rev{};
  for (std::size_t i = 0; i < sizeof(T); ++i) {
    rev[i] = buf[sizeof(T) - 1 - i];
  }
  T out{};
  std::memcpy(&out, rev.data(), sizeof(T));
  return out;
}

// Append `value` to `buf` in little-endian byte order (host-endian aware).
template <typename T>
void append_le(std::vector<std::byte> & buf, T value)
{
  if constexpr (sizeof(T) > 1) {
    if constexpr (std::endian::native != std::endian::little) {
      value = swap_bytes(value);
    }
  }
  std::array<std::byte, sizeof(T)> bytes{};
  std::memcpy(bytes.data(), &value, sizeof(T));
  buf.insert(buf.end(), bytes.begin(), bytes.end());
}

}  // namespace

CdrWriter::CdrWriter()
{
  // 4-byte CDR encapsulation header: representation kind 0x01 (PLAIN_CDR_LE),
  // options 0 (no trailing pad). Byte 0 is the high id (always 0).
  buf_.push_back(std::byte{0x00});
  buf_.push_back(std::byte{0x01});
  buf_.push_back(std::byte{0x00});
  buf_.push_back(std::byte{0x00});
}

void CdrWriter::align(std::size_t size)
{
  if (size <= 1) {
    return;
  }
  // (offset - 4) % size — exclude the 4-byte header from alignment math, the
  // exact rule CdrReader::align uses. Pad with zero bytes.
  const std::size_t mod = (buf_.size() - 4) % size;
  if (mod != 0) {
    const std::size_t pad = size - mod;
    buf_.insert(buf_.end(), pad, std::byte{0x00});
  }
}

void CdrWriter::write_bool(bool value)
{
  write_u8(value ? 1 : 0);
}

void CdrWriter::write_i8(std::int8_t value)
{
  buf_.push_back(static_cast<std::byte>(value));
}

void CdrWriter::write_u8(std::uint8_t value)
{
  buf_.push_back(static_cast<std::byte>(value));
}

void CdrWriter::write_i16(std::int16_t value)
{
  align(2);
  append_le(buf_, value);
}

void CdrWriter::write_u16(std::uint16_t value)
{
  align(2);
  append_le(buf_, value);
}

void CdrWriter::write_i32(std::int32_t value)
{
  align(4);
  append_le(buf_, value);
}

void CdrWriter::write_u32(std::uint32_t value)
{
  align(4);
  append_le(buf_, value);
}

void CdrWriter::write_i64(std::int64_t value)
{
  align(8);
  append_le(buf_, value);
}

void CdrWriter::write_u64(std::uint64_t value)
{
  align(8);
  append_le(buf_, value);
}

void CdrWriter::write_f32(float value)
{
  align(4);
  append_le(buf_, value);
}

void CdrWriter::write_f64(double value)
{
  align(8);
  append_le(buf_, value);
}

void CdrWriter::write_string(std::string_view value)
{
  // length prefix includes the trailing NUL (canonical CDR string).
  const auto length_with_nul = static_cast<std::uint32_t>(value.size() + 1);
  write_u32(length_with_nul);
  const auto * bytes = reinterpret_cast<const std::byte *>(value.data());
  buf_.insert(buf_.end(), bytes, bytes + value.size());
  buf_.push_back(std::byte{0x00});
}

void CdrWriter::write_bytes(std::span<const std::byte> bytes)
{
  buf_.insert(buf_.end(), bytes.begin(), bytes.end());
}

}  // namespace bagwiz::core::cdr_walker
