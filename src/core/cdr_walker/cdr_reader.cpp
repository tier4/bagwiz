// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/cdr_walker/cdr_reader.hpp"

#include <bit>
#include <cstdint>
#include <cstring>
#include <span>
#include <stdexcept>
#include <string>

namespace bagwiz::core::cdr_walker
{

namespace
{

// Byte-swap a value of integral type if the host endianness differs from
// the desired wire endianness. Restricted to `is_trivially_copyable` types
// so std::memcpy is safe for both directions.
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

// Read a value of trivially-copyable type T from `src`, applying
// endian conversion when wire and host differ.
template <typename T>
T load_le(const std::byte * src, bool little_endian)
{
  T value{};
  std::memcpy(&value, src, sizeof(T));
  if constexpr (sizeof(T) == 1) {
    return value;
  } else {
    constexpr bool host_le = std::endian::native == std::endian::little;
    return little_endian == host_le ? value : swap_bytes(value);
  }
}

}  // namespace

CdrReader::CdrReader(std::span<const std::byte> data) : data_(data)
{
  if (data.size() < 4) {
    throw std::runtime_error(
      "CDR payload too short for encapsulation header (got " + std::to_string(data.size()) +
      " bytes; need at least 4)");
  }
  const auto kind = static_cast<std::uint8_t>(data[1]);
  // PL_CDR_BE = 0x02, PL_CDR_LE = 0x03 — reject; these carry XCDR-2
  // mutable types that bagwiz's plain-CDR walker can't handle.
  if (kind >= 0x02) {
    throw std::runtime_error(
      "PL_CDR encapsulation (kind=" + std::to_string(kind) +
      ") is not supported; payload requires introspection-typesupport decode");
  }
  little_endian_ = (kind & 0x01) != 0;
  // Lower two bits of representation_options encode the count of pad
  // bytes appended after the body to align the encapsulation to a
  // 4-byte boundary (OMG DDS-XTYPES 1.3 §7.6.3.1.2). Trim those so
  // remaining()/ensure_remaining() reflect the real body length.
  const auto pad = static_cast<std::size_t>(static_cast<std::uint8_t>(data[3]) & 0x03);
  trailing_pad_ = pad <= data.size() - 4 ? pad : 0;
}

void CdrReader::ensure_remaining(std::size_t n) const
{
  const std::size_t end = data_.size() - trailing_pad_;
  if (end < offset_ || end - offset_ < n) {
    throw std::runtime_error(
      "CDR underflow: need " + std::to_string(n) + " more bytes at offset " +
      std::to_string(offset_) + " (payload size " + std::to_string(data_.size()) + ")");
  }
}

void CdrReader::align(std::size_t size)
{
  if (size <= 1) {
    return;
  }
  // (offset - 4) % size — exclude the 4-byte header from alignment math.
  const std::size_t mod = (offset_ - 4) % size;
  if (mod != 0) {
    offset_ += size - mod;
  }
}

bool CdrReader::read_bool()
{
  return read_u8() != 0;
}

std::int8_t CdrReader::read_i8()
{
  ensure_remaining(1);
  const auto v = static_cast<std::int8_t>(data_[offset_]);
  ++offset_;
  return v;
}

std::uint8_t CdrReader::read_u8()
{
  ensure_remaining(1);
  const auto v = static_cast<std::uint8_t>(data_[offset_]);
  ++offset_;
  return v;
}

std::int16_t CdrReader::read_i16()
{
  align(2);
  ensure_remaining(2);
  const auto v = load_le<std::int16_t>(&data_[offset_], little_endian_);
  offset_ += 2;
  return v;
}

std::uint16_t CdrReader::read_u16()
{
  align(2);
  ensure_remaining(2);
  const auto v = load_le<std::uint16_t>(&data_[offset_], little_endian_);
  offset_ += 2;
  return v;
}

std::int32_t CdrReader::read_i32()
{
  align(4);
  ensure_remaining(4);
  const auto v = load_le<std::int32_t>(&data_[offset_], little_endian_);
  offset_ += 4;
  return v;
}

std::uint32_t CdrReader::read_u32()
{
  align(4);
  ensure_remaining(4);
  const auto v = load_le<std::uint32_t>(&data_[offset_], little_endian_);
  offset_ += 4;
  return v;
}

std::int64_t CdrReader::read_i64()
{
  align(8);
  ensure_remaining(8);
  const auto v = load_le<std::int64_t>(&data_[offset_], little_endian_);
  offset_ += 8;
  return v;
}

std::uint64_t CdrReader::read_u64()
{
  align(8);
  ensure_remaining(8);
  const auto v = load_le<std::uint64_t>(&data_[offset_], little_endian_);
  offset_ += 8;
  return v;
}

float CdrReader::read_f32()
{
  align(4);
  ensure_remaining(4);
  const auto v = load_le<float>(&data_[offset_], little_endian_);
  offset_ += 4;
  return v;
}

double CdrReader::read_f64()
{
  align(8);
  ensure_remaining(8);
  const auto v = load_le<double>(&data_[offset_], little_endian_);
  offset_ += 8;
  return v;
}

std::string CdrReader::read_string()
{
  const auto length_with_nul = read_u32();
  if (length_with_nul == 0) {
    // Some serializers emit length 0 for the empty string instead of
    // the canonical 1 (just NUL). Treat it as empty without consuming
    // any further bytes; foxglove _cdr.py does the same.
    return std::string{};
  }
  ensure_remaining(length_with_nul);
  // length_with_nul includes the trailing NUL byte; the caller only
  // wants the (length-1) bytes of payload.
  const auto payload_len = static_cast<std::size_t>(length_with_nul) - 1;
  std::string out;
  out.assign(reinterpret_cast<const char *>(&data_[offset_]), payload_len);
  offset_ += length_with_nul;
  return out;
}

std::span<const std::byte> CdrReader::read_bytes(std::size_t n)
{
  ensure_remaining(n);
  auto out = data_.subspan(offset_, n);
  offset_ += n;
  return out;
}

}  // namespace bagwiz::core::cdr_walker
