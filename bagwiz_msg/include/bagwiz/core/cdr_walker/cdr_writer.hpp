// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__CDR_WALKER__CDR_WRITER_HPP_
#define BAGWIZ__CORE__CDR_WALKER__CDR_WRITER_HPP_

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <utility>
#include <vector>

namespace bagwiz::core::cdr_walker
{

// Alignment-aware writer for the CDR-1 wire format used by ROS 2 over
// FastDDS / CycloneDDS. The exact symmetric counterpart of CdrReader: it
// emits the 4-byte little-endian encapsulation header on construction and
// applies the same `(offset - 4) % size` alignment before each primitive,
// so any message written here parses back byte-for-byte through CdrReader.
//
// Little-endian only (representation kind 0x01, options 0). That matches
// every plain-CDR-1 payload bagwiz reads and keeps the output deterministic
// and host-endian-independent.
class CdrWriter
{
public:
  CdrWriter();

  void write_bool(bool value);
  void write_i8(std::int8_t value);
  void write_u8(std::uint8_t value);
  void write_i16(std::int16_t value);
  void write_u16(std::uint16_t value);
  void write_i32(std::int32_t value);
  void write_u32(std::uint32_t value);
  void write_i64(std::int64_t value);
  void write_u64(std::uint64_t value);
  void write_f32(float value);
  void write_f64(double value);

  // CDR string: uint32 length prefix (payload length + 1 for the trailing
  // NUL) followed by the bytes and the NUL. The empty string serializes as
  // `01 00 00 00 00` (canonical), matching what CdrReader expects.
  void write_string(std::string_view value);

  // Write N raw bytes with no alignment — the symmetric counterpart of
  // CdrReader::read_bytes, for `uint8[]` sequence payloads.
  void write_bytes(std::span<const std::byte> bytes);

  // Write a uint32 sequence-length prefix (same wire format as write_u32).
  void write_sequence_length(std::uint32_t length) { write_u32(length); }

  // The full payload, including the 4-byte encapsulation header.
  [[nodiscard]] const std::vector<std::byte> & data() const noexcept { return buf_; }
  [[nodiscard]] std::vector<std::byte> take() noexcept { return std::move(buf_); }

private:
  void align(std::size_t size);

  std::vector<std::byte> buf_;
};

}  // namespace bagwiz::core::cdr_walker

#endif  // BAGWIZ__CORE__CDR_WALKER__CDR_WRITER_HPP_
