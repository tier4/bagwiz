// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__CDR_WALKER__CDR_READER_HPP_
#define BAGWIZ__CORE__CDR_WALKER__CDR_READER_HPP_

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>

namespace bagwiz::core::cdr_walker
{

// Alignment-aware reader for the CDR-1 wire format used by ROS 2 over
// FastDDS / CycloneDDS. Wraps an externally-owned span of bytes; does
// not copy.
//
// On construction the 4-byte CDR encapsulation header is parsed:
//   byte 0: representation_identifier_high (always 0)
//   byte 1: representation_identifier_low (encoding kind: 0=BE, 1=LE,
//           2=PL_CDR_BE, 3=PL_CDR_LE)
//   bytes 2-3: representation_options. Per OMG DDS-XTYPES 1.3
//              §7.6.3.1.2, the lower two bits of this field encode the
//              number of padding bytes (0-3) appended after the body so
//              the total encapsulated size ends on a 4-byte boundary.
//              We honor that: those bytes are excluded from the body so
//              `remaining()` and the underflow checks reflect the real
//              payload length. Legacy PLAIN_CDR_LE writers set
//              options=0, so this is a no-op for them.
//
// Endianness is taken from the low bit of the kind byte (matching what
// foxglove `_cdr.py` does). PL_CDR (parameter-list-with-CDR, used for
// XCDR-2 mutable types) is rejected — bagwiz schemas all decode as
// plain CDR-1, and a PL_CDR payload is a strong signal that the bag
// was produced by a stack we can't decode without introspection.
//
// Alignment math uses (offset - 4) % size: the 4-byte header is
// excluded so primitive boundaries align to file-relative offsets, not
// to absolute byte 0. This matches every CDR-1 implementation I know of
// (FastDDS, Cyclone, Foxglove, eProsima) and the Python reference.
//
// Methods throw std::runtime_error on:
//   - read past end (insufficient bytes after alignment)
//   - PL_CDR encoding rejection
//   - invalid string length (would overflow remaining bytes)
class CdrReader
{
public:
  explicit CdrReader(std::span<const std::byte> data);

  bool little_endian() const noexcept { return little_endian_; }
  std::size_t offset() const noexcept { return offset_; }
  std::size_t remaining() const noexcept
  {
    const std::size_t end = data_.size() - trailing_pad_;
    return end > offset_ ? end - offset_ : 0;
  }

  // Primitive reads. Each one aligns to its size and advances offset_.
  bool read_bool();
  std::int8_t read_i8();
  std::uint8_t read_u8();
  std::int16_t read_i16();
  std::uint16_t read_u16();
  std::int32_t read_i32();
  std::uint32_t read_u32();
  std::int64_t read_i64();
  std::uint64_t read_u64();
  float read_f32();
  double read_f64();

  // CDR string: uint32 length prefix (length includes the trailing NUL,
  // so a 0-length empty string serializes as `00 00 00 00` and a 1-length
  // empty string as `01 00 00 00 00`). Returns the bytes between length
  // and NUL as a UTF-8 std::string.
  std::string read_string();

  // Read N raw bytes without alignment. Used by sequence-of-byte fast
  // paths (e.g. `uint8[]` payloads) where each element is 1-byte and the
  // walker would otherwise call read_u8() in a loop.
  std::span<const std::byte> read_bytes(std::size_t n);

  // Read a uint32 sequence-length prefix. Same wire format as read_u32()
  // — exposed under a separate name so call sites read intentionally.
  std::uint32_t read_sequence_length() { return read_u32(); }

private:
  void align(std::size_t size);
  void ensure_remaining(std::size_t n) const;

  std::span<const std::byte> data_;
  std::size_t offset_ = 4;  // skip the 4-byte CDR header
  std::size_t trailing_pad_ = 0;
  bool little_endian_ = true;
};

}  // namespace bagwiz::core::cdr_walker

#endif  // BAGWIZ__CORE__CDR_WALKER__CDR_READER_HPP_
