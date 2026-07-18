// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__IO__MESSAGE_DECOMPRESSOR_HPP_
#define BAGWIZ__IO__MESSAGE_DECOMPRESSOR_HPP_

#include <cstddef>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace bagwiz::io
{

// Forward declaration so the zstd C type does not leak into clients of this
// header. The deleter is defined in the implementation file.
struct MessageDecompressorState;

// Per-message decompressor for rosbag2 `compression_mode: MESSAGE` bags.
//
// rosbag2's per-message compression path (see rosbag2_compression_zstd's
// ZstdDecompressor::decompress_serialized_bag_message) wraps each message's
// serialized payload in a single zstd frame with no extra header. This class
// mirrors that contract: callers hand in the compressed bytes read from the
// storage backend (`messages.data` for SQLite3, the per-message payload for
// MCAP), and receive the decompressed bytes back as a view into an
// internal, reusable buffer.
//
// The returned span is **invalidated by the next call to decompress()**;
// copy it if you need to outlive the next call.
//
// The class is move-only (Rule of Five): the ZSTD_DCtx held internally is
// expensive to allocate and cheap to reuse, matching upstream rosbag2's
// long-lived context pattern.
class MessageDecompressor
{
public:
  // Construct a decompressor for `format`. Only "zstd" is supported today;
  // anything else throws std::runtime_error with a clear diagnostic so the
  // factory can fail fast rather than silently producing wrong bytes.
  explicit MessageDecompressor(std::string_view format);

  ~MessageDecompressor();

  MessageDecompressor(const MessageDecompressor &) = delete;
  MessageDecompressor & operator=(const MessageDecompressor &) = delete;
  MessageDecompressor(MessageDecompressor &&) noexcept;
  MessageDecompressor & operator=(MessageDecompressor &&) noexcept;

  // Decompresses `compressed`. The returned span points into an internal
  // buffer owned by this object and is invalidated by the next call to
  // decompress(). Throws std::runtime_error on a malformed frame.
  [[nodiscard]] std::span<const std::byte> decompress(std::span<const std::byte> compressed);

  // The format string this decompressor was constructed with (e.g. "zstd").
  // Useful for diagnostics; never empty after construction.
  [[nodiscard]] const std::string & format() const noexcept { return format_; }

private:
  std::string format_;
  std::unique_ptr<MessageDecompressorState> state_;
  std::vector<std::byte> out_buffer_;
};

}  // namespace bagwiz::io

#endif  // BAGWIZ__IO__MESSAGE_DECOMPRESSOR_HPP_
