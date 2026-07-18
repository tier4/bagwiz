// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/message_decompressor.hpp"

#include <zstd.h>

#include <cstddef>
#include <iterator>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace bagwiz::io
{

namespace
{

constexpr std::string_view kZstd = "zstd";

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

}  // namespace

// Hide the zstd C type behind a thin opaque struct so the public header does
// not have to expose <zstd.h>. The PIMPL is intentionally tiny — only the
// long-lived decompression context — because the reusable output buffer
// lives on MessageDecompressor itself (it must stay accessible to the public
// span return value).
struct MessageDecompressorState
{
  ZstdDCtxPtr context;
};

MessageDecompressor::MessageDecompressor(std::string_view format)
: format_(format), state_(std::make_unique<MessageDecompressorState>())
{
  if (format_ != kZstd) {
    throw std::runtime_error(
      "MessageDecompressor: unsupported compression_format '" + format_ +
      "' (only 'zstd' is implemented for rosbag2 MESSAGE-mode)");
  }
  state_->context = ZstdDCtxPtr{ZSTD_createDCtx()};
  if (state_->context == nullptr) {
    throw std::runtime_error("MessageDecompressor: failed to allocate ZSTD_DCtx");
  }
}

MessageDecompressor::~MessageDecompressor() = default;
MessageDecompressor::MessageDecompressor(MessageDecompressor &&) noexcept = default;
MessageDecompressor & MessageDecompressor::operator=(MessageDecompressor &&) noexcept = default;

std::span<const std::byte> MessageDecompressor::decompress(std::span<const std::byte> compressed)
{
  if (compressed.empty()) {
    out_buffer_.clear();
    return {out_buffer_.data(), out_buffer_.size()};
  }

  // Fast path: a single zstd frame whose decompressed size is known up-front
  // (the rosbag2 MESSAGE-mode writer always emits such frames; see
  // rosbag2_compression_zstd::ZstdDecompressor::decompress_serialized_bag_message).
  const auto known_size = ZSTD_getFrameContentSize(compressed.data(), compressed.size());
  if (known_size != ZSTD_CONTENTSIZE_ERROR && known_size != ZSTD_CONTENTSIZE_UNKNOWN) {
    out_buffer_.resize(static_cast<std::size_t>(known_size));
    const auto written = ZSTD_decompressDCtx(
      state_->context.get(), out_buffer_.data(), out_buffer_.size(), compressed.data(),
      compressed.size());
    if (ZSTD_isError(written) != 0U) {
      throw std::runtime_error(
        std::string{"MessageDecompressor: zstd decompress failed: "} + ZSTD_getErrorName(written));
    }
    out_buffer_.resize(written);
    return {out_buffer_.data(), out_buffer_.size()};
  }

  if (known_size == ZSTD_CONTENTSIZE_ERROR) {
    throw std::runtime_error(
      "MessageDecompressor: input is not a valid zstd frame (likely "
      "not a rosbag2 MESSAGE-mode payload)");
  }

  // Streaming fallback: the frame omitted its decompressed-size field. This
  // is rare for rosbag2-written payloads but spec-legal. Grow the buffer in
  // chunks until the frame is fully consumed.
  const std::size_t chunk_size = ZSTD_DStreamOutSize();
  out_buffer_.clear();
  std::vector<std::byte> chunk(chunk_size);
  ZSTD_inBuffer in{compressed.data(), compressed.size(), 0};
  ZSTD_initDStream(state_->context.get());

  while (in.pos < in.size) {
    ZSTD_outBuffer out{chunk.data(), chunk.size(), 0};
    const auto ret = ZSTD_decompressStream(state_->context.get(), &out, &in);
    if (ZSTD_isError(ret) != 0U) {
      throw std::runtime_error(
        std::string{"MessageDecompressor: zstd streaming decompress failed: "} +
        ZSTD_getErrorName(ret));
    }
    const auto first = chunk.cbegin();
    const auto last = std::next(first, static_cast<std::ptrdiff_t>(out.pos));
    out_buffer_.insert(out_buffer_.end(), first, last);
    if (ret == 0 && in.pos == in.size) {
      break;
    }
  }
  return {out_buffer_.data(), out_buffer_.size()};
}

}  // namespace bagwiz::io
