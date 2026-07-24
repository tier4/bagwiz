// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "mcap_chunk_prefetch.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <zstd.h>

#include <algorithm>
#include <cstring>
#include <exception>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace bagwiz::io::detail
{

namespace
{

constexpr std::size_t kChunkRecordHeaderBytes = 1 + 8;  // opcode + record length
// Minimum chunk record body: message start/end time (8+8), uncompressed size
// (8), uncompressed CRC (4), compression string length (4), compressed size
// (8) — with an empty compression string.
constexpr std::size_t kMinChunkBodyBytes = 8 + 8 + 8 + 4 + 4 + 8;
constexpr std::uint8_t kChunkOpCode = 0x06;

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

std::uint32_t read_u32(const std::byte * p)
{
  std::uint32_t v = 0;
  std::memcpy(&v, p, sizeof(v));
  return v;
}

std::uint64_t read_u64(const std::byte * p)
{
  std::uint64_t v = 0;
  std::memcpy(&v, p, sizeof(v));
  return v;
}

// Result of decompress_chunk_record: the chunk itself, plus whichever of the
// two scratch buffers it did not consume — handed back explicitly so the
// caller never has to infer ownership from a moved-from vector's state.
struct DecompressOutcome
{
  PrefetchedChunk chunk;
  std::vector<std::byte> spare_record;  // `record`, if not consumed
  std::vector<std::byte> spare_dst;     // `dst`, if not consumed
};

// Parse one raw chunk record and produce its decompressed records blob.
// Uncompressed chunks take ownership of `record` (no blob copy — the raw
// buffer moves into the result with an offset); zstd chunks decompress into
// `dst` (a pooled buffer, possibly non-empty already) via the caller-owned
// `dctx`, reused across chunks instead of paying per-call setup cost.
DecompressOutcome decompress_chunk_record(
  std::vector<std::byte> && record, std::vector<std::byte> && dst, ZSTD_DCtx * dctx)
{
  DecompressOutcome outcome;
  PrefetchedChunk & out = outcome.chunk;
  if (record.size() < kChunkRecordHeaderBytes + kMinChunkBodyBytes) {
    out.error = "chunk record truncated";
    outcome.spare_record = std::move(record);
    outcome.spare_dst = std::move(dst);
    return outcome;
  }
  if (std::to_integer<std::uint8_t>(record[0]) != kChunkOpCode) {
    out.error = "record at chunk offset is not a Chunk record";
    outcome.spare_record = std::move(record);
    outcome.spare_dst = std::move(dst);
    return outcome;
  }

  const std::byte * body = record.data() + kChunkRecordHeaderBytes;
  const std::size_t body_size = record.size() - kChunkRecordHeaderBytes;
  // Field layout: messageStartTime u64, messageEndTime u64, uncompressedSize
  // u64, uncompressedCrc u32, compression (u32 length + bytes),
  // compressedSize u64, then the (possibly compressed) records blob.
  const std::uint64_t uncompressed_size = read_u64(body + 16);
  const std::uint32_t compression_len = read_u32(body + 28);
  const std::size_t records_header = 8 + 8 + 8 + 4 + 4 + compression_len + 8;
  if (body_size < records_header) {
    out.error = "chunk record truncated inside its header";
    outcome.spare_record = std::move(record);
    outcome.spare_dst = std::move(dst);
    return outcome;
  }
  const char * compression_ptr = reinterpret_cast<const char *>(body + 32);
  const std::string compression(compression_ptr, compression_len);
  const std::uint64_t compressed_size = read_u64(body + 32 + compression_len);
  if (body_size - records_header < compressed_size) {
    out.error = "chunk record's compressed blob exceeds the record";
    outcome.spare_record = std::move(record);
    outcome.spare_dst = std::move(dst);
    return outcome;
  }
  const std::byte * blob = body + records_header;
  const std::size_t blob_offset = kChunkRecordHeaderBytes + records_header;

  if (compression.empty() || compression == "none") {
    // Hand the raw record buffer over as-is; the blob is served in place.
    out.offset = blob_offset;
    out.size = static_cast<std::size_t>(compressed_size);
    out.records = std::move(record);
    outcome.spare_dst = std::move(dst);
    return outcome;
  }
  if (compression == "zstd") {
    // Grow-only resize: a pooled buffer arriving at >= uncompressed_size is
    // shrunk without touching memory, so no zero-fill pass in steady state.
    dst.resize(uncompressed_size);
    const std::size_t written = ZSTD_decompressDCtx(
      dctx, dst.data(), dst.size(), blob, static_cast<std::size_t>(compressed_size));
    outcome.spare_record = std::move(record);
    if (ZSTD_isError(written) != 0u) {
      out.error = std::string("zstd decompress failed: ") + ZSTD_getErrorName(written);
      outcome.spare_dst = std::move(dst);
    } else if (written != uncompressed_size) {
      out.error = "zstd decompress produced an unexpected size";
      outcome.spare_dst = std::move(dst);
    } else {
      out.offset = 0;
      out.size = static_cast<std::size_t>(uncompressed_size);
      out.records = std::move(dst);
    }
    return outcome;
  }
  out.error = "unsupported chunk compression: " + compression;
  outcome.spare_record = std::move(record);
  outcome.spare_dst = std::move(dst);
  return outcome;
}

}  // namespace

ChunkPrefetcher::ChunkPrefetcher(
  std::filesystem::path path, std::vector<ChunkRef> schedule, int num_threads)
: path_(std::move(path)),
  schedule_(std::move(schedule)),
  lookahead_(static_cast<std::size_t>(std::max(num_threads, 1)) + 2)
{
  const int workers = std::max(num_threads, 1);
  workers_.reserve(static_cast<std::size_t>(workers));
  for (int i = 0; i < workers; ++i) {
    workers_.emplace_back([this]() { worker_loop(); });
  }
}

ChunkPrefetcher::~ChunkPrefetcher()
{
  {
    std::lock_guard lock(mutex_);
    cancel_ = true;
  }
  cv_.notify_all();
  // jthread joins on destruction anyway; the explicit join keeps the worker
  // file handles from outliving the object in an undefined order.
  for (auto & w : workers_) {
    w.join();
  }
}

PrefetchedChunk ChunkPrefetcher::get(std::size_t index)
{
  std::unique_lock lock(mutex_);
  cv_.wait(lock, [&] { return cancel_ || ready_.count(index) != 0; });
  if (cancel_ && ready_.count(index) == 0) {
    PrefetchedChunk out;
    out.error = worker_fatal_error_.empty() ? "chunk prefetch cancelled" : worker_fatal_error_;
    return out;
  }
  auto node = ready_.extract(index);
  consumed_ = index + 1;
  lock.unlock();
  cv_.notify_all();
  PrefetchedChunk out = std::move(node.mapped());
  return out;
}

void ChunkPrefetcher::recycle(std::vector<std::byte> && buf)
{
  if (buf.capacity() == 0) {
    return;
  }
  std::lock_guard lock(mutex_);
  buffer_pool_.push_back(std::move(buf));
}

std::vector<std::byte> ChunkPrefetcher::take_pooled_buffer()
{
  std::lock_guard lock(mutex_);
  if (buffer_pool_.empty()) {
    return {};
  }
  std::vector<std::byte> buf = std::move(buffer_pool_.back());
  buffer_pool_.pop_back();
  return buf;
}

void ChunkPrefetcher::worker_loop()
{
  std::ifstream file(path_, std::ios::binary);
  // Each worker owns one reusable zstd context; per-chunk contexts would pay
  // the (allocation + init) setup on every decompress call. jthread bodies
  // must not throw (an uncaught exception here would call std::terminate),
  // so an allocation failure cancels the whole prefetch instead.
  const ZstdDCtxPtr dctx{ZSTD_createDCtx()};
  if (dctx == nullptr) {
    std::lock_guard lock(mutex_);
    cancel_ = true;
    if (worker_fatal_error_.empty()) {
      worker_fatal_error_ = "chunk prefetch worker failed to allocate a ZSTD_DCtx";
    }
    cv_.notify_all();
    return;
  }

  // `raw` (bytes read from disk) and `scratch` (zstd destination) are the
  // worker's two reusable buffers. Each iteration overwrites both from
  // decompress_chunk_record's outcome unconditionally — ownership is never
  // inferred from a moved-from vector's (unspecified) post-move state.
  std::vector<std::byte> raw;
  std::vector<std::byte> scratch;
  while (true) {
    std::size_t index = 0;
    {
      std::unique_lock lock(mutex_);
      cv_.wait(lock, [&] {
        return cancel_ || (next_claim_ < schedule_.size() && next_claim_ < consumed_ + lookahead_);
      });
      if (cancel_) {
        return;
      }
      index = next_claim_++;
    }

    const ChunkRef & ref = schedule_[index];
    PrefetchedChunk result;
    if (raw.capacity() == 0) {
      raw = take_pooled_buffer();
    }
    try {
      raw.resize(ref.length);
      file.clear();
      file.seekg(static_cast<std::streamoff>(ref.start_offset));
      file.read(reinterpret_cast<char *>(raw.data()), static_cast<std::streamsize>(ref.length));
      if (!file || static_cast<std::uint64_t>(file.gcount()) != ref.length) {
        result.error = "short read of chunk record from " + path_.string();
      } else {
        if (scratch.capacity() == 0) {
          scratch = take_pooled_buffer();
        }
        auto outcome = decompress_chunk_record(std::move(raw), std::move(scratch), dctx.get());
        result = std::move(outcome.chunk);
        raw = std::move(outcome.spare_record);
        scratch = std::move(outcome.spare_dst);
      }
    } catch (const std::exception & e) {
      // Chunk length/size fields come straight from the file (or, upstream,
      // from the summary's ChunkIndex), so a corrupt value can overflow a
      // resize/allocation. Surface it as a normal read error instead of
      // letting it escape this jthread body.
      result.error = std::string("chunk prefetch worker exception: ") + e.what();
    }

    {
      std::lock_guard lock(mutex_);
      ready_.emplace(index, std::move(result));
    }
    cv_.notify_all();
  }
}

}  // namespace bagwiz::io::detail
