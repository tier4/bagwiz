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
#include <fstream>
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

// Parse one raw chunk record and produce its decompressed records blob.
PrefetchedChunk decompress_chunk_record(const std::vector<std::byte> & record)
{
  PrefetchedChunk out;
  if (record.size() < kChunkRecordHeaderBytes + kMinChunkBodyBytes) {
    out.error = "chunk record truncated";
    return out;
  }
  if (std::to_integer<std::uint8_t>(record[0]) != kChunkOpCode) {
    out.error = "record at chunk offset is not a Chunk record";
    return out;
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
    return out;
  }
  const char * compression_ptr = reinterpret_cast<const char *>(body + 32);
  const std::string compression(compression_ptr, compression_len);
  const std::uint64_t compressed_size = read_u64(body + 32 + compression_len);
  if (body_size - records_header < compressed_size) {
    out.error = "chunk record's compressed blob exceeds the record";
    return out;
  }
  const std::byte * blob = body + records_header;

  if (compression.empty() || compression == "none") {
    out.records.assign(blob, blob + compressed_size);
    return out;
  }
  if (compression == "zstd") {
    out.records.resize(uncompressed_size);
    const std::size_t written = ZSTD_decompress(
      out.records.data(), out.records.size(), blob, static_cast<std::size_t>(compressed_size));
    if (ZSTD_isError(written) != 0u) {
      out.records.clear();
      out.error = std::string("zstd decompress failed: ") + ZSTD_getErrorName(written);
    } else if (written != uncompressed_size) {
      out.records.clear();
      out.error = "zstd decompress produced an unexpected size";
    }
    return out;
  }
  out.error = "unsupported chunk compression: " + compression;
  return out;
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
    out.error = "chunk prefetch cancelled";
    return out;
  }
  auto node = ready_.extract(index);
  consumed_ = index + 1;
  lock.unlock();
  cv_.notify_all();
  PrefetchedChunk out = std::move(node.mapped());
  return out;
}

void ChunkPrefetcher::worker_loop()
{
  std::ifstream file(path_, std::ios::binary);
  std::vector<std::byte> raw;
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
    raw.resize(ref.length);
    file.clear();
    file.seekg(static_cast<std::streamoff>(ref.start_offset));
    file.read(reinterpret_cast<char *>(raw.data()), static_cast<std::streamsize>(ref.length));
    if (!file || static_cast<std::uint64_t>(file.gcount()) != ref.length) {
      result.error = "short read of chunk record from " + path_.string();
    } else {
      result = decompress_chunk_record(raw);
    }

    {
      std::lock_guard lock(mutex_);
      ready_.emplace(index, std::move(result));
    }
    cv_.notify_all();
  }
}

}  // namespace bagwiz::io::detail
