// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "mcap_chunk_prefetch.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "mcap_chunk_codec.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <algorithm>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

namespace bagwiz::io::detail
{

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
      // The shared codec also parses the chunk's header times; the prefetch
      // consumer only needs the records blob, so those fields are dropped.
      auto decoded = decompress_chunk_record(raw);
      result.records = std::move(decoded.records);
      result.error = std::move(decoded.error);
    }

    {
      std::lock_guard lock(mutex_);
      ready_.emplace(index, std::move(result));
    }
    cv_.notify_all();
  }
}

}  // namespace bagwiz::io::detail
