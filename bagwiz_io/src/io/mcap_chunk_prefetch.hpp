// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef IO__MCAP_CHUNK_PREFETCH_HPP_
#define IO__MCAP_CHUNK_PREFETCH_HPP_

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// Src-local building block of the parallel indexed mcap read path: a small
// worker pool that reads and decompresses one mcap file's chunk records ahead
// of the consumer, in a fixed schedule order, with a bounded lookahead window.
namespace bagwiz::io::detail
{

// One chunk record to prefetch, straight from the bag's ChunkIndex summary.
struct ChunkRef
{
  std::uint64_t start_offset = 0;  // file offset of the chunk record
  std::uint64_t length = 0;        // full record length (opcode + len + body)
};

// One decompressed chunk, ready for record iteration.
struct PrefetchedChunk
{
  std::vector<std::byte> records;  // the chunk's decompressed records blob
  std::string error;               // non-empty => read/decompress failed
};

// Decompresses `schedule`'s chunks on `num_threads` workers, each with its own
// file handle, at most a bounded lookahead ahead of the consumer. The consumer
// calls get(0), get(1), ... in ascending order; each call blocks until that
// chunk is ready and moves the buffer out. Destruction cancels outstanding
// work and joins the workers, so early consumer exit is safe.
class ChunkPrefetcher
{
public:
  ChunkPrefetcher(std::filesystem::path path, std::vector<ChunkRef> schedule, int num_threads);
  ~ChunkPrefetcher();

  ChunkPrefetcher(const ChunkPrefetcher &) = delete;
  ChunkPrefetcher & operator=(const ChunkPrefetcher &) = delete;
  ChunkPrefetcher(ChunkPrefetcher &&) = delete;
  ChunkPrefetcher & operator=(ChunkPrefetcher &&) = delete;

  [[nodiscard]] std::size_t size() const { return schedule_.size(); }

  // Blocking hand-off of schedule entry `index`; must be called with
  // ascending indexes (0, 1, ...), each exactly once.
  [[nodiscard]] PrefetchedChunk get(std::size_t index);

private:
  void worker_loop();

  const std::filesystem::path path_;
  const std::vector<ChunkRef> schedule_;
  const std::size_t lookahead_;

  std::mutex mutex_;
  std::condition_variable cv_;
  std::size_t next_claim_ = 0;  // next schedule index a worker may take
  std::size_t consumed_ = 0;    // entries already handed to the consumer
  bool cancel_ = false;
  std::map<std::size_t, PrefetchedChunk> ready_;

  std::vector<std::jthread> workers_;
};

}  // namespace bagwiz::io::detail

#endif  // IO__MCAP_CHUNK_PREFETCH_HPP_
