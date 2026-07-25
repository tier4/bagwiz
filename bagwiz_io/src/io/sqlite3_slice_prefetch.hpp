// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef IO__SQLITE3_SLICE_PREFETCH_HPP_
#define IO__SQLITE3_SLICE_PREFETCH_HPP_

#include "sqlite3_slice_schedule.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

// Src-local building block of the parallel db3 read path: a small worker pool
// that scans one .db3's messages table ahead of the consumer, one read-only
// connection per worker, over a fixed schedule of disjoint timestamp ranges
// with a bounded lookahead window. The mcap analogue is ChunkPrefetcher.
namespace bagwiz::io::detail
{

// One materialized message inside a prefetched slice. `offset`/`size` index
// into the owning PrefetchedSlice::blobs; `size == 0` means the row's payload
// was deliberately not read (see SliceScanSpec::payload_filter_active) or the
// blob was empty.
struct SliceRecord
{
  std::int64_t topic_id = 0;
  std::int64_t timestamp_ns = 0;
  std::size_t offset = 0;
  std::size_t size = 0;
};

// One fully scanned slice. `records` is in the exact order the serial scan
// would emit these rows. `blobs`/`records` are recycled back to the scanner
// once consumed. Both are meaningful only when `error` is empty.
struct PrefetchedSlice
{
  std::vector<std::byte> blobs;
  std::vector<SliceRecord> records;
  std::string error;
};

// Everything every slice statement shares, resolved once by the reader.
struct SliceScanSpec
{
  // Topic restriction, e.g. "topic_id IN (1,2)". Empty means no restriction.
  // Composed by the caller so the scanner never has to know about TopicInfo.
  std::string topic_clause;
  // Rows referencing a topic_id outside this set are dropped without being
  // materialized, matching the serial reader's `continue`.
  std::unordered_set<std::int64_t> known_topic_ids;
  // Payload allow-list, meaningful only when payload_filter_active. Rows on
  // other topics never call sqlite3_column_blob, so SQLite leaves their
  // overflow pages unread — the whole point of ReadFilter::payload_topics.
  std::unordered_set<std::int64_t> payload_topic_ids;
  bool payload_filter_active = false;
};

// Target bytes per slice, from BAGWIZ_DB3_SLICE_BYTES (default 32 MiB).
// Peak prefetch memory is roughly (workers + 2) * this value.
std::uint64_t resolve_slice_bytes(const char * logger);

class SliceScanner
{
public:
  SliceScanner(
    std::filesystem::path path, SliceScanSpec spec, std::vector<SliceRef> schedule,
    int num_threads);
  ~SliceScanner();

  SliceScanner(const SliceScanner &) = delete;
  SliceScanner & operator=(const SliceScanner &) = delete;
  SliceScanner(SliceScanner &&) = delete;
  SliceScanner & operator=(SliceScanner &&) = delete;

  [[nodiscard]] std::size_t size() const { return schedule_.size(); }

  // Blocking hand-off of schedule entry `index`; must be called with ascending
  // indexes (0, 1, ...), each exactly once.
  [[nodiscard]] PrefetchedSlice get(std::size_t index);

  // Return a consumed slice's buffers for reuse by the workers, keeping the
  // steady state free of large allocations.
  void recycle(std::vector<std::byte> && blobs, std::vector<SliceRecord> && records);

private:
  [[nodiscard]] std::vector<std::byte> take_pooled_blobs();
  [[nodiscard]] std::vector<SliceRecord> take_pooled_records();
  void worker_loop();

  const std::filesystem::path path_;
  const SliceScanSpec spec_;
  const std::vector<SliceRef> schedule_;
  const std::size_t lookahead_;

  std::mutex mutex_;
  std::condition_variable cv_;
  std::size_t next_claim_ = 0;  // next schedule index a worker may take
  std::size_t consumed_ = 0;    // entries already handed to the consumer
  bool cancel_ = false;
  std::string worker_fatal_error_;  // non-empty => a worker failed to start
  std::map<std::size_t, PrefetchedSlice> ready_;
  std::vector<std::vector<std::byte>> blob_pool_;
  std::vector<std::vector<SliceRecord>> record_pool_;

  std::vector<std::jthread> workers_;
};

}  // namespace bagwiz::io::detail

#endif  // IO__SQLITE3_SLICE_PREFETCH_HPP_
