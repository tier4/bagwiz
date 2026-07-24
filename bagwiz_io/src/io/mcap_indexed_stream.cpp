// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "mcap_indexed_stream.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "mcap_read_job_compat.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <algorithm>
#include <cstring>
#include <memory>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace bagwiz::io::detail
{

namespace
{

constexpr std::size_t kRecordHeaderBytes = 1 + 8;  // opcode + record length
// Message record body prefix: channelId u16, sequence u32, logTime u64,
// publishTime u64 — the payload follows.
constexpr std::size_t kMessagePrefixBytes = 2 + 4 + 8 + 8;
constexpr std::uint8_t kMessageOpCode = 0x05;

std::uint16_t read_u16(const std::byte * p)
{
  std::uint16_t v = 0;
  std::memcpy(&v, p, sizeof(v));
  return v;
}

std::uint64_t read_u64(const std::byte * p)
{
  std::uint64_t v = 0;
  std::memcpy(&v, p, sizeof(v));
  return v;
}

}  // namespace

bool ParallelIndexedStream::supported(const mcap::McapReader & reader)
{
  const auto & chunk_indexes = reader.chunkIndexes();
  if (chunk_indexes.empty()) {
    return false;
  }
  return std::all_of(chunk_indexes.begin(), chunk_indexes.end(), [](const mcap::ChunkIndex & c) {
    return c.compression.empty() || c.compression == "none" || c.compression == "zstd";
  });
}

ParallelIndexedStream::ParallelIndexedStream(
  const std::filesystem::path & path, const mcap::McapReader & reader, Options options)
: options_(std::move(options))
{
  const std::uint64_t start_ns = options_.start_ns.value_or(0);
  const std::uint64_t end_ns = options_.end_ns.value_or(mcap::MaxTime);

  // Resolve the topic filter to channel ids exactly like IndexedMessageReader
  // resolves selectedChannels_: an empty set with a filter set means nothing
  // was selected.
  has_filter_ = static_cast<bool>(options_.topic_filter);
  if (has_filter_) {
    for (const auto & [channel_id, channel] : reader.channels()) {
      if (options_.topic_filter(std::string_view(channel->topic))) {
        selected_channels_.insert(channel_id);
      }
    }
  }

  // Select the chunks the indexed reader would visit: overlap the [start,
  // end) range, and — when a filter is set — carry at least one selected
  // channel in their message index.
  std::vector<const mcap::ChunkIndex *> selected;
  for (const auto & chunk : reader.chunkIndexes()) {
    if (chunk.messageEndTime < start_ns || chunk.messageStartTime >= end_ns) {
      continue;
    }
    if (has_filter_) {
      const bool has_selected = std::any_of(
        chunk.messageIndexOffsets.begin(), chunk.messageIndexOffsets.end(),
        [&](const auto & entry) { return selected_channels_.count(entry.first) != 0; });
      if (!has_selected) {
        continue;
      }
    }
    selected.push_back(&chunk);
  }

  // Chunk decompress jobs leave the ReadJobQueue in ascending
  // (messageStartTime, chunkStartOffset) order — message jobs interleave but
  // never reorder chunk jobs relative to each other — so prefetching in that
  // exact order guarantees get() is consumed strictly in schedule order.
  std::sort(selected.begin(), selected.end(), [](const auto * a, const auto * b) {
    if (a->messageStartTime != b->messageStartTime) {
      return a->messageStartTime < b->messageStartTime;
    }
    return a->chunkStartOffset < b->chunkStartOffset;
  });

  std::vector<ChunkRef> schedule;
  schedule.reserve(selected.size());
  for (std::size_t i = 0; i < selected.size(); ++i) {
    const auto & chunk = *selected[i];
    schedule.push_back({chunk.chunkStartOffset, chunk.chunkLength});
    schedule_index_by_offset_[chunk.chunkStartOffset] = i;
    queue_.push(
      mcap_compat::DecompressChunkJob{
        chunk.messageStartTime, chunk.messageEndTime, chunk.chunkStartOffset,
        chunk.chunkStartOffset + chunk.chunkLength + chunk.messageIndexLength});
  }
  prefetcher_ = std::make_unique<ChunkPrefetcher>(path, std::move(schedule), options_.num_threads);
}

std::size_t ParallelIndexedStream::find_free_slot()
{
  for (std::size_t i = 0; i < slots_.size(); ++i) {
    if (slots_[i].unread == 0) {
      return i;
    }
  }
  slots_.emplace_back();
  return slots_.size() - 1;
}

// Take the chunk's decompressed buffer from the prefetcher and push one
// ReadMessageJob per selected message, keyed exactly like the upstream
// reader: (logTime, RecordOffset{record start within the decompressed blob,
// chunk file offset}).
void ParallelIndexedStream::ingest_chunk(const mcap_compat::DecompressChunkJob & job)
{
  auto pre = prefetcher_->get(schedule_index_by_offset_.at(job.chunkStartOffset));
  if (!pre.error.empty()) {
    error_ = pre.error;
    return;
  }

  const std::uint64_t start_ns = options_.start_ns.value_or(0);
  const std::uint64_t end_ns = options_.end_ns.value_or(mcap::MaxTime);
  const std::size_t slot_index = find_free_slot();
  Slot & slot = slots_[slot_index];
  // Return the evicted chunk's buffer to the prefetcher pool before the new
  // chunk takes the slot, so workers reuse it instead of allocating afresh.
  if (slot.chunk.records.capacity() != 0) {
    prefetcher_->recycle(std::move(slot.chunk.records));
  }
  slot.chunk = std::move(pre);
  slot.chunk_start_offset = job.chunkStartOffset;
  slot.unread = 0;

  const std::byte * data = slot.chunk.data();
  const std::size_t size = slot.chunk.size;
  std::size_t pos = 0;
  while (pos + kRecordHeaderBytes <= size) {
    const auto opcode = std::to_integer<std::uint8_t>(data[pos]);
    const std::uint64_t length = read_u64(data + pos + 1);
    if (length > size - pos - kRecordHeaderBytes) {
      error_ = "chunk record overruns its decompressed blob";
      return;
    }
    if (opcode == kMessageOpCode && length >= kMessagePrefixBytes) {
      const std::byte * body = data + pos + kRecordHeaderBytes;
      const std::uint16_t channel_id = read_u16(body);
      const std::uint64_t log_time = read_u64(body + 6);
      const bool channel_ok = !has_filter_ || selected_channels_.count(channel_id) != 0;
      if (channel_ok && log_time >= start_ns && log_time < end_ns) {
        queue_.push(
          mcap_compat::ReadMessageJob{
            log_time, mcap::RecordOffset(pos, job.chunkStartOffset), slot_index});
        ++slot.unread;
      }
    }
    pos += kRecordHeaderBytes + length;
  }
}

bool ParallelIndexedStream::next(Message & out)
{
  if (!error_.empty()) {
    return false;
  }
  while (queue_.len() > 0) {
    auto job = queue_.pop();
    if (std::holds_alternative<mcap_compat::DecompressChunkJob>(job)) {
      ingest_chunk(std::get<mcap_compat::DecompressChunkJob>(job));
      if (!error_.empty()) {
        return false;
      }
      continue;
    }
    const auto & msg = std::get<mcap_compat::ReadMessageJob>(job);
    Slot & slot = slots_[msg.chunkReaderIndex];
    const std::byte * record = slot.chunk.data() + msg.offset.offset;
    const std::uint64_t length = read_u64(record + 1);
    const std::byte * body = record + kRecordHeaderBytes;
    out.channel_id = read_u16(body);
    out.log_time = read_u64(body + 6);
    out.payload = std::span<const std::byte>(
      body + kMessagePrefixBytes, static_cast<std::size_t>(length) - kMessagePrefixBytes);
    --slot.unread;
    return true;
  }
  return false;
}

}  // namespace bagwiz::io::detail
