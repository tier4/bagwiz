// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef IO__MCAP_PARALLEL_CHUNK_WRITER_HPP_
#define IO__MCAP_PARALLEL_CHUNK_WRITER_HPP_

#include <mcap/types.hpp>

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

namespace bagwiz::io::detail
{

// Chunk-compression worker count for the parallel mcap write path. Defaults to
// 8, capped at the host's hardware concurrency so low-core machines keep a
// smaller worker count. BAGWIZ_WRITE_THREADS overrides the default, and 0 or 1
// selects the serial libmcap writer (the debugging escape hatch).
int resolve_write_threads();

// Chunked mcap file writer that moves chunk compression off the caller thread:
// the calling thread serializes message records into chunk-sized staging
// buffers, a worker pool compresses each full buffer (one-shot, single-threaded
// per chunk, so the compressed bytes do not depend on the worker count), and a
// sequencer thread writes Chunk + MessageIndex records to the file in emission
// order. Used by McapFileWriter for zstd/lz4 outputs; uncompressed output has
// no chunk encode to parallelize and stays on mcap::McapWriter.
//
// write_schema/write_channel/write_message/close must be called from a single
// thread (the BagWriter contract the pipeline relies on). Schema and Channel
// records are emitted as top-level data-section records — the same layout the
// chunk pass-through engine produces — with ids pre-assigned by the caller.
class ParallelChunkMcapWriter
{
public:
  // `compression` is "zstd" or "lz4" (the normalized mcap codec name).
  // `num_threads` is the compression worker count (>= 2; the caller resolves
  // it via resolve_write_threads).
  ParallelChunkMcapWriter(
    const std::filesystem::path & path, std::string compression, std::uint64_t chunk_size,
    int num_threads);
  ~ParallelChunkMcapWriter();

  ParallelChunkMcapWriter(const ParallelChunkMcapWriter &) = delete;
  ParallelChunkMcapWriter & operator=(const ParallelChunkMcapWriter &) = delete;
  ParallelChunkMcapWriter(ParallelChunkMcapWriter &&) = delete;
  ParallelChunkMcapWriter & operator=(ParallelChunkMcapWriter &&) = delete;

  void write_schema(const mcap::Schema & schema);
  void write_channel(const mcap::Channel & channel);
  void write_message(const mcap::Message & message);

  // Flush the partial chunk, drain the pool, and write the summary section.
  // Idempotent; throws std::runtime_error on a latched worker/IO failure.
  void close();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace bagwiz::io::detail

#endif  // IO__MCAP_PARALLEL_CHUNK_WRITER_HPP_
