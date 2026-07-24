// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "mcap_chunk_codec.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <mcap/reader.hpp>
#include <mcap/writer.hpp>

#include <zstd.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
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

}  // namespace

DecodedChunk decompress_chunk_record(std::span<const std::byte> record)
{
  DecodedChunk out;
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
  out.message_start_time = read_u64(body);
  out.message_end_time = read_u64(body + 8);
  const std::uint64_t uncompressed_size = read_u64(body + 16);
  const std::uint32_t compression_len = read_u32(body + 28);
  const std::size_t records_header = 8 + 8 + 8 + 4 + 4 + compression_len + 8;
  if (body_size < records_header) {
    out.error = "chunk record truncated inside its header";
    return out;
  }
  const char * compression_ptr = reinterpret_cast<const char *>(body + 32);
  out.compression.assign(compression_ptr, compression_len);
  const std::uint64_t compressed_size = read_u64(body + 32 + compression_len);
  if (body_size - records_header < compressed_size) {
    out.error = "chunk record's compressed blob exceeds the record";
    return out;
  }
  const std::byte * blob = body + records_header;

  if (out.compression.empty() || out.compression == "none") {
    out.records.assign(blob, blob + compressed_size);
    return out;
  }
  if (out.compression == "zstd") {
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
  if (out.compression == "lz4") {
    mcap::LZ4Reader lz4;
    mcap::ByteArray decompressed;
    const auto status = lz4.decompressAll(blob, compressed_size, uncompressed_size, &decompressed);
    if (!status.ok()) {
      out.error = "lz4 decompress failed: " + status.message;
    } else if (decompressed.size() != uncompressed_size) {
      out.error = "lz4 decompress produced an unexpected size";
    } else {
      out.records = std::move(decompressed);
    }
    return out;
  }
  out.error = "unsupported chunk compression: " + out.compression;
  return out;
}

std::vector<std::byte> compress_chunk_records(
  std::span<const std::byte> records, std::string_view compression)
{
  if (compression.empty() || compression == "none") {
    return {records.begin(), records.end()};
  }

  // IChunkWriter's chunkSize constructor argument only pre-sizes the staging
  // buffers, so the exact blob size is the natural value (clamped to 1 so an
  // empty blob stays valid).
  const std::uint64_t buffer_hint = records.empty() ? 1 : records.size();
  std::unique_ptr<mcap::IChunkWriter> writer;
  if (compression == "zstd") {
    writer = std::make_unique<mcap::ZStdWriter>(mcap::CompressionLevel::Default, buffer_hint);
  } else if (compression == "lz4") {
    writer = std::make_unique<mcap::LZ4Writer>(mcap::CompressionLevel::Default, buffer_hint);
  } else {
    throw std::runtime_error("unsupported chunk compression: " + std::string(compression));
  }
  writer->write(records.data(), records.size());
  writer->end();
  const std::byte * data = writer->compressedData();
  return {data, data + writer->compressedSize()};
}

}  // namespace bagwiz::io::detail
