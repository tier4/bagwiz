// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__IO__SQLITE3_READER_HPP_
#define BAGWIZ__IO__SQLITE3_READER_HPP_

#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/file_decompressor.hpp"
#include "bagwiz/io/message_decompressor.hpp"
#include "bagwiz/io/metadata_yaml.hpp"

#include <filesystem>
#include <memory>

namespace bagwiz::io::detail
{

// Open a single .db3 file as a BagReader. `decompressor` is null for an
// uncompressed bag; pass a non-null instance to transparently decompress
// every message payload before it reaches the caller (used for rosbag2
// `compression_mode: MESSAGE` bags).
//
// `temp` lets the caller hand off ownership of a temporary .db3 that was
// decompressed from a FILE-mode `.db3.zstd` envelope: the reader keeps it
// alive for its whole lifetime and the file is removed when the reader is
// destroyed. Pass a default-constructed (empty) TempFile for ordinary
// on-disk bags.
std::unique_ptr<BagReader> open_sqlite3_file(
  const std::filesystem::path & path, std::shared_ptr<MessageDecompressor> decompressor = nullptr,
  TempFile temp = {});

// Open a directory of .db3 shards. `metadata` must describe the layout
// (storage_identifier and relative_file_paths at minimum) — the caller is
// responsible for sourcing it from metadata.yaml or MetadataComputer.
// `decompressor` is null for uncompressed bags and non-null for MESSAGE-mode
// bags; the same instance is shared across shards so the underlying
// ZSTD_DCtx is reused for the entire iteration.
//
// When `zstd_file_envelope` is true the shards named in `metadata` are
// whole-database `.db3.zstd` envelopes (rosbag2 `compression_mode: FILE`):
// each shard is stream-decompressed to a temp `.db3` lazily, the first time
// it is iterated, and the temp file is removed when the reader closes.
// Metadata-only access (topics(), summary stats) never triggers
// decompression.
std::unique_ptr<BagReader> open_sqlite3_directory(
  const std::filesystem::path & dir, BagMetadata md,
  std::shared_ptr<MessageDecompressor> decompressor = nullptr, bool zstd_file_envelope = false);

}  // namespace bagwiz::io::detail

#endif  // BAGWIZ__IO__SQLITE3_READER_HPP_
