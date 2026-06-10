// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__BROKEN_BAG_HPP_
#define BAGWIZ__CORE__BROKEN_BAG_HPP_

#include <filesystem>
#include <optional>
#include <string>
#include <system_error>
#include <vector>

namespace bagwiz::core
{

// A single rosbag discovered on disk. A bag is either a single-file bag
// (*.mcap, *.db3, *.db3.zstd) or a directory-layout bag (a directory
// containing metadata.yaml). The distinction drives deletion: a directory
// bag must be removed recursively, a single-file bag is one file.
struct BagUnit
{
  std::filesystem::path path;
  bool is_directory_bag = false;
};

// Recursively discover rosbag units at or beneath `input`.
//
//   - `input` is a single file: treated as one bag unit when its storage
//     format is recognizable (by extension or magic bytes), otherwise the
//     result is empty.
//   - `input` is a directory containing metadata.yaml: one directory-bag
//     unit. Its shards are never reported as separate units.
//   - `input` is any other directory: walked recursively. Every directory
//     that contains metadata.yaml becomes one directory-bag unit (and is not
//     descended into); every regular file with a recognized bag extension
//     (.mcap, .db3, .db3.zstd) becomes one single-file unit.
//
// Discovery during the recursive walk is extension-based so a large tree is
// not opened file-by-file. Results are sorted by path and de-duplicated.
// Best-effort: subdirectories that cannot be read are skipped rather than
// throwing.
std::vector<BagUnit> discover_bags(const std::filesystem::path & input);

// Diagnose whether the bag at `path` can be read as a rosbag. Returns
// std::nullopt when the bag is healthy, or a short human-readable reason when
// it is broken.
//
// The default check is structural: it opens the bag, lists its topics, and
// computes summary statistics, all without decoding any message payload. This
// validates the storage container (MCAP header/footer/index, SQLite database
// structure). A mismatch between metadata.yaml's recorded statistics and the
// actual records is NOT treated as broken; only an unreadable storage
// container is.
//
// Limitation of the structural check: for a directory bag whose metadata.yaml
// already carries a complete message summary, compute_stats() trusts that
// summary and does not re-open every shard, so a shard corrupted after the
// bag was finalized can be missed. Pass `deep = true` to additionally stream
// every message to end-of-file (still without decoding payloads); that forces
// every shard and chunk to be read and catches such corruption, at the cost
// of reading the whole bag.
std::optional<std::string> diagnose_bag(const std::filesystem::path & path, bool deep = false);

// Delete a discovered bag unit: remove_all() for a directory bag, remove()
// for a single-file bag. Returns a default-constructed (success) error_code
// on success, or the first failure's error_code.
std::error_code delete_bag(const BagUnit & unit);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__BROKEN_BAG_HPP_
