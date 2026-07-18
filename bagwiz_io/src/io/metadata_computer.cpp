// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/metadata_computer.hpp"

#include "bagwiz/io/bag_io.hpp"

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace bagwiz::io
{

namespace
{

// One discovered shard plus its parsed shard index. The index is -1 when
// the stem does not end in `_<digits>`; such files sort after indexed ones
// so well-formed bags (the common case) get a deterministic numeric order.
struct IndexedShard
{
  std::filesystem::path relative_path;
  std::int64_t index{-1};
};

// Returns the value of the trailing `_<digits>` group in `stem`, or -1 if
// there isn't one. The function is noexcept-by-design: parse failures are
// reported through the sentinel so the caller's sort stays total.
std::int64_t parse_trailing_index(std::string_view stem) noexcept
{
  const auto pos = stem.rfind('_');
  if (pos == std::string_view::npos || pos + 1 >= stem.size()) {
    return -1;
  }
  const auto digits = stem.substr(pos + 1);
  if (digits.empty()) {
    return -1;
  }
  const auto all_digits = std::all_of(digits.begin(), digits.end(), [](char c) {
    return std::isdigit(static_cast<unsigned char>(c)) != 0;
  });
  if (!all_digits) {
    return -1;
  }
  try {
    return static_cast<std::int64_t>(std::stoll(std::string(digits)));
  } catch (const std::exception &) {
    // Overflow on absurdly large `_<n>` -> treat as unindexed.
    return -1;
  }
}

bool shard_less(const IndexedShard & a, const IndexedShard & b) noexcept
{
  // Indexed shards come first so single-shard or fully-numbered bags get
  // their natural ordering. Ties within each bucket fall back to a path
  // comparison for determinism.
  const bool a_indexed = a.index >= 0;
  const bool b_indexed = b.index >= 0;
  if (a_indexed != b_indexed) {
    return a_indexed;
  }
  if (a.index != b.index) {
    return a.index < b.index;
  }
  return a.relative_path < b.relative_path;
}

}  // namespace

namespace
{

// Builds the BagMetadata once the storage_identifier has been resolved.
// Consumes `shards` by reference because the caller still owns the
// chosen vector — moving each filename out of it is fine, but the
// vector itself is destroyed when the caller's scope ends.
BagMetadata build_metadata(
  const std::filesystem::path & dir, std::vector<IndexedShard> & shards, std::string storage_id,
  Format expected_format)
{
  std::sort(shards.begin(), shards.end(), shard_less);

  // One magic-byte sniff on the first shard guards against renamed or
  // corrupt files. detect_format() reads at most 16 bytes; full-file scans
  // never happen here.
  const auto first_full = dir / shards.front().relative_path;
  if (detect_format(first_full) != expected_format) {
    throw std::runtime_error(
      "first shard's magic bytes do not match its `." + storage_id +
      "` extension: " + first_full.string());
  }

  BagMetadata md;
  md.storage_identifier = std::move(storage_id);
  md.relative_file_paths.reserve(shards.size());
  for (auto & s : shards) {
    md.relative_file_paths.push_back(std::move(s.relative_path));
  }
  return md;
}

}  // namespace

BagMetadata MetadataComputer::compute(const std::filesystem::path & dir)
{
  std::error_code ec;
  if (!std::filesystem::is_directory(dir, ec) || ec) {
    throw std::runtime_error("not a bag directory: " + dir.string());
  }

  std::vector<IndexedShard> mcaps;
  std::vector<IndexedShard> sqlites;
  // `directory_iterator` can throw `filesystem_error` (e.g. dir is
  // removed mid-scan on a network mount). Normalise it to runtime_error
  // so callers only see the exception type documented by the header.
  try {
    for (const auto & entry : std::filesystem::directory_iterator(dir)) {
      if (!entry.is_regular_file()) {
        continue;
      }
      const auto & p = entry.path();
      const auto ext = p.extension().string();
      if (ext == ".mcap") {
        mcaps.push_back({p.filename(), parse_trailing_index(p.stem().string())});
      } else if (ext == ".db3") {
        sqlites.push_back({p.filename(), parse_trailing_index(p.stem().string())});
      }
    }
  } catch (const std::filesystem::filesystem_error & e) {
    throw std::runtime_error("failed to scan bag directory '" + dir.string() + "': " + e.what());
  }

  if (!mcaps.empty() && !sqlites.empty()) {
    throw std::runtime_error(
      "directory mixes .mcap and .db3 shards; cannot infer storage_identifier: " + dir.string());
  }
  if (!mcaps.empty()) {
    return build_metadata(dir, mcaps, "mcap", Format::Mcap);
  }
  if (!sqlites.empty()) {
    return build_metadata(dir, sqlites, "sqlite3", Format::Sqlite3);
  }
  throw std::runtime_error("no .mcap or .db3 shards found in directory: " + dir.string());
}

}  // namespace bagwiz::io
