// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__IO__METADATA_COMPUTER_HPP_
#define BAGWIZ__IO__METADATA_COMPUTER_HPP_

#include "bagwiz/io/metadata_yaml.hpp"

#include <filesystem>

namespace bagwiz::io
{

// Reconstructs a BagMetadata view for a bag directory whose metadata.yaml is
// absent (or otherwise has to be regenerated). bagwiz never delegates this to
// rosbag2 / rosbag2_storage's get_metadata(); this class is the single
// in-house entry point for the operation.
//
// Fast path: one directory listing plus one magic-byte sniff. Message
// records are not scanned and no shard is opened. The returned BagMetadata
// leaves has_summary=false and topics empty so callers (*ShardReader)
// continue to derive topics and stats via the same per-shard fallback they
// already use for summary-less YAML inputs. Heavy work stays lazy and only
// runs when a consumer actually asks for it.
class MetadataComputer
{
public:
  // Reconstruct a BagMetadata for `dir`. Throws std::runtime_error when:
  //   - `dir` does not exist or is not a directory,
  //   - the directory contains no `.mcap` or `.db3` shard files,
  //   - it mixes both extensions (storage_identifier would be ambiguous),
  //   - the first shard's magic prefix disagrees with its extension
  //     (catches renamed/corrupt files without a record-level scan).
  //
  // Shards are sorted by the trailing `_<n>` integer in the stem so that
  // 10+-shard bags do not end up with `_10` between `_1` and `_2` (the
  // failure mode of plain lexicographic sort).
  //
  // Static because the operation is stateless. The class is kept (rather
  // than a free function) to give future configuration knobs — alternate
  // shard orderings, strictness levels — a natural home without breaking
  // call sites.
  [[nodiscard]] static BagMetadata compute(const std::filesystem::path & dir);
};

}  // namespace bagwiz::io

#endif  // BAGWIZ__IO__METADATA_COMPUTER_HPP_
