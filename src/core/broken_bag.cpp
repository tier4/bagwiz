// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/broken_bag.hpp"

#include "bagwiz/io/bag_io.hpp"

#include <algorithm>
#include <cctype>
#include <exception>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <system_error>
#include <vector>

namespace bagwiz::core
{

namespace
{

namespace fs = std::filesystem;

std::string to_lower_copy(std::string s)
{
  for (auto & c : s) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return s;
}

bool has_metadata_yaml(const fs::path & dir)
{
  std::error_code ec;
  const bool present = fs::exists(dir / "metadata.yaml", ec);
  return present && !ec;
}

// True for a regular file whose name ends in a recognized single-file bag
// extension: `.mcap`, `.db3`, or the whole-database zstd envelope `.db3.zstd`.
// `.mcap.zstd` is intentionally excluded — the io layer does not accept a
// zstd-wrapped MCAP as input, so it is not a bag bagwiz can read.
bool is_bag_file(const fs::path & path)
{
  std::error_code ec;
  if (!fs::is_regular_file(path, ec) || ec) {
    return false;
  }
  const std::string ext = to_lower_copy(path.extension().string());
  if (ext == ".mcap" || ext == ".db3") {
    return true;
  }
  // `.db3.zstd`: path::extension() is ".zstd"; the inner extension must be
  // ".db3" for the envelope to name a sqlite3 bag.
  if (ext == ".zstd") {
    return to_lower_copy(path.stem().extension().string()) == ".db3";
  }
  return false;
}

void sort_unique(std::vector<BagUnit> & units)
{
  std::sort(units.begin(), units.end(), [](const BagUnit & a, const BagUnit & b) {
    return a.path < b.path;
  });
  units.erase(
    std::unique(
      units.begin(), units.end(),
      [](const BagUnit & a, const BagUnit & b) { return a.path == b.path; }),
    units.end());
}

}  // namespace

std::vector<BagUnit> discover_bags(const fs::path & input)
{
  std::vector<BagUnit> units;

  std::error_code ec;
  if (!fs::exists(input, ec) || ec) {
    return units;
  }

  // A single file given directly: accept it when the io layer can recognize
  // its format. detect_format() sniffs magic bytes (so an extensionless or
  // renamed bag still counts) and resolves the `.db3.zstd` envelope, which is
  // worth the one open for an explicitly-named input.
  if (fs::is_regular_file(input, ec) && !ec) {
    if (is_bag_file(input) || io::detect_format(input) != io::Format::Auto) {
      units.push_back({input, false});
    }
    return units;
  }

  if (!fs::is_directory(input, ec) || ec) {
    return units;
  }

  // A directory that is itself a bag is one unit; its shards are not walked.
  if (has_metadata_yaml(input)) {
    units.push_back({input, true});
    return units;
  }

  // Otherwise recurse, collecting directory bags (without descending into
  // them) and single-file bags by extension.
  fs::recursive_directory_iterator it(input, fs::directory_options::skip_permission_denied, ec);
  if (ec) {
    return units;
  }
  const fs::recursive_directory_iterator end;
  while (it != end) {
    const fs::path entry = it->path();

    std::error_code dir_ec;
    const bool is_dir = it->is_directory(dir_ec) && !dir_ec;
    if (is_dir && has_metadata_yaml(entry)) {
      units.push_back({entry, true});
      it.disable_recursion_pending();  // do not descend into a bag directory
    } else if (!is_dir && is_bag_file(entry)) {
      units.push_back({entry, false});
    }

    it.increment(ec);
    if (ec) {
      break;  // iteration error: stop rather than loop forever
    }
  }

  sort_unique(units);
  return units;
}

std::optional<std::string> diagnose_bag(const fs::path & path, bool deep)
{
  try {
    auto reader = io::open_read(path);

    // Listing topics and computing summary statistics validate the storage
    // container's structure (MCAP summary/index, SQLite schema) without
    // decoding any message payload. For a directory bag without a complete
    // metadata summary, compute_stats() also opens every shard.
    (void)reader->topics();
    (void)reader->compute_stats();

    if (deep) {
      // Thorough pass: stream every message to EOF. This forces every shard
      // and chunk to be read (decompressing as needed) and surfaces payload
      // corruption a structural check cannot see. A fresh reader is used so
      // the structural pass above cannot have left iteration state behind.
      auto scan_reader = io::open_read(path);
      scan_reader->set_filter({});
      io::RawMessage raw;
      while (scan_reader->next(raw)) {
        // Raw bytes only; payloads are deliberately not decoded.
      }
    }

    return std::nullopt;
  } catch (const std::exception & e) {
    return std::string(e.what());
  }
}

std::error_code delete_bag(const BagUnit & unit)
{
  std::error_code ec;
  if (unit.is_directory_bag) {
    fs::remove_all(unit.path, ec);
  } else {
    fs::remove(unit.path, ec);
  }
  return ec;
}

}  // namespace bagwiz::core
