// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagcli/core/logging.hpp"
#include "bagcli/io/bag_io.hpp"
#include "bagcli/io/mcap_reader.hpp"
#include "bagcli/io/metadata_yaml.hpp"
#include "bagcli/io/sqlite3_reader.hpp"

#include <array>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

namespace bagcli::io
{

namespace
{
constexpr const char * kLogger = "bagcli.io";

// MCAP magic prefix: 0x89, 'M', 'C', 'A', 'P', 0x30
constexpr std::array<unsigned char, 6> kMcapMagic = {0x89, 'M', 'C', 'A', 'P', '0'};

// SQLite3 header prefix (first 16 bytes are "SQLite format 3\0").
constexpr const char * kSqliteMagic = "SQLite format 3";

Format detect_format_from_file(const std::filesystem::path & path)
{
  // Fast path: by extension.
  const auto ext = path.extension().string();
  if (ext == ".mcap") {
    return Format::Mcap;
  }
  if (ext == ".db3") {
    return Format::Sqlite3;
  }

  // Fallback: peek magic bytes.
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    throw std::runtime_error("cannot open file for format detection: " + path.string());
  }
  std::array<char, 16> buf{};
  f.read(buf.data(), buf.size());
  const auto read = f.gcount();

  if (static_cast<std::size_t>(read) >= kMcapMagic.size()) {
    if (std::memcmp(buf.data(), kMcapMagic.data(), kMcapMagic.size()) == 0) {
      return Format::Mcap;
    }
  }
  if (read >= static_cast<std::streamsize>(std::strlen(kSqliteMagic))) {
    if (std::memcmp(buf.data(), kSqliteMagic, std::strlen(kSqliteMagic)) == 0) {
      return Format::Sqlite3;
    }
  }

  throw std::runtime_error("unable to detect bag format: " + path.string());
}

}  // namespace

std::unique_ptr<BagReader> open_read(const std::filesystem::path & path, OpenOptions options)
{
  std::error_code ec;
  const bool exists = std::filesystem::exists(path, ec);
  if (ec || !exists) {
    throw std::runtime_error("path does not exist: " + path.string());
  }

  const bool is_dir = std::filesystem::is_directory(path, ec);

  if (is_dir) {
    const auto metadata_path = path / "metadata.yaml";
    if (!std::filesystem::exists(metadata_path)) {
      throw std::runtime_error(
        "directory is not a rosbag2 bag (missing metadata.yaml): " + path.string());
    }
    const auto md = load_metadata_yaml(metadata_path);
    if (md.storage_identifier == "mcap") {
      return detail::open_mcap_directory(path);
    }
    if (md.storage_identifier == "sqlite3") {
      return detail::open_sqlite3_directory(path);
    }
    throw std::runtime_error("unknown storage_identifier: " + md.storage_identifier);
  }

  Format fmt = options.format;
  if (fmt == Format::Auto) {
    fmt = detect_format_from_file(path);
  }

  switch (fmt) {
    case Format::Mcap:
      return detail::open_mcap_file(path);
    case Format::Sqlite3:
      return detail::open_sqlite3_file(path);
    case Format::Auto:
      throw std::runtime_error("format auto-detection failed: " + path.string());
  }
  throw std::runtime_error("unreachable: unhandled Format");
}

std::unique_ptr<BagWriter> open_write(const std::filesystem::path & path, CreateOptions /*options*/)
{
  BAGCLI_LOG_ERROR(kLogger, "open_write not yet implemented for %s", path.c_str());
  throw std::runtime_error("bagcli::io::open_write is not yet implemented");
}

}  // namespace bagcli::io
