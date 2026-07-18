// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/file_decompressor.hpp"
#include "bagwiz/io/mcap_reader.hpp"
#include "bagwiz/io/mcap_writer.hpp"
#include "bagwiz/io/message_decompressor.hpp"
#include "bagwiz/io/metadata_computer.hpp"
#include "bagwiz/io/metadata_yaml.hpp"
#include "bagwiz/io/sqlite3_reader.hpp"
#include "bagwiz/io/sqlite3_writer.hpp"

#include <array>
#include <cctype>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace bagwiz::io
{

namespace
{
constexpr const char * kLogger = "bagwiz.io";

// MCAP magic prefix: 0x89, 'M', 'C', 'A', 'P', 0x30
constexpr std::array<unsigned char, 6> kMcapMagic = {0x89, 'M', 'C', 'A', 'P', '0'};

// SQLite3 header prefix (first 16 bytes are "SQLite format 3\0").
constexpr const char * kSqliteMagic = "SQLite format 3";

std::string to_lower_copy(std::string s)
{
  for (auto & c : s) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return s;
}

// Decide what (if any) decompressor a directory bag needs based on its
// metadata. Returns nullptr for uncompressed / chunk-compressed-MCAP bags
// and a shared MessageDecompressor for MESSAGE+zstd bags. Throws on
// unsupported combinations (SQLite FILE-mode envelopes, non-zstd
// MESSAGE-mode, unknown modes). Kept separate from the `open_read`
// dispatcher so that dispatcher stays readable.
std::shared_ptr<MessageDecompressor> select_decompressor(
  const BagMetadata & md, const std::filesystem::path & path)
{
  // rosbag2 emits the mode in lowercase ("none" / "file" / "message") and
  // bagwiz's own writer matches that. Compare case-insensitively so both
  // conventions work in case some legacy bag emits "FILE" etc.
  const std::string mode = to_lower_copy(md.compression_mode);
  const std::string fmt = to_lower_copy(md.compression_format);

  if (mode.empty() || mode == "none") {
    return nullptr;
  }

  if (mode == "message") {
    if (fmt != "zstd") {
      throw std::runtime_error(
        "compression_format '" + md.compression_format +
        "' not supported (only 'zstd' is implemented for MESSAGE-mode bags); "
        "re-encode with `ros2 bag convert --compression-format zstd`");
    }
    BAGWIZ_LOG_INFO(
      kLogger, "%s: decompressing MESSAGE-mode (zstd) payloads on read", path.c_str());
    return std::make_shared<MessageDecompressor>(fmt);
  }

  if (mode == "file") {
    // MCAP FILE-mode = storage-internal chunk compression. libmcap
    // decompresses chunks transparently, so bagwiz needs no extra work.
    // SQLite FILE-mode = whole-database `.db3.zstd` envelope; that is handled
    // upstream in open_read() by routing to the envelope-decompressing
    // sqlite3 reader, so select_decompressor is never asked for it. No
    // per-message decompressor is involved in either case.
    return nullptr;
  }

  throw std::runtime_error("unknown compression_mode '" + mode + "' in " + path.string());
}

// True when `md` describes a rosbag2 `compression_mode: FILE` whole-database
// zstd envelope over sqlite3 storage (the `.db3.zstd` case). Case-insensitive
// on the mode/format strings to tolerate both rosbag2's uppercase enum names
// and bagwiz's lowercase output.
bool is_sqlite3_file_zstd_envelope(const BagMetadata & md)
{
  return to_lower_copy(md.compression_mode) == "file" && md.storage_identifier == "sqlite3";
}

// Resolve the storage format hidden inside a single-file zstd envelope from
// its extension alone, by stripping a trailing `.zstd` and re-inferring
// (e.g. `foo.db3.zstd` -> Sqlite3, `foo.mcap.zstd` -> Mcap). Returns
// Format::Auto when the inner extension is not recognised. Cheap and never
// touches file contents.
Format infer_inner_format_from_zstd_extension(const std::filesystem::path & path) noexcept
{
  if (to_lower_copy(path.extension().string()) != ".zstd") {
    return Format::Auto;
  }
  return infer_format_from_extension(path.stem());
}

// Magic-byte sniff: opens `path`, reads up to 16 bytes, and matches the
// MCAP / SQLite3 prefix. Returns Format::Auto on any failure (open error,
// short read, no match) so callers can fall through to higher-level
// diagnostics.
Format sniff_file_magic(const std::filesystem::path & path) noexcept
{
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    return Format::Auto;
  }
  std::array<char, 16> buf{};
  f.read(buf.data(), buf.size());
  const auto read = f.gcount();
  if (read < 0) {
    return Format::Auto;
  }
  const auto bytes = static_cast<std::size_t>(read);

  if (bytes >= kMcapMagic.size()) {
    if (std::memcmp(buf.data(), kMcapMagic.data(), kMcapMagic.size()) == 0) {
      return Format::Mcap;
    }
  }
  const auto sqlite_len = std::strlen(kSqliteMagic);
  if (bytes >= sqlite_len) {
    if (std::memcmp(buf.data(), kSqliteMagic, sqlite_len) == 0) {
      return Format::Sqlite3;
    }
  }
  return Format::Auto;
}

}  // namespace

// True when `path` is a rosbag2 FILE-mode (whole-database zstd envelope) bag:
// a directory whose metadata.yaml declares `compression_mode: FILE`, or a
// bare `.db3.zstd` single file. In both cases the bag cannot be read without
// first decompressing the entire database, so callers that only need cheap
// metadata (in-place rewrite guards, shell completion) use this to bail out
// before triggering a whole-database decompress. MESSAGE-mode bags return
// false: their topic list comes from metadata.yaml and payloads decompress
// per-message, so no envelope expansion is involved. Never throws.
bool is_file_compressed_bag(const std::filesystem::path & path) noexcept
{
  std::error_code ec;
  if (std::filesystem::is_directory(path, ec)) {
    const auto metadata_path = path / "metadata.yaml";
    if (!std::filesystem::exists(metadata_path, ec)) {
      return false;
    }
    try {
      const auto md = load_metadata_yaml(metadata_path);
      return to_lower_copy(md.compression_mode) == "file";
    } catch (const std::exception &) {
      return false;
    }
  }
  return is_zstd_file(path);
}

Format detect_format(const std::filesystem::path & path) noexcept
{
  std::error_code ec;
  if (!std::filesystem::exists(path, ec)) {
    return Format::Auto;
  }

  if (std::filesystem::is_directory(path, ec)) {
    const auto metadata_path = path / "metadata.yaml";
    if (!std::filesystem::exists(metadata_path, ec)) {
      return Format::Auto;
    }
    try {
      const auto md = load_metadata_yaml(metadata_path);
      if (md.storage_identifier == "mcap") {
        return Format::Mcap;
      }
      if (md.storage_identifier == "sqlite3") {
        return Format::Sqlite3;
      }
    } catch (const std::exception &) {
      // fall through; treat unparseable metadata.yaml as undetectable
    }
    return Format::Auto;
  }

  const Format sniffed = sniff_file_magic(path);
  if (sniffed != Format::Auto) {
    return sniffed;
  }
  // Single-file zstd envelope: resolve the inner storage format cheaply from
  // the `.db3.zstd` / `.mcap.zstd` extension without decompressing, so this
  // stays the documented "reads at most 16 bytes" cheap path.
  if (is_zstd_file(path)) {
    return infer_inner_format_from_zstd_extension(path);
  }
  return Format::Auto;
}

Format infer_format_from_extension(const std::filesystem::path & path) noexcept
{
  // path.extension() includes the leading dot. We compare case-sensitively
  // because rosbag2 / mcap tooling consistently emit lowercase extensions;
  // an uppercase `.MCAP` is unusual enough that we'd rather force `--storage`
  // than guess.
  const auto ext = path.extension().string();
  if (ext == ".mcap") {
    return Format::Mcap;
  }
  if (ext == ".db3") {
    return Format::Sqlite3;
  }
  return Format::Auto;
}

std::unique_ptr<BagReader> open_read(const std::filesystem::path & path, OpenOptions options)
{
  std::error_code ec;
  const bool exists = std::filesystem::exists(path, ec);
  if (ec || !exists) {
    throw std::runtime_error("path does not exist: " + path.string());
  }

  const bool is_dir = std::filesystem::is_directory(path, ec);

  if (is_dir) {
    // Prefer metadata.yaml when present; fall back to MetadataComputer
    // (directory listing + magic-byte sniff) when it's absent. The
    // fallback does not scan message records, so reconstruction stays
    // cheap even for multi-shard bags.
    const auto metadata_path = path / "metadata.yaml";
    auto md = std::filesystem::exists(metadata_path) ? load_metadata_yaml(metadata_path)
                                                     : MetadataComputer::compute(path);

    // FILE-mode `.db3.zstd` envelope over sqlite3: each shard is a whole
    // database wrapped in zstd. Route to the envelope-decompressing reader,
    // which expands each shard to a temp .db3 lazily on first iteration.
    // Validate the format here so an unsupported envelope (e.g. lz4) fails
    // fast before any shard work.
    if (is_sqlite3_file_zstd_envelope(md)) {
      const std::string fmt = to_lower_copy(md.compression_format);
      if (fmt != "zstd") {
        throw std::runtime_error(
          "FILE-level compression format '" + md.compression_format +
          "' is not supported on sqlite3 storage (only 'zstd' is implemented); "
          "re-encode with `ros2 bag convert --compression-format zstd` or "
          "decompress with `--compression-mode none` (input: " +
          path.string() + ")");
      }
      BAGWIZ_LOG_INFO(
        kLogger, "%s: FILE-mode (zstd) sqlite3 bag; shards decompressed to temp .db3 on read",
        path.c_str());
      return detail::open_sqlite3_directory(
        path, std::move(md), nullptr, /*zstd_file_envelope=*/true);
    }

    auto decompressor = select_decompressor(md, path);

    if (md.storage_identifier == "mcap") {
      return detail::open_mcap_directory(path, std::move(md), std::move(decompressor));
    }
    if (md.storage_identifier == "sqlite3") {
      return detail::open_sqlite3_directory(path, std::move(md), std::move(decompressor));
    }
    throw std::runtime_error("unknown storage_identifier: " + md.storage_identifier);
  }

  Format fmt = options.format;
  if (fmt == Format::Auto) {
    fmt = sniff_file_magic(path);
  }

  // Single-file zstd envelope (e.g. `foo.db3.zstd`): decompress to a temp
  // file, detect the inner format, and open that. Only sqlite3 inner content
  // is supported — rosbag2 never wraps MCAP this way (MCAP carries its own
  // internal chunk compression).
  if (fmt == Format::Auto && is_zstd_file(path)) {
    TempFile temp = decompress_zstd_file_to_temp(path);
    const Format inner = sniff_file_magic(temp.path());
    if (inner == Format::Sqlite3) {
      const auto temp_path = temp.path();
      return detail::open_sqlite3_file(temp_path, nullptr, std::move(temp));
    }
    if (inner == Format::Mcap) {
      throw std::runtime_error(
        "zstd-wrapped MCAP files are not supported (MCAP uses internal chunk "
        "compression; decompress the envelope first): " +
        path.string());
    }
    throw std::runtime_error("unable to detect bag format inside zstd envelope: " + path.string());
  }

  switch (fmt) {
    case Format::Mcap:
      return detail::open_mcap_file(path);
    case Format::Sqlite3:
      return detail::open_sqlite3_file(path);
    case Format::Auto:
      throw std::runtime_error("unable to detect bag format: " + path.string());
  }
  throw std::runtime_error("unreachable: unhandled Format");
}

std::unique_ptr<BagWriter> open_write(const std::filesystem::path & path, CreateOptions options)
{
  // Layout selection: if caller asked Auto, pick by whether the path looks
  // like a single .mcap/.db3 file (use SingleFile + matching format) or
  // anything else (use Directory).
  Layout layout = options.layout;
  Format format = options.format;
  if (layout == Layout::Auto) {
    const auto ext = path.extension().string();
    if (ext == ".mcap") {
      layout = Layout::SingleFile;
      if (format == Format::Auto) {
        format = Format::Mcap;
      }
    } else if (ext == ".db3") {
      layout = Layout::SingleFile;
      if (format == Format::Auto) {
        format = Format::Sqlite3;
      }
    } else {
      layout = Layout::Directory;
    }
  }
  if (format == Format::Auto) {
    format = Format::Mcap;
  }

  if (layout == Layout::Directory) {
    if (format == Format::Mcap) {
      return detail::create_mcap_directory(path, options);
    }
    if (format == Format::Sqlite3) {
      return detail::create_sqlite3_directory(path, options);
    }
    throw std::runtime_error("unsupported format for directory writer");
  }

  if (format == Format::Mcap) {
    return detail::create_mcap_file(path, options);
  }
  if (format == Format::Sqlite3) {
    return detail::create_sqlite3_file(path, options);
  }
  throw std::runtime_error("unsupported format for single-file writer");
}

CreateOptions create_options_inheriting_format(
  const std::filesystem::path & reference_path, const std::filesystem::path & output_path) noexcept
{
  CreateOptions opts;
  opts.format = Format::Auto;
  opts.layout = Layout::Auto;

  // The user named a single-file output: defer to factory-level
  // extension resolution so their choice wins (including any explicit
  // cross-format conversion such as a sqlite3 reference + .mcap output).
  if (infer_format_from_extension(output_path) != Format::Auto) {
    return opts;
  }

  // The reference may be a directory bag OR a single-file bag —
  // detect_format works on both. The output is always a directory in
  // this branch because the user's extension-less -o is what signalled
  // their directory intent.
  const auto detected = detect_format(reference_path);
  if (detected == Format::Auto) {
    return opts;
  }
  opts.format = detected;
  opts.layout = Layout::Directory;
  return opts;
}

CreateOptions create_options_preserving_storage(
  const std::filesystem::path & reference_path) noexcept
{
  CreateOptions opts;
  opts.format = Format::Auto;
  opts.layout = Layout::Auto;

  // FILE-compressed sources cannot be preserved by an in-place rewrite:
  // bagwiz's writers emit uncompressed bags, so pinning the storage format
  // would silently replace a `.db3.zstd` envelope with a plain `.db3`. Return
  // Auto/Auto so the caller surfaces a "could not detect storage format"
  // error and asks the user to pass an explicit `-o` output instead.
  if (is_file_compressed_bag(reference_path)) {
    return opts;
  }

  const auto detected = detect_format(reference_path);
  if (detected == Format::Auto) {
    return opts;
  }

  std::error_code ec;
  const bool is_dir = std::filesystem::is_directory(reference_path, ec);
  if (ec) {
    // Surface stat failure as detection failure so the caller errors
    // out rather than silently mis-pinning the layout.
    return opts;
  }

  opts.format = detected;
  opts.layout = is_dir ? Layout::Directory : Layout::SingleFile;
  return opts;
}

}  // namespace bagwiz::io
