// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagcli/commands/command.hpp"
#include "bagcli/core/logging.hpp"
#include "bagcli/io/bag_io.hpp"

#include <algorithm>
#include <cinttypes>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

namespace bagcli::commands
{

namespace
{
constexpr const char * kLogger = "bagcli.cmd.filter";
constexpr uint32_t kDefaultChunkSize = 4 * 1024 * 1024;
}  // namespace

// `bagcli filter <input> <output> [--topics ...] [--start ...] [--end ...]`
// writes a subset of `input` into `output`. Both filters are pushed into
// the BagReader so the underlying storage layer can skip data it doesn't
// need to decode (chunk index for MCAP, WHERE clauses for SQLite3).
class FilterCommand : public Command
{
public:
  std::string_view name() const override { return "filter"; }
  std::string_view description() const override
  {
    return "Write a subset of a bag (topic / time range) to a new bag";
  }

  void configure(CLI::App & app) override
  {
    app.add_option("input", input_path_, "Input bag (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    app.add_option("output", output_path_, "Output bag path")->required();

    app.add_option("--topics", topics_, "Keep only these topics (space-separated)")->expected(-1);
    app.add_option(
      "--start-ns", start_ns_, "Drop messages with timestamp < start (ns since epoch)");
    app.add_option("--end-ns", end_ns_, "Drop messages with timestamp > end (ns since epoch)");

    app.add_option("--compression", compression_, "Output MCAP compression: none | lz4 | zstd")
      ->default_val("zstd")
      ->check(CLI::IsMember({"none", "lz4", "zstd"}));
    app.add_option("--chunk-size", chunk_size_, "MCAP chunk size in bytes")
      ->default_val(kDefaultChunkSize);
  }

  int run() override
  {
    io::CreateOptions options;
    options.mcap_compression = compression_;
    options.mcap_chunk_size = chunk_size_;

    std::unique_ptr<io::BagReader> reader;
    std::unique_ptr<io::BagWriter> writer;
    try {
      reader = io::open_read(input_path_);
      writer = io::open_write(output_path_, options);
    } catch (const std::exception & e) {
      BAGCLI_LOG_ERROR(kLogger, "Failed to open bags: %s", e.what());
      return 1;
    }

    io::ReadFilter filter;
    filter.topics = topics_;
    if (start_ns_ > 0) {
      filter.start_ns = start_ns_;
    }
    if (end_ns_ > 0) {
      filter.end_ns = end_ns_;
    }
    reader->set_filter(filter);

    // Only declare topics the user actually kept. When --topics is empty we
    // keep everything.
    const std::unordered_set<std::string> keep(topics_.begin(), topics_.end());
    for (const auto & t : reader->topics()) {
      if (keep.empty() || keep.contains(t.name)) {
        writer->declare_topic(t);
      }
    }

    int64_t kept = 0;
    try {
      io::RawMessage msg;
      while (reader->next(msg)) {
        writer->write(msg.topic->name, msg.timestamp_ns, msg.payload);
        ++kept;
      }
      writer->close();
    } catch (const std::exception & e) {
      BAGCLI_LOG_ERROR(kLogger, "Filter failed after %" PRId64 " messages: %s", kept, e.what());
      return 1;
    }

    BAGCLI_LOG_INFO(kLogger, "Wrote %" PRId64 " messages to %s", kept, output_path_.c_str());
    return 0;
  }

private:
  std::filesystem::path input_path_;
  std::filesystem::path output_path_;
  std::vector<std::string> topics_;
  int64_t start_ns_ = 0;
  int64_t end_ns_ = 0;
  std::string compression_;
  uint32_t chunk_size_ = kDefaultChunkSize;
};

BAGCLI_REGISTER_COMMAND(FilterCommand)

}  // namespace bagcli::commands
