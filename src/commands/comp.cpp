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

#include <cinttypes>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <memory>
#include <string>
#include <string_view>
#include <system_error>

namespace bagcli::commands
{

namespace
{
constexpr const char * kLogger = "bagcli.cmd.comp";

constexpr uint32_t kDefaultChunkSize = 4 * 1024 * 1024;
}  // namespace

// `bagcli comp <input> <output>` re-encodes a rosbag with the chosen
// compression (zstd by default). With `--inplace` the rewrite is atomic: a
// sibling temp file is written first and rename(2)'d over the input on
// success. Directory bags are not supported for --inplace in the first cut
// because atomic directory swap is fiddly; the user can pass an explicit
// output path instead.
class CompCommand : public Command
{
public:
  std::string_view name() const override { return "comp"; }
  std::string_view description() const override
  {
    return "Re-encode a rosbag with the chosen compression";
  }

  void configure(CLI::App & app) override
  {
    app.add_option("input", input_path_, "Input bag (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);

    auto * output_opt = app.add_option("output", output_path_, "Output bag path");
    auto * inplace_flag =
      app.add_flag("--inplace", inplace_, "Overwrite the input atomically; no output arg");
    output_opt->excludes(inplace_flag);
    inplace_flag->excludes(output_opt);

    app.add_option("--compression", compression_, "MCAP compression: none | lz4 | zstd")
      ->default_val("zstd")
      ->check(CLI::IsMember({"none", "lz4", "zstd"}));
    app.add_option("--chunk-size", chunk_size_, "MCAP chunk size in bytes")
      ->default_val(kDefaultChunkSize);
  }

  int run() override
  {
    if (!inplace_ && output_path_.empty()) {
      BAGCLI_LOG_ERROR(kLogger, "either <output> or --inplace must be specified");
      return 1;
    }

    std::filesystem::path write_path;
    std::filesystem::path tmp_path;
    if (inplace_) {
      std::error_code ec;
      if (std::filesystem::is_directory(input_path_, ec)) {
        BAGCLI_LOG_ERROR(kLogger, "--inplace is not supported for directory bags yet");
        return 1;
      }
      tmp_path = input_path_.parent_path() /
                 (input_path_.stem().string() + ".bagcli-tmp" + input_path_.extension().string());
      write_path = tmp_path;
    } else {
      write_path = output_path_;
    }

    io::CreateOptions options;
    options.mcap_compression = compression_;
    options.mcap_chunk_size = chunk_size_;

    std::unique_ptr<io::BagReader> reader;
    std::unique_ptr<io::BagWriter> writer;
    try {
      reader = io::open_read(input_path_);
      writer = io::open_write(write_path, options);
    } catch (const std::exception & e) {
      BAGCLI_LOG_ERROR(kLogger, "Failed to open bags: %s", e.what());
      remove_tmp_if_any(tmp_path);
      return 1;
    }

    for (const auto & topic : reader->topics()) {
      writer->declare_topic(topic);
    }

    int64_t count = 0;
    try {
      io::RawMessage msg;
      while (reader->next(msg)) {
        writer->write(msg.topic->name, msg.timestamp_ns, msg.payload);
        ++count;
      }
      writer->close();
    } catch (const std::exception & e) {
      BAGCLI_LOG_ERROR(
        kLogger, "Compression failed after %" PRId64 " messages: %s", count, e.what());
      remove_tmp_if_any(tmp_path);
      return 1;
    }

    if (inplace_) {
      std::error_code ec;
      std::filesystem::rename(tmp_path, input_path_, ec);
      if (ec) {
        BAGCLI_LOG_ERROR(
          kLogger, "rename %s -> %s failed: %s", tmp_path.c_str(), input_path_.c_str(),
          ec.message().c_str());
        remove_tmp_if_any(tmp_path);
        return 1;
      }
    }

    BAGCLI_LOG_INFO(
      kLogger, "Compressed %" PRId64 " messages with %s into %s", count, compression_.c_str(),
      inplace_ ? input_path_.c_str() : output_path_.c_str());
    return 0;
  }

private:
  static void remove_tmp_if_any(const std::filesystem::path & tmp)
  {
    if (tmp.empty()) {
      return;
    }
    std::error_code ec;
    std::filesystem::remove_all(tmp, ec);
  }

  std::filesystem::path input_path_;
  std::filesystem::path output_path_;
  bool inplace_ = false;
  std::string compression_;
  uint32_t chunk_size_ = kDefaultChunkSize;
};

BAGCLI_REGISTER_COMMAND(CompCommand)

}  // namespace bagcli::commands
