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

#include <fmt/chrono.h>
#include <fmt/core.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace bagcli::commands
{

namespace
{
constexpr const char * kLogger = "bagcli.cmd.info";

std::uintmax_t bag_size_bytes(const std::filesystem::path & path)
{
  std::error_code ec;
  if (std::filesystem::is_directory(path, ec)) {
    std::uintmax_t total = 0;
    for (const auto & entry : std::filesystem::recursive_directory_iterator(path, ec)) {
      if (entry.is_regular_file(ec)) {
        total += entry.file_size(ec);
      }
    }
    return total;
  }
  return std::filesystem::file_size(path, ec);
}

std::string format_size(std::uintmax_t bytes)
{
  constexpr double kKi = 1024.0;
  constexpr double kMi = kKi * 1024.0;
  constexpr double kGi = kMi * 1024.0;
  const auto b = static_cast<double>(bytes);
  if (b >= kGi) {
    return fmt::format("{:.2f} GiB", b / kGi);
  }
  if (b >= kMi) {
    return fmt::format("{:.2f} MiB", b / kMi);
  }
  if (b >= kKi) {
    return fmt::format("{:.2f} KiB", b / kKi);
  }
  return fmt::format("{} B", bytes);
}

std::string format_time(int64_t ns)
{
  // Convert to a human-readable UTC timestamp; also print the raw seconds
  // so scripts that parse the output can skip the formatted form.
  const std::chrono::system_clock::time_point tp{std::chrono::nanoseconds(ns)};
  const auto secs_part = ns / 1'000'000'000;
  const auto frac_part = std::abs(ns % 1'000'000'000);
  return fmt::format(
    "{}.{:09d} ({:%Y-%m-%d %H:%M:%S} UTC)", secs_part, static_cast<int>(frac_part), tp);
}

}  // namespace

// `bagcli info <input>...` prints a human-readable summary of each bag.
// Multiple inputs are processed sequentially; output is separated by a
// blank line so per-bag sections stay visually distinct.
class InfoCommand : public Command
{
public:
  std::string_view name() const override { return "info"; }
  std::string_view description() const override { return "Print summary information for bags"; }

  void configure(CLI::App & app) override
  {
    app.add_option("inputs", input_paths_, "One or more bag paths (file or directory)")
      ->required()
      ->expected(-1)
      ->check(CLI::ExistingPath);
  }

  int run() override
  {
    int failures = 0;
    for (std::size_t i = 0; i < input_paths_.size(); ++i) {
      if (i > 0) {
        fmt::print(stdout, "\n");
      }
      if (!print_one(input_paths_[i])) {
        ++failures;
      }
    }
    return failures == 0 ? 0 : 1;
  }

private:
  bool print_one(const std::filesystem::path & path)
  {
    std::unique_ptr<io::BagReader> reader;
    try {
      reader = io::open_read(path);
    } catch (const std::exception & e) {
      BAGCLI_LOG_ERROR(kLogger, "Failed to open %s: %s", path.c_str(), e.what());
      return false;
    }

    const auto stats = reader->compute_stats();
    const auto size = bag_size_bytes(path);
    const double duration_sec = stats.total_messages > 0 && stats.end_ns >= stats.start_ns
                                  ? static_cast<double>(stats.end_ns - stats.start_ns) / 1e9
                                  : 0.0;

    fmt::print(stdout, "Path:              {}\n", path.string());
    fmt::print(stdout, "Size:              {}\n", format_size(size));
    fmt::print(
      stdout, "Layout:            {}\n",
      std::filesystem::is_directory(path) ? "directory" : "file");
    fmt::print(stdout, "Duration:          {:.3f} s\n", duration_sec);
    if (stats.total_messages > 0) {
      fmt::print(stdout, "Start:             {}\n", format_time(stats.start_ns));
      fmt::print(stdout, "End:               {}\n", format_time(stats.end_ns));
    }
    fmt::print(stdout, "Messages:          {}\n", stats.total_messages);
    fmt::print(stdout, "Stats source:      {}\n", stats.from_summary ? "summary (O(1))" : "scan");

    fmt::print(stdout, "Topics ({}):\n", reader->topics().size());
    std::vector<io::TopicInfo> sorted(reader->topics().begin(), reader->topics().end());
    std::sort(
      sorted.begin(), sorted.end(), [](const auto & a, const auto & b) { return a.name < b.name; });
    for (const auto & t : sorted) {
      int64_t count = 0;
      if (auto it = stats.per_topic.find(t.name); it != stats.per_topic.end()) {
        count = it->second;
      }
      const double freq =
        (duration_sec > 0.0 && count > 1) ? static_cast<double>(count - 1) / duration_sec : 0.0;
      fmt::print(
        stdout, "  {} | type: {} | count: {} | serialization: {} | freq: {:.2f} Hz\n", t.name,
        t.type, count, t.serialization_format, freq);
    }
    return true;
  }

  std::vector<std::filesystem::path> input_paths_;
};

BAGCLI_REGISTER_COMMAND(InfoCommand)

}  // namespace bagcli::commands
