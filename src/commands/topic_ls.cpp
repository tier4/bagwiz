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

#include <fmt/core.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace bagcli::commands
{

namespace
{
constexpr const char * kLogger = "bagcli.cmd.topic";

// Column widths for the text table. Names/types rarely exceed these; if they
// do, fmt wraps the field rather than truncating.
constexpr int kNameWidth = 40;
constexpr int kTypeWidth = 40;
constexpr int kCountWidth = 10;
constexpr int kFreqWidth = 10;

}  // namespace

class TopicCommand : public Command
{
public:
  std::string_view name() const override { return "topic"; }
  std::string_view description() const override { return "Inspect topics in a rosbag"; }

  void configure(CLI::App & app) override
  {
    app.require_subcommand(1);

    auto * ls = app.add_subcommand("ls", "List topics with type, message count, and frequency");
    ls->add_option("input", input_path_, "Bag file (.mcap/.db3) or directory with metadata.yaml")
      ->required()
      ->check(CLI::ExistingPath);
    ls->callback([this]() { selected_op_ = Op::Ls; });
  }

  int run() override
  {
    switch (selected_op_) {
      case Op::Ls:
        return run_ls();
      case Op::None:
        return 1;
    }
    return 1;
  }

private:
  enum class Op { None, Ls };

  int run_ls()
  {
    std::unique_ptr<io::BagReader> reader;
    try {
      reader = io::open_read(input_path_);
    } catch (const std::exception & e) {
      BAGCLI_LOG_ERROR(kLogger, "Failed to open %s: %s", input_path_.c_str(), e.what());
      return 1;
    }

    auto stats = reader->compute_stats();

    // Duration in seconds; protected against empty bags and single-message
    // bags where end - start == 0.
    double duration_sec = 0.0;
    if (stats.end_ns > stats.start_ns) {
      duration_sec = static_cast<double>(stats.end_ns - stats.start_ns) / 1e9;
    }

    // Sort topics by name for stable output that pipelines can diff.
    std::vector<io::TopicInfo> sorted(reader->topics().begin(), reader->topics().end());
    std::sort(
      sorted.begin(), sorted.end(), [](const auto & a, const auto & b) { return a.name < b.name; });

    fmt::print(
      stdout, "{:<{}} {:<{}} {:>{}} {:>{}}\n", "NAME", kNameWidth, "TYPE", kTypeWidth, "COUNT",
      kCountWidth, "FREQ(Hz)", kFreqWidth);

    for (const auto & t : sorted) {
      int64_t count = 0;
      if (auto it = stats.per_topic.find(t.name); it != stats.per_topic.end()) {
        count = it->second;
      }
      // n-1 intervals between n messages; for single-message topics the
      // frequency is undefined (report 0.00).
      const double freq =
        (duration_sec > 0.0 && count > 1) ? static_cast<double>(count - 1) / duration_sec : 0.0;
      fmt::print(
        stdout, "{:<{}} {:<{}} {:>{}} {:>{}.2f}\n", t.name, kNameWidth, t.type, kTypeWidth, count,
        kCountWidth, freq, kFreqWidth);
    }
    return 0;
  }

  std::filesystem::path input_path_;
  Op selected_op_ = Op::None;
};

BAGCLI_REGISTER_COMMAND(TopicCommand)

}  // namespace bagcli::commands
