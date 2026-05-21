// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/core/logging.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <fmt/core.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{
constexpr const char * kLogger = "bagwiz.cmd.ls";

// Minimum widths so the header never looks cramped for short topic lists.
// Actual widths are computed from the data so long topic / type names do
// not push later columns out of alignment.
constexpr int kMinTopicWidth = 5;  // "TOPIC"
constexpr int kMinTypeWidth = 4;   // "TYPE"
constexpr int kMinCountWidth = 5;  // "COUNT"
constexpr int kMinFreqWidth = 2;   // "HZ"

struct Row
{
  std::string topic;
  std::string type;
  int64_t count = 0;
};

}  // namespace

// `bagwiz ls <input>` lists every topic in a single rosbag. Filtering is
// intentionally left to downstream tools (grep, awk, …) on the tabular
// output rather than being baked into the command itself.
class LsCommand : public Command
{
public:
  std::string_view name() const override { return "ls"; }
  std::string_view description() const override { return "List topics in a rosbag"; }

  void configure(CLI::App & app) override
  {
    app.add_option("input", input_path_, "Bag path (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
  }

  int run() override
  {
    std::unique_ptr<io::BagReader> reader;
    try {
      reader = io::open_read(input_path_);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", input_path_.c_str(), e.what());
      return 1;
    }

    const auto stats = reader->compute_stats();
    const double duration_sec = stats.total_messages > 0 && stats.end_ns > stats.start_ns
                                  ? static_cast<double>(stats.end_ns - stats.start_ns) / 1e9
                                  : 0.0;

    std::vector<Row> rows;
    for (const auto & t : reader->topics()) {
      int64_t count = 0;
      if (auto it = stats.per_topic.find(t.name); it != stats.per_topic.end()) {
        count = it->second;
      }
      rows.push_back({t.name, t.type, count});
    }

    // Sort by topic name for stable output that pipelines can diff.
    std::sort(
      rows.begin(), rows.end(), [](const auto & a, const auto & b) { return a.topic < b.topic; });

    // Pre-compute column widths from the data so long topic / type names do
    // not push later columns out of alignment.
    int topic_w = kMinTopicWidth;
    int type_w = kMinTypeWidth;
    int count_w = kMinCountWidth;
    int freq_w = kMinFreqWidth;
    for (const auto & row : rows) {
      topic_w = std::max(topic_w, static_cast<int>(row.topic.size()));
      type_w = std::max(type_w, static_cast<int>(row.type.size()));
      count_w = std::max(count_w, static_cast<int>(fmt::format("{}", row.count).size()));
      const double freq = (duration_sec > 0.0 && row.count > 1)
                            ? static_cast<double>(row.count - 1) / duration_sec
                            : 0.0;
      freq_w = std::max(freq_w, static_cast<int>(fmt::format("{:.2f}", freq).size()));
    }

    fmt::print(
      stdout, "{:<{}} {:<{}} {:>{}} {:>{}}\n", "TOPIC", topic_w, "TYPE", type_w, "COUNT", count_w,
      "HZ", freq_w);
    for (const auto & row : rows) {
      const double freq = (duration_sec > 0.0 && row.count > 1)
                            ? static_cast<double>(row.count - 1) / duration_sec
                            : 0.0;
      fmt::print(
        stdout, "{:<{}} {:<{}} {:>{}} {:>{}.2f}\n", row.topic, topic_w, row.type, type_w, row.count,
        count_w, freq, freq_w);
    }
    return 0;
  }

private:
  std::filesystem::path input_path_;
};

BAGWIZ_REGISTER_COMMAND(LsCommand)

}  // namespace bagwiz::commands
