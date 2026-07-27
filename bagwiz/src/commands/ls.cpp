// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/bag_open.hpp"

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

// Average publish rate over the whole bag's time span. Topics with a single
// message (or a zero-duration bag) have no meaningful rate and report 0.
double topic_hz(int64_t count, double duration_sec)
{
  return (duration_sec > 0.0 && count > 1) ? static_cast<double>(count - 1) / duration_sec : 0.0;
}

// Short listing: just the names and types, both available from the cheap
// topics() path. No message scan, so this stays O(1) regardless of bag size
// or storage format.
void print_short(const std::vector<Row> & rows)
{
  int topic_w = kMinTopicWidth;
  int type_w = kMinTypeWidth;
  for (const auto & row : rows) {
    topic_w = std::max(topic_w, static_cast<int>(row.topic.size()));
    type_w = std::max(type_w, static_cast<int>(row.type.size()));
  }

  fmt::print(stdout, "{:<{}} {:<{}}\n", "TOPIC", topic_w, "TYPE", type_w);
  for (const auto & row : rows) {
    fmt::print(stdout, "{:<{}} {:<{}}\n", row.topic, topic_w, row.type, type_w);
  }
}

// Long listing: adds per-topic COUNT and average HZ, which require the stats
// scan the caller opted into via `-l`.
void print_long(const std::vector<Row> & rows, double duration_sec)
{
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
    freq_w = std::max(
      freq_w, static_cast<int>(fmt::format("{:.2f}", topic_hz(row.count, duration_sec)).size()));
  }

  fmt::print(
    stdout, "{:<{}} {:<{}} {:>{}} {:>{}}\n", "TOPIC", topic_w, "TYPE", type_w, "COUNT", count_w,
    "HZ", freq_w);
  for (const auto & row : rows) {
    fmt::print(
      stdout, "{:<{}} {:<{}} {:>{}} {:>{}.2f}\n", row.topic, topic_w, row.type, type_w, row.count,
      count_w, topic_hz(row.count, duration_sec), freq_w);
  }
}

}  // namespace

// `bagwiz ls <input>` lists every topic in a single rosbag. Filtering is
// intentionally left to downstream tools (grep, awk, …) on the tabular
// output rather than being baked into the command itself.
//
// Plain `ls` prints only topic names and types, sourced from the cheap
// topics() path. `ls -l` adds per-topic message counts and average Hz, which
// require compute_stats(). A single-file `.db3` answers those from its
// `metadata` table when it has one — every bagwiz-written bag, and any
// recorded by rosbag2 iron or newer. Older recordings (humble) leave that
// table empty and force a full scan of the messages table, so `-l` stays
// gated behind the explicit flag.
class LsCommand : public Command
{
public:
  [[nodiscard]] std::string_view name() const override { return "ls"; }
  [[nodiscard]] std::string_view description() const override { return "List topics in a rosbag"; }

  void configure(CLI::App & app) override
  {
    app.add_option("input", input_path_, "Bag path (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    app.add_flag(
      "-l,--long", long_listing_,
      "Show per-topic message COUNT and average HZ (scans the bag; slow on "
      "single-file SQLite bags)");
  }

  int run() override
  {
    auto reader = io::open_read_or_log(input_path_, kLogger);
    if (!reader) {
      return 1;
    }

    std::vector<Row> rows;
    for (const auto & t : reader->topics()) {
      rows.push_back({t.name, t.type, 0});
    }

    double duration_sec = 0.0;
    if (long_listing_) {
      const auto stats = reader->compute_stats();
      duration_sec = stats.total_messages > 0 && stats.end_ns > stats.start_ns
                       ? static_cast<double>(stats.end_ns - stats.start_ns) / 1e9
                       : 0.0;
      for (auto & row : rows) {
        if (auto it = stats.per_topic.find(row.topic); it != stats.per_topic.end()) {
          row.count = it->second;
        }
      }
    }

    // Sort by topic name for stable output that pipelines can diff.
    std::sort(
      rows.begin(), rows.end(), [](const auto & a, const auto & b) { return a.topic < b.topic; });

    if (long_listing_) {
      print_long(rows, duration_sec);
    } else {
      print_short(rows);
    }
    return 0;
  }

private:
  std::filesystem::path input_path_;
  bool long_listing_ = false;
};

BAGWIZ_REGISTER_COMMAND(LsCommand)

}  // namespace bagwiz::commands
