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
#include <unordered_map>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{
constexpr const char * kLogger = "bagwiz.cmd.topic";

// Minimum widths so the header never looks cramped for short topic lists.
// Actual widths are computed from the data so long topic / type names do
// not push later columns out of alignment.
constexpr int kMinNameWidth = 4;   // "NAME"
constexpr int kMinTypeWidth = 4;   // "TYPE"
constexpr int kMinCountWidth = 5;  // "COUNT"
constexpr int kMinFreqWidth = 8;   // "FREQ(Hz)"

struct AggregatedTopic
{
  std::string type;
  std::string serialization_format;
  int64_t count = 0;
};

}  // namespace

// `bagwiz topic ls <input>...` accepts one or many bag paths. With multiple
// inputs the output is the union of all topics across every bag, with
// counts summed and frequency computed against the total observed duration
// (max-end minus min-start across all bags).
class TopicCommand : public Command
{
public:
  std::string_view name() const override { return "topic"; }
  std::string_view description() const override { return "Inspect topics in rosbags"; }

  void configure(CLI::App & app) override
  {
    app.require_subcommand(1);

    auto * ls =
      app.add_subcommand("ls", "List topics (union + summed counts when multiple bags are given)");
    ls->add_option("inputs", input_paths_, "One or more bag paths (file or directory)")
      ->required()
      ->expected(-1)
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
    std::unordered_map<std::string, AggregatedTopic> aggregated;
    int64_t earliest_ns = 0;
    int64_t latest_ns = 0;
    bool any_stats = false;
    int failures = 0;

    for (const auto & path : input_paths_) {
      std::unique_ptr<io::BagReader> reader;
      try {
        reader = io::open_read(path);
      } catch (const std::exception & e) {
        BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", path.c_str(), e.what());
        ++failures;
        continue;
      }

      const auto stats = reader->compute_stats();
      if (stats.total_messages > 0) {
        if (!any_stats || stats.start_ns < earliest_ns) {
          earliest_ns = stats.start_ns;
        }
        if (!any_stats || stats.end_ns > latest_ns) {
          latest_ns = stats.end_ns;
        }
        any_stats = true;
      }

      for (const auto & t : reader->topics()) {
        auto & agg = aggregated[t.name];
        if (agg.type.empty()) {
          agg.type = t.type;
          agg.serialization_format = t.serialization_format;
        } else if (agg.type != t.type) {
          BAGWIZ_LOG_WARN(
            kLogger, "topic %s has conflicting types across bags: %s vs %s", t.name.c_str(),
            agg.type.c_str(), t.type.c_str());
        }
        if (auto it = stats.per_topic.find(t.name); it != stats.per_topic.end()) {
          agg.count += it->second;
        }
      }
    }

    const double duration_sec = any_stats && latest_ns > earliest_ns
                                  ? static_cast<double>(latest_ns - earliest_ns) / 1e9
                                  : 0.0;

    // Sort topics by name for stable output that pipelines can diff.
    std::vector<std::pair<std::string, AggregatedTopic>> rows(aggregated.begin(), aggregated.end());
    std::sort(
      rows.begin(), rows.end(), [](const auto & a, const auto & b) { return a.first < b.first; });

    // Pre-compute column widths from the data so long topic / type names do
    // not push later columns out of alignment.
    int name_w = kMinNameWidth;
    int type_w = kMinTypeWidth;
    int count_w = kMinCountWidth;
    int freq_w = kMinFreqWidth;
    for (const auto & [name, agg] : rows) {
      name_w = std::max(name_w, static_cast<int>(name.size()));
      type_w = std::max(type_w, static_cast<int>(agg.type.size()));
      count_w = std::max(count_w, static_cast<int>(fmt::format("{}", agg.count).size()));
      const double freq = (duration_sec > 0.0 && agg.count > 1)
                            ? static_cast<double>(agg.count - 1) / duration_sec
                            : 0.0;
      freq_w = std::max(freq_w, static_cast<int>(fmt::format("{:.2f}", freq).size()));
    }

    fmt::print(
      stdout, "{:<{}} {:<{}} {:>{}} {:>{}}\n", "NAME", name_w, "TYPE", type_w, "COUNT", count_w,
      "FREQ(Hz)", freq_w);
    for (const auto & [name, agg] : rows) {
      const double freq = (duration_sec > 0.0 && agg.count > 1)
                            ? static_cast<double>(agg.count - 1) / duration_sec
                            : 0.0;
      fmt::print(
        stdout, "{:<{}} {:<{}} {:>{}} {:>{}.2f}\n", name, name_w, agg.type, type_w, agg.count,
        count_w, freq, freq_w);
    }
    return failures == 0 ? 0 : 1;
  }

  std::vector<std::filesystem::path> input_paths_;
  Op selected_op_ = Op::None;
};

BAGWIZ_REGISTER_COMMAND(TopicCommand)

}  // namespace bagwiz::commands
