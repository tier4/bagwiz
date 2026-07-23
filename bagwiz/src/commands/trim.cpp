// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/trim.hpp"

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/core/bag/bag_copy.hpp"
#include "bagwiz/core/bag/rewrite.hpp"
#include "bagwiz/core/base/duration_parse.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/base/str_utils.hpp"
#include "bagwiz/core/base/topic_match.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/bag_open.hpp"
#include "bagwiz/io/topics.hpp"
#include "trim_stamp.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <algorithm>
#include <charconv>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kLogger = "bagwiz.cmd.trim";

// Parse one --start/--end/--duration value; the unit is mandatory. On failure
// logs a targeted error — distinguishing a bare number (missing unit) from
// unparseable text — and returns nullopt.
std::optional<std::int64_t> parse_offset_or_log(const std::string & value, const char * flag)
{
  const auto ns = core::parse_duration_ns(value, core::DurationUnitPolicy::RequireUnit);
  if (ns.has_value()) {
    return ns;
  }
  if (core::parse_duration_ns(value).has_value()) {
    BAGWIZ_LOG_ERROR(
      kLogger, "trim: %s '%s' has no unit; append one of ns, us, ms, s (e.g. '5s')", flag,
      value.c_str());
  } else {
    BAGWIZ_LOG_ERROR(
      kLogger, "trim: could not parse %s '%s' (expected <number><unit>, e.g. 5s, 500ms, 1.5s)",
      flag, value.c_str());
  }
  return std::nullopt;
}

std::string_view trim_ws_view(std::string_view s)
{
  const auto first = s.find_first_not_of(" \t");
  if (first == std::string_view::npos) {
    return {};
  }
  const auto last = s.find_last_not_of(" \t");
  return s.substr(first, last - first + 1);
}

// True when the value carries the `msg` unit (a message count).
// cppcheck-suppress passedByValue  // std::string_view is a cheap value type
bool has_msg_unit(std::string_view text)
{
  return trim_ws_view(text).ends_with("msg");
}

// A --start/--end/--both value: either a time offset from the bag start or a
// message count (the `msg` unit).
struct WindowBound
{
  bool is_message_count = false;
  std::int64_t value = 0;  // ns offset, or a message count
};

// Parse one --start/--end/--both value: a time offset with a mandatory unit
// (5s, 500ms) or a message count (100msg). Logs a targeted error and returns
// nullopt on failure.
std::optional<WindowBound> parse_bound_or_log(const std::string & value, const char * flag)
{
  const std::string_view text = trim_ws_view(value);
  if (text.ends_with("msg")) {
    const std::string_view digits = trim_ws_view(text.substr(0, text.size() - 3));
    std::int64_t count = 0;
    // Digits only: from_chars alone would accept a leading '-' into the
    // signed count, and a fraction must not silently truncate.
    bool ok = !digits.empty() && std::all_of(digits.begin(), digits.end(), [](char c) {
      return c >= '0' && c <= '9';
    });
    if (ok) {
      const auto [ptr, ec] = std::from_chars(digits.data(), digits.data() + digits.size(), count);
      ok = ec == std::errc() && ptr == digits.data() + digits.size();
    }
    if (!ok) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "trim: %s '%s' is not a valid message count (expected a whole number >= 0, e.g. 100msg)",
        flag, value.c_str());
      return std::nullopt;
    }
    return WindowBound{true, count};
  }
  const auto ns = core::parse_duration_ns(value, core::DurationUnitPolicy::RequireUnit);
  if (ns.has_value()) {
    return WindowBound{false, *ns};
  }
  if (core::parse_duration_ns(value).has_value()) {
    BAGWIZ_LOG_ERROR(
      kLogger, "trim: %s '%s' has no unit; append one of ns, us, ms, s, msg (e.g. '5s', '100msg')",
      flag, value.c_str());
  } else {
    BAGWIZ_LOG_ERROR(
      kLogger, "trim: could not parse %s '%s' (expected <number><unit>, e.g. 5s, 500ms, 100msg)",
      flag, value.c_str());
  }
  return std::nullopt;
}

// The reference clock of one message. Under --stamp header a message on a
// headered topic uses its header.stamp; everything else (headerless type,
// stamp left at 0, --stamp recv) uses the record time — the same fallback rule
// the point-cloud fetcher applies.
std::int64_t message_clock_ns(
  const io::RawMessage & raw, bool use_header_stamp,
  const std::unordered_set<std::string> & headered)
{
  if (use_header_stamp && headered.count(raw.topic->name) != 0) {
    const auto stamp = read_leading_header_stamp_ns(raw.payload);
    if (stamp.has_value() && *stamp > 0) {
      return *stamp;
    }
  }
  return raw.timestamp_ns;
}

// The topics whose payloads a clock scan must materialize: under --stamp
// header only headered topics carry the stamp in their payload. An allow-list
// that names no topic ({""}) marks every row timestamps-only (see
// ReadFilter::payload_topics).
std::vector<std::string> clock_scan_payload_topics(
  bool use_header_stamp, const std::unordered_set<std::string> & headered,
  const std::unordered_set<std::string> & scanned)
{
  std::vector<std::string> payload_topics;
  if (use_header_stamp) {
    for (const auto & name : scanned) {
      if (headered.count(name) != 0) {
        payload_topics.push_back(name);
      }
    }
  }
  if (payload_topics.empty()) {
    payload_topics.emplace_back("");
  }
  return payload_topics;
}

// Full-scan clock extent of the bag under --stamp header: the min/max
// message_clock_ns over every message. The receive-time twin is the cheap
// BagReader::compute_time_extent(); header stamps live in payloads, so no
// summary can answer this. `reader` must be fresh.
io::BagReader::TimeExtent compute_clock_extent(
  io::BagReader & reader, const std::unordered_set<std::string> & headered)
{
  std::unordered_set<std::string> all_topics;
  for (const auto & t : reader.topics()) {
    all_topics.insert(t.name);
  }
  io::ReadFilter filter;
  filter.payload_topics = clock_scan_payload_topics(true, headered, all_topics);
  reader.set_filter(filter);

  io::BagReader::TimeExtent extent;
  extent.start_ns = std::numeric_limits<std::int64_t>::max();
  extent.end_ns = std::numeric_limits<std::int64_t>::min();
  io::RawMessage raw;
  while (reader.next(raw)) {
    const auto clock = message_clock_ns(raw, true, headered);
    extent.start_ns = std::min(extent.start_ns, clock);
    extent.end_ns = std::max(extent.end_ns, clock);
    extent.has_data = true;
  }
  if (!extent.has_data) {
    extent.start_ns = 0;
    extent.end_ns = 0;
  }
  return extent;
}

// Every message's clock value, ascending. Backs the `msg` (message-count)
// bounds: index N is the clock of the (N+1)-th message on the reference
// clock. Payloads materialize only where the clock needs them. `reader` must
// be fresh.
std::vector<std::int64_t> collect_sorted_clocks(
  io::BagReader & reader, bool use_header_stamp, const std::unordered_set<std::string> & headered)
{
  std::unordered_set<std::string> all_topics;
  for (const auto & t : reader.topics()) {
    all_topics.insert(t.name);
  }
  io::ReadFilter filter;
  filter.payload_topics = clock_scan_payload_topics(use_header_stamp, headered, all_topics);
  reader.set_filter(filter);

  std::vector<std::int64_t> clocks;
  io::RawMessage raw;
  while (reader.next(raw)) {
    clocks.push_back(message_clock_ns(raw, use_header_stamp, headered));
  }
  std::sort(clocks.begin(), clocks.end());
  return clocks;
}

// Resolve --align into an absolute window: expand the selectors against the
// bag's topic list, then scan the selected topics' clock values and take
// their common time span — from the latest first message to the earliest
// last message. Both boundary messages are inside the window, so the
// exclusive upper bound is last + 1. Payloads are materialized only where the
// clock needs them (header stamps); pure receive-time scans skip them
// entirely (see ReadFilter::payload_topics). `reader` must be fresh
// (set_filter precedes iteration). Logs and returns false on any failure.
bool resolve_align_window(
  const TrimArgs & args, io::BagReader & reader, bool use_header_stamp,
  const std::unordered_set<std::string> & headered, std::int64_t & abs_start_ns,
  std::optional<std::int64_t> & abs_end_ns)
{
  const auto topic_names = io::snapshot_topic_names(reader);
  const auto resolution = core::resolve_topic_patterns(args.align, topic_names);
  if (!resolution.unmatched.empty()) {
    for (const auto & pattern : resolution.unmatched) {
      BAGWIZ_LOG_ERROR(
        kLogger, "trim: --align selector '%s' matched no topic in %s", pattern.c_str(),
        args.input_path.c_str());
    }
    return false;
  }

  io::ReadFilter filter;
  filter.topics.assign(resolution.matched.begin(), resolution.matched.end());
  filter.payload_topics = clock_scan_payload_topics(use_header_stamp, headered, resolution.matched);
  reader.set_filter(filter);

  // First/last clock value per selected topic. Iteration order is not
  // assumed (an unindexed MCAP falls back to file order), so track min/max.
  std::unordered_map<std::string, std::pair<std::int64_t, std::int64_t>> spans;
  io::RawMessage raw;
  while (reader.next(raw)) {
    const auto clock = message_clock_ns(raw, use_header_stamp, headered);
    auto [it, inserted] = spans.try_emplace(raw.topic->name, clock, clock);
    if (!inserted) {
      it->second.first = std::min(it->second.first, clock);
      it->second.second = std::max(it->second.second, clock);
    }
  }

  std::int64_t start = std::numeric_limits<std::int64_t>::min();
  std::int64_t last = std::numeric_limits<std::int64_t>::max();
  for (const auto & name : resolution.matched) {
    const auto it = spans.find(name);
    if (it == spans.end()) {
      BAGWIZ_LOG_ERROR(kLogger, "trim: --align topic '%s' has no messages.", name.c_str());
      return false;
    }
    start = std::max(start, it->second.first);
    last = std::min(last, it->second.second);
  }
  if (start > last) {
    BAGWIZ_LOG_ERROR(
      kLogger, "trim: the %zu --align topic(s) do not overlap in time; the window is empty.",
      resolution.matched.size());
    return false;
  }

  abs_start_ns = start;
  if (last == std::numeric_limits<std::int64_t>::max()) {
    abs_end_ns = std::nullopt;  // last + 1 would overflow: unbounded end
  } else {
    abs_end_ns = last + 1;  // include the boundary message under [start, end)
  }
  BAGWIZ_LOG_INFO(
    kLogger, "trim: aligned to %zu topic(s); window %s .. %s (both bounds inclusive).",
    resolution.matched.size(), core::format_timestamp(start).c_str(),
    core::format_timestamp(last).c_str());
  return true;
}

// Declare every input topic verbatim, then stream-copy only the messages in
// the half-open window [abs_start_ns, abs_end_ns). Under --stamp recv the
// range rides in the reader's ReadFilter, so the storage layer skips
// out-of-range chunks/rows; under --stamp header the clock lives in the
// payloads, so the whole bag streams through a per-message keep predicate
// instead. Shared by the in-place and -o modes (core::run_bag_rewrite injects
// the writer factory). Returns a process exit code.
int execute_trim_pass(
  const TrimArgs & args, std::int64_t abs_start_ns, std::optional<std::int64_t> abs_end_ns,
  bool use_header_stamp, const std::unordered_set<std::string> & headered,
  const io::WriterFactory & open_writer)
{
  auto reader = io::open_read_or_log(args.input_path, kLogger);
  if (!reader) {
    return 1;
  }
  // Backfill embedded schemas so MCAP outputs keep self-description (no-op
  // for single-file readers and SQLite3 inputs).
  reader->populate_schemas();

  auto writer = io::open_write_or_log(open_writer, kLogger);
  if (!writer) {
    return 1;
  }

  // Trim removes messages, never topics: every topic is declared verbatim, so
  // a topic whose messages all fall outside the window stays declared and even
  // an empty window yields a structurally valid bag.
  std::size_t declared = 0;
  for (const auto & t : reader->topics()) {
    try {
      writer->declare_topic(t);
      ++declared;
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "declare_topic failed for '%s': %s", t.name.c_str(), e.what());
      return 1;
    }
  }

  core::MessagePredicate keep;
  if (use_header_stamp) {
    keep = [&](const io::RawMessage & raw) {
      const auto clock = message_clock_ns(raw, true, headered);
      return clock >= abs_start_ns && (!abs_end_ns || clock < *abs_end_ns);
    };
  } else {
    io::ReadFilter filter;
    filter.start_ns = abs_start_ns;
    filter.end_ns = abs_end_ns;
    reader->set_filter(filter);
  }

  core::BagCopyCounts counts;
  try {
    const std::unordered_set<std::string> none;
    counts = core::bag_copy_filtered(
      *reader, *writer, none, "trim", core::pipeline::BackendKind::Pipelined, keep);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Stream copy from %s failed: %s", args.input_path.c_str(), e.what());
    return 1;
  }

  if (!io::close_writer_or_log(*writer, kLogger)) {
    return 1;
  }

  if (counts.copied == 0) {
    BAGWIZ_LOG_WARN(kLogger, "trim: the window contains no messages; the output bag is empty.");
  }
  const std::string window_end =
    abs_end_ns ? core::format_timestamp(*abs_end_ns) : std::string("bag end");
  BAGWIZ_LOG_INFO(
    kLogger, "trim: copied %" PRIu64 " message(s) across %zu topic(s); window %s .. %s.",
    counts.copied, declared, core::format_timestamp(abs_start_ns).c_str(), window_end.c_str());
  return 0;
}

}  // namespace

int run_trim(const TrimArgs & args)
{
  // 0. Validate the window arguments before touching the filesystem. The CLI
  //    already enforces the flag exclusions (CLI11 excludes()) and the --stamp
  //    choices (CLI11 IsMember), but run_trim is also called directly from
  //    tests.
  const bool use_header_stamp = args.stamp == "header";
  if (!use_header_stamp && args.stamp != "recv") {
    BAGWIZ_LOG_ERROR(
      kLogger, "trim: unknown --stamp '%s'; expected 'header' or 'recv'.", args.stamp.c_str());
    return 1;
  }
  if (!args.align.empty() && (args.start || args.end || args.duration || args.both)) {
    BAGWIZ_LOG_ERROR(
      kLogger,
      "trim: --align is mutually exclusive with the offset flags "
      "(--start, --end, --duration, --both).");
    return 1;
  }
  if (args.both && (args.start || args.end || args.duration)) {
    BAGWIZ_LOG_ERROR(
      kLogger, "trim: --both is mutually exclusive with --start, --end, and --duration.");
    return 1;
  }
  if (args.end.has_value() && args.duration.has_value()) {
    BAGWIZ_LOG_ERROR(kLogger, "trim: --end and --duration are mutually exclusive.");
    return 1;
  }
  if (!args.start && !args.end && !args.duration && !args.both && args.align.empty()) {
    BAGWIZ_LOG_ERROR(
      kLogger,
      "trim: no window given; pass --start and/or --end or --duration, --both, or --align "
      "(for a full copy use `cp -r`).");
    return 1;
  }

  std::optional<WindowBound> start_b;
  if (args.start) {
    start_b = parse_bound_or_log(*args.start, "--start");
    if (!start_b) {
      return 1;
    }
  }
  std::optional<WindowBound> end_b;
  if (args.end) {
    end_b = parse_bound_or_log(*args.end, "--end");
    if (!end_b) {
      return 1;
    }
  }
  std::optional<std::int64_t> duration_ns;
  if (args.duration) {
    if (has_msg_unit(*args.duration)) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "trim: --duration takes a time length (e.g. 30s); the msg unit is only valid for "
        "--start, --end, and --both.");
      return 1;
    }
    const auto off = parse_offset_or_log(*args.duration, "--duration");
    if (!off) {
      return 1;
    }
    duration_ns = *off;
  }
  std::optional<WindowBound> both_b;
  if (args.both) {
    both_b = parse_bound_or_log(*args.both, "--both");
    if (!both_b) {
      return 1;
    }
  }
  // Message counts are validated by the parser (whole number >= 0); time
  // offsets can carry a sign the duration grammar allows but trim rejects.
  const auto is_negative_time = [](const std::optional<WindowBound> & b) {
    return b && !b->is_message_count && b->value < 0;
  };
  if (
    is_negative_time(start_b) || is_negative_time(end_b) || is_negative_time(both_b) ||
    (duration_ns && *duration_ns < 0)) {
    BAGWIZ_LOG_ERROR(kLogger, "trim: offsets are relative to the bag start and must be >= 0.");
    return 1;
  }
  if (both_b && both_b->value == 0) {
    BAGWIZ_LOG_ERROR(
      kLogger, "trim: --both '%s' trims nothing; pass a positive value.", args.both->c_str());
    return 1;
  }

  // 1. Resolve the window to absolute bounds up front so an out-of-range or
  //    unmatched window fails before any writer (or in-place tmp) is created.
  //    The reader is scoped: the bounds are snapshotted and the input released
  //    before the rewrite pass reopens it.
  std::int64_t abs_start_ns = 0;
  std::optional<std::int64_t> abs_end_ns;
  // Under --stamp header: the topics whose clock is their header.stamp,
  // classified once here and reused by the rewrite pass. Empty under recv.
  std::unordered_set<std::string> headered;
  {
    auto reader = io::open_read_or_log(args.input_path, kLogger);
    if (!reader) {
      return 1;
    }
    if (use_header_stamp) {
      // Embedded schemas feed the leading-Header classification; SQLite3
      // inputs fall back to $AMENT_PREFIX_PATH .msg resolution inside
      // classify_headered_topics.
      reader->populate_schemas();
      auto classified = classify_headered_topics(reader->topics());
      headered = std::move(classified.topics);
      if (!classified.unresolved_types.empty()) {
        BAGWIZ_LOG_WARN(
          kLogger,
          "trim: %zu message type(s) could not be classified for header.stamp (e.g. '%s'); "
          "their topics use receive time.",
          classified.unresolved_types.size(), classified.unresolved_types.front().c_str());
      }
    }
    if (!args.align.empty()) {
      if (!resolve_align_window(
            args, *reader, use_header_stamp, headered, abs_start_ns, abs_end_ns)) {
        return 1;
      }
      // fallthrough to the rewrite dispatch below
    } else {
      // Message-count bounds rank every message on the reference clock, so
      // they need the full sorted clock list; that scan also yields the
      // extent. Pure time bounds keep the cheap summary-based extent path.
      const auto is_msg = [](const std::optional<WindowBound> & b) {
        return b && b->is_message_count;
      };
      std::vector<std::int64_t> clocks;
      io::BagReader::TimeExtent extent;
      if (is_msg(start_b) || is_msg(end_b) || is_msg(both_b)) {
        clocks = collect_sorted_clocks(*reader, use_header_stamp, headered);
        extent.has_data = !clocks.empty();
        if (extent.has_data) {
          extent.start_ns = clocks.front();
          extent.end_ns = clocks.back();
        }
      } else {
        extent = use_header_stamp ? compute_clock_extent(*reader, headered)
                                  : reader->compute_time_extent();
      }
      if (!extent.has_data) {
        BAGWIZ_LOG_ERROR(
          kLogger,
          "trim: cannot resolve offsets relative to the bag start: %s has no time extent "
          "(empty bag or no summary index).",
          args.input_path.c_str());
        return 1;
      }
      const std::int64_t bag_duration_ns = extent.end_ns - extent.start_ns;
      const auto total = static_cast<std::int64_t>(clocks.size());

      if (both_b && both_b->is_message_count) {
        // --both Nmsg trims the first N and the last N messages.
        const std::int64_t n = both_b->value;
        if (n >= total || n >= total - n) {
          BAGWIZ_LOG_ERROR(
            kLogger, "trim: --both %s trims away the entire bag (%" PRId64 " message(s)).",
            args.both->c_str(), total);
          return 1;
        }
        abs_start_ns = clocks[static_cast<std::size_t>(n)];
        abs_end_ns = clocks[static_cast<std::size_t>(total - n)];
      } else if (both_b) {
        // --both X is shorthand for --start X --end (bag_duration - X): the
        // kept window shrinks symmetrically from both ends.
        const std::int64_t start_off = both_b->value;
        const std::int64_t end_off = bag_duration_ns - both_b->value;
        if (start_off >= end_off) {
          BAGWIZ_LOG_ERROR(
            kLogger, "trim: --both %s trims away the entire bag (bag duration %.3f s).",
            args.both->c_str(), static_cast<double>(bag_duration_ns) / 1e9);
          return 1;
        }
        abs_start_ns = extent.start_ns + start_off;
        abs_end_ns = extent.start_ns + end_off;
      } else {
        if (is_msg(start_b)) {
          // --start Nmsg skips the first N messages: the window starts at the
          // clock of the (N+1)-th.
          if (start_b->value >= total) {
            BAGWIZ_LOG_ERROR(
              kLogger, "trim: --start %s is past the end of the bag (%" PRId64 " message(s)).",
              args.start->c_str(), total);
            return 1;
          }
          abs_start_ns = clocks[static_cast<std::size_t>(start_b->value)];
        } else {
          const std::int64_t start_off = start_b ? start_b->value : 0;
          if (start_off > bag_duration_ns) {
            BAGWIZ_LOG_ERROR(
              kLogger, "trim: --start %s is past the end of the bag (bag duration %.3f s).",
              args.start ? args.start->c_str() : "?", static_cast<double>(bag_duration_ns) / 1e9);
            return 1;
          }
          // start_off <= bag_duration_ns keeps extent.start_ns + start_off
          // inside the bag's own timestamp range, so this cannot overflow.
          abs_start_ns = extent.start_ns + start_off;
        }
        if (is_msg(end_b)) {
          // --end Nmsg keeps (at most) the first N messages: the exclusive
          // bound is the clock of the (N+1)-th.
          if (end_b->value >= total) {
            BAGWIZ_LOG_WARN(
              kLogger,
              "trim: --end %s exceeds the bag's %" PRId64
              " message(s); the output ends at the "
              "bag end.",
              args.end->c_str(), total);
          } else {
            abs_end_ns = clocks[static_cast<std::size_t>(end_b->value)];
          }
        } else if (end_b) {
          if (end_b->value > bag_duration_ns) {
            BAGWIZ_LOG_WARN(
              kLogger,
              "trim: the requested window end is past the bag end (bag duration %.3f s); the "
              "output ends at the bag end.",
              static_cast<double>(bag_duration_ns) / 1e9);
          }
          if (end_b->value > std::numeric_limits<std::int64_t>::max() - extent.start_ns) {
            abs_end_ns = std::nullopt;  // absurdly large --end: same as omitting it
          } else {
            abs_end_ns = extent.start_ns + end_b->value;
          }
        } else if (duration_ns) {
          if (*duration_ns > std::numeric_limits<std::int64_t>::max() - abs_start_ns) {
            abs_end_ns = std::nullopt;  // start + duration overflows: unbounded end
          } else {
            abs_end_ns = abs_start_ns + *duration_ns;
          }
        }
      }
      if (abs_end_ns && abs_start_ns >= *abs_end_ns) {
        BAGWIZ_LOG_ERROR(
          kLogger, "trim: empty window; the start bound must be strictly before the end bound.");
        return 1;
      }
    }
  }

  // 2. -o vs in-place dispatch, shared with the other rewrite-style commands:
  //    -o writes a new bag and leaves <input> untouched; otherwise <input> is
  //    rewritten atomically via a sibling tmp, preserving its storage format
  //    and layout.
  core::BagRewriteOptions rewrite_opts;
  rewrite_opts.logger = kLogger;
  rewrite_opts.format_unknown_error = "trim: could not detect storage format of input bag '%s'.";
  rewrite_opts.pass_failed_error = "trim: pass failed; aborting in-place swap";
  rewrite_opts.inherit_output_format = true;
  return core::run_bag_rewrite(
    args.input_path, args.output_path, args.overwrite, rewrite_opts,
    [&](const io::WriterFactory & open_writer) {
      return execute_trim_pass(
        args, abs_start_ns, abs_end_ns, use_header_stamp, headered, open_writer);
    });
}

// `bagwiz trim` cuts a bag down to a time window given as offsets from the
// bag's start time.
class TrimCommand : public Command
{
public:
  [[nodiscard]] std::string_view name() const override { return "trim"; }
  [[nodiscard]] std::string_view description() const override
  {
    return "Trim a rosbag to a time window";
  }

  void configure(CLI::App & app) override
  {
    app.add_option("input", args_.input_path, "Input ROS 2 rosbag (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    auto * start_opt = app.add_option(
      "--start", args_.start,
      "Window start: an offset from the bag start (e.g. 5s, 500ms) or a message count "
      "(100msg skips the first 100). Unit required. Default: bag start.");
    auto * end_opt = app.add_option(
      "--end", args_.end,
      "Window end, exclusive: an offset from the bag start (e.g. 90s) or a message count "
      "(500msg keeps the first 500). Unit required. Default: bag end.");
    auto * dur_opt = app.add_option(
      "--duration", args_.duration,
      "Window length measured from the window start (e.g. 30s). Unit required; time only.");
    auto * both_opt = app.add_option(
      "--both", args_.both,
      "Trim this much from both the bag start and the bag end: a time offset (5s) or a "
      "message count (50msg). Unit required.");
    auto * align_opt = app.add_option(
      "--align", args_.align,
      "Trim to the common time span of these topics (latest first message to earliest last "
      "message, both included). Literal names or '*' globs.");
    app
      .add_option(
        "--stamp", args_.stamp,
        "Reference clock for the window: 'header' (header.stamp; per-message fallback to "
        "receive time) or 'recv' (record time, fastest).")
      ->check(CLI::IsMember({"header", "recv"}))
      ->capture_default_str();
    end_opt->excludes(dur_opt);
    both_opt->excludes(start_opt);
    both_opt->excludes(end_opt);
    both_opt->excludes(dur_opt);
    align_opt->excludes(start_opt);
    align_opt->excludes(end_opt);
    align_opt->excludes(dur_opt);
    align_opt->excludes(both_opt);
    app.add_option(
      "-o,--output", args_.output_path,
      "Write the result to this new bag instead of rewriting <input> in place.");
    app.add_flag(
      "-w,--overwrite", args_.overwrite,
      "Replace an existing -o/--output path. Without it, an existing output path stops the run.");
  }

  int run() override { return run_trim(args_); }

private:
  TrimArgs args_;
};

BAGWIZ_REGISTER_COMMAND(TrimCommand)

}  // namespace bagwiz::commands
