// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/logging.hpp"

#include <rcutils/logging.h>
#include <rcutils/time.h>
#include <unistd.h>

#include <array>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>

namespace bagwiz::core
{

namespace
{

// Short label for an rcutils severity level. Falls back to "UNKNOWN" for any
// value outside the known set rather than indexing past a lookup table.
const char * severity_label(int severity)
{
  switch (severity) {
    case RCUTILS_LOG_SEVERITY_DEBUG:
      return "DEBUG";
    case RCUTILS_LOG_SEVERITY_INFO:
      return "INFO";
    case RCUTILS_LOG_SEVERITY_WARN:
      return "WARN";
    case RCUTILS_LOG_SEVERITY_ERROR:
      return "ERROR";
    case RCUTILS_LOG_SEVERITY_FATAL:
      return "FATAL";
    default:
      return "UNKNOWN";
  }
}

// ANSI colour escape for a severity, matching rcutils' default console scheme:
// DEBUG green, WARN yellow, ERROR and FATAL red. INFO and any unknown level are
// left uncoloured (empty string), so they print in the terminal's default ink.
const char * severity_color(int severity)
{
  switch (severity) {
    case RCUTILS_LOG_SEVERITY_DEBUG:
      return "\033[32m";
    case RCUTILS_LOG_SEVERITY_WARN:
      return "\033[33m";
    case RCUTILS_LOG_SEVERITY_ERROR:
    case RCUTILS_LOG_SEVERITY_FATAL:
      return "\033[31m";
    default:
      return "";
  }
}

// Render an rcutils nanosecond timestamp as a human-readable local datetime
// "YYYY-MM-DD HH:MM:SS.mmm TZ" (e.g. "2026-06-08 00:31:31.010 JST"). rcutils'
// default console layout prints the timestamp as a raw unix-epoch float, which
// is hard to read at a glance; a wall-clock calendar date in the host's local
// timezone is far friendlier. On any conversion failure we fall back to the
// raw nanosecond count so a line is still emitted.
std::string format_local_datetime(rcutils_time_point_value_t timestamp)
{
  constexpr std::int64_t kNanosPerSecond = 1'000'000'000;
  constexpr std::int64_t kNanosPerMilli = 1'000'000;

  const auto seconds = static_cast<std::time_t>(timestamp / kNanosPerSecond);
  const auto millis = static_cast<int>((timestamp % kNanosPerSecond) / kNanosPerMilli);

  std::tm tm_local{};
  // localtime_r resolves the host's local timezone and is thread-safe (no
  // shared static buffer of localtime()).
  if (::localtime_r(&seconds, &tm_local) == nullptr) {
    return std::to_string(timestamp) + "ns";
  }

  std::array<char, 24> date{};
  if (std::strftime(date.data(), date.size(), "%Y-%m-%d %H:%M:%S", &tm_local) == 0) {
    return std::to_string(timestamp) + "ns";
  }

  // %Z is the local timezone abbreviation (e.g. "JST"). It can be empty on a
  // host without zone data, in which case the suffix is dropped.
  std::array<char, 16> zone{};
  std::strftime(zone.data(), zone.size(), "%Z", &tm_local);

  std::array<char, 48> line{};
  if (zone.front() == '\0') {
    std::snprintf(line.data(), line.size(), "%s.%03d", date.data(), millis);
  } else {
    std::snprintf(line.data(), line.size(), "%s.%03d %s", date.data(), millis, zone.data());
  }
  return {line.data()};
}

// Expand a printf-style log format and its arguments into a single string.
std::string expand_message(const char * format, va_list * args)
{
  if (format == nullptr || args == nullptr) {
    return {};
  }

  // First pass measures the required length on a copy of the args; the
  // original va_list is consumed by the second pass that fills the buffer.
  va_list measure;
  va_copy(measure, *args);
  const int needed = std::vsnprintf(nullptr, 0, format, measure);
  va_end(measure);

  if (needed <= 0) {
    return {};
  }

  std::string message(static_cast<std::size_t>(needed), '\0');
  [[maybe_unused]] const int written =
    std::vsnprintf(message.data(), message.size() + 1, format, *args);
  return message;
}

// Whether to wrap a line in ANSI colour escapes, mirroring rcutils' own policy
// via the same RCUTILS_COLORIZED_OUTPUT variable: "1" forces colour on, "0"
// forces it off, and when unset (or any other value) colour is used only if
// stderr is a terminal. Re-evaluated per line rather than cached so the
// variable can be toggled at runtime (e.g. in tests); the getenv/isatty pair
// is negligible for an interactive CLI.
bool colorize_output()
{
  const char * env = std::getenv("RCUTILS_COLORIZED_OUTPUT");
  if (env != nullptr) {
    if (std::strcmp(env, "1") == 0) {
      return true;
    }
    if (std::strcmp(env, "0") == 0) {
      return false;
    }
  }
  return ::isatty(STDERR_FILENO) != 0;
}

// Replacement rcutils console handler. Every level is emitted uniformly as
// "[SEVERITY] [YYYY-MM-DD HH:MM:SS.mmm TZ] [name]: message" on stderr, so a
// human-readable local date replaces rcutils' default unix-epoch timestamp
// across all log levels. The whole line is wrapped in the severity's ANSI
// colour (matching rcutils' scheme) when colour is enabled. stderr (not
// stdout) keeps command data output pipe-clean, per the convention in
// logging.hpp.
void console_output_handler(
  const rcutils_log_location_t * /*location*/, int severity, const char * name,
  rcutils_time_point_value_t timestamp, const char * format, va_list * args)
{
  const std::string datetime = format_local_datetime(timestamp);
  const std::string message = expand_message(format, args);

  const char * color = colorize_output() ? severity_color(severity) : "";
  const char * reset = *color != '\0' ? "\033[0m" : "";

  std::fprintf(
    stderr, "%s[%s] [%s] [%s]: %s%s\n", color, severity_label(severity), datetime.c_str(),
    name != nullptr ? name : "", message.c_str(), reset);
}

}  // namespace

void init_logging()
{
  // rcutils_logging_initialize() is idempotent and installs the default
  // console handler. It is marked warn_unused_result, so capture the value.
  [[maybe_unused]] const auto ret = rcutils_logging_initialize();

  // Override the default handler so the timestamp renders as a human-readable
  // calendar date instead of a raw unix epoch, uniformly across every level.
  // Re-installing the same handler on subsequent calls is harmless.
  rcutils_logging_set_output_handler(console_output_handler);
}

}  // namespace bagwiz::core
