// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pipeline/stage_profiler.hpp"

#include "bagwiz/core/base/logging.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <string>

namespace bagwiz::core::pipeline
{

namespace
{
constexpr const char * kLogger = "bagwiz.pipeline.profile";

// Unit-conversion constants shared by the report helpers (ES.45: no repeated
// magic numbers; the MiB divisor in particular appears in several formulas).
constexpr double kNanosPerSecond = 1e9;
constexpr double kBytesPerMiB = 1024.0 * 1024.0;

double to_seconds(std::int64_t ns)
{
  return static_cast<double>(ns) / kNanosPerSecond;
}

double percent(std::int64_t part, std::int64_t total)
{
  return total > 0 ? (100.0 * static_cast<double>(part) / static_cast<double>(total)) : 0.0;
}

// MiB/s of `bytes` over `ns` nanoseconds; 0 when no time elapsed.
double mib_per_s(std::uint64_t bytes, std::int64_t ns)
{
  if (ns <= 0) {
    return 0.0;
  }
  const double mib = static_cast<double>(bytes) / kBytesPerMiB;
  return mib / to_seconds(ns);
}

void emit_stage_line(
  std::ostringstream & os, const char * name, std::int64_t ns, std::int64_t total_ns)
{
  os << "  " << std::left << std::setw(8) << name << std::right << std::fixed
     << std::setprecision(3) << std::setw(9) << to_seconds(ns) << " s  " << std::setw(5)
     << std::setprecision(1) << percent(ns, total_ns) << " %\n";
}
}  // namespace

bool profile_value_enabled(const char * value) noexcept
{
  if (value == nullptr) {
    return false;
  }
  std::string v(value);
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return !(v.empty() || v == "0" || v == "false" || v == "no" || v == "off");
}

// cppcheck-suppress passedByValue  // std::string_view is a cheap value type
std::string format_stage_report(std::string_view command, const StageTotals & t)
{
  const std::int64_t stage_ns = t.read_ns + t.process_ns + t.write_ns;
  // Percentages are shares of the real elapsed time, so a backend that overlaps
  // its stages shows each one's true occupancy of the run (and they can sum to
  // more than 100 %, which is exactly the signal). Without a measured elapsed
  // the stage sum is all that is available, and then the two coincide.
  const std::int64_t wall_ns = t.elapsed_ns > 0 ? t.elapsed_ns : stage_ns;

  std::ostringstream os;
  os << "profile [" << command << "]: wall " << std::fixed << std::setprecision(3)
     << to_seconds(wall_ns) << " s, " << t.messages << " msg, "
     << (static_cast<double>(t.in_bytes) / kBytesPerMiB) << " MiB in / "
     << (static_cast<double>(t.out_bytes) / kBytesPerMiB) << " MiB out\n";
  emit_stage_line(os, "read", t.read_ns, wall_ns);
  emit_stage_line(os, "process", t.process_ns, wall_ns);
  emit_stage_line(os, "write", t.write_ns, wall_ns);
  if (stage_ns > wall_ns) {
    // Say it outright: otherwise a reader sums the stage lines, gets more than
    // the wall, and concludes the threaded backend is the slow one.
    os << "  stages overlap: " << std::fixed << std::setprecision(3) << to_seconds(stage_ns)
       << " s of stage time in " << to_seconds(wall_ns) << " s of wall\n";
  }
  os << "  read " << std::setprecision(1) << mib_per_s(t.in_bytes, t.read_ns) << " MiB/s, write "
     << mib_per_s(t.out_bytes, t.write_ns) << " MiB/s";
  return os.str();
}

// --- StageProfiler -----------------------------------------------------------

StageProfiler::StageProfiler() : enabled_(profile_value_enabled(std::getenv("BAGWIZ_PROFILE")))
{
}

StageProfiler::StageProfiler(bool enabled) : enabled_(enabled)
{
}

void StageProfiler::add(Stage stage, std::chrono::nanoseconds elapsed) noexcept
{
  const auto ns = static_cast<std::int64_t>(elapsed.count());
  switch (stage) {
    case Stage::kRead:
      totals_.read_ns += ns;
      break;
    case Stage::kProcess:
      totals_.process_ns += ns;
      break;
    case Stage::kWrite:
      totals_.write_ns += ns;
      break;
  }
}

void StageProfiler::set_elapsed(std::chrono::nanoseconds elapsed) noexcept
{
  totals_.elapsed_ns = static_cast<std::int64_t>(elapsed.count());
}

void StageProfiler::add_message(std::uint64_t in_bytes, std::uint64_t out_bytes) noexcept
{
  ++totals_.messages;
  totals_.in_bytes += in_bytes;
  totals_.out_bytes += out_bytes;
}

void StageProfiler::report(std::string_view command) const
{
  if (!enabled_) {
    return;
  }
  const std::string text = format_stage_report(command, totals_);
  BAGWIZ_LOG_INFO(kLogger, "%s", text.c_str());
}

// --- StageProfiler::Scope ----------------------------------------------------

StageProfiler::Scope::Scope(StageProfiler * profiler, Stage stage) noexcept
: profiler_(profiler), stage_(stage)
{
  if (profiler_ != nullptr) {
    start_ = std::chrono::steady_clock::now();
  }
}

StageProfiler::Scope::~Scope()
{
  if (profiler_ != nullptr) {
    profiler_->add(stage_, std::chrono::steady_clock::now() - start_);
  }
}

}  // namespace bagwiz::core::pipeline
