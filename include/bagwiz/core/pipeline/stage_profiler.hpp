// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__PIPELINE__STAGE_PROFILER_HPP_
#define BAGWIZ__CORE__PIPELINE__STAGE_PROFILER_HPP_

#include <chrono>
#include <cstdint>
#include <string>
#include <string_view>

// Phase-0 instrumentation for the read -> process -> write rewrite loop.
//
// A StageProfiler accumulates wall-clock time spent in each logical stage of a
// rewrite command (read+decompress, process/transform, write) plus message and
// byte counters, and prints a one-shot bottleneck report on demand. It is gated
// behind the BAGWIZ_PROFILE environment variable so it ships dormant: the
// default-constructed profiler is enabled only when BAGWIZ_PROFILE is set to a
// truthy value, and a disabled profiler's scope timers are no-ops with no clock
// reads on the hot path.
//
// This lives in the pipeline namespace because the abstraction built on top of
// it (SequentialBackend and the threaded backends) reuses the same profiler to
// report per-stage timings under every backend.
namespace bagwiz::core::pipeline
{

// The three logical stages of a rewrite. kProcess is zero-cost for pure-copy
// commands (drop/keep/rename/convert-format) and non-trivial only for decode
// /transform commands (convert msg geo).
enum class Stage : std::uint8_t { kRead, kProcess, kWrite };

// Parse a BAGWIZ_PROFILE value. nullptr, empty, and the common falsey spellings
// ("0", "false", "no", "off", case-insensitive) disable profiling; any other
// non-empty value enables it. Pure and side-effect free for testability.
[[nodiscard]] bool profile_value_enabled(const char * value) noexcept;

// Plain accumulators. Public so the pure formatter and tests can build a report
// without timing anything.
struct StageTotals
{
  std::int64_t read_ns = 0;
  std::int64_t process_ns = 0;
  std::int64_t write_ns = 0;
  std::uint64_t messages = 0;
  std::uint64_t in_bytes = 0;   // payload bytes pulled from the reader
  std::uint64_t out_bytes = 0;  // payload bytes handed to the writer
};

// Pure, deterministic, multi-line human-readable report. Safe for all-zero
// totals (no division by zero). `command` is echoed so a profiled run is
// self-identifying. Exposed for unit testing independent of any wall clock.
[[nodiscard]] std::string format_stage_report(std::string_view command, const StageTotals & totals);

// Accumulates per-stage timings and counters. Cheap to construct; when disabled
// the scope timers do nothing.
class StageProfiler
{
public:
  // Enabled from getenv("BAGWIZ_PROFILE") via profile_value_enabled().
  StageProfiler();
  // Explicit control for tests and callers that already resolved the flag.
  explicit StageProfiler(bool enabled);

  [[nodiscard]] bool enabled() const noexcept { return enabled_; }

  // RAII stage timer. A Scope constructed from a disabled profiler (p == null)
  // never reads the clock and never accumulates.
  class Scope
  {
  public:
    Scope(StageProfiler * profiler, Stage stage) noexcept;
    ~Scope();

    // Non-copyable, non-movable: the lifetime is a single lexical scope.
    Scope(const Scope &) = delete;
    Scope & operator=(const Scope &) = delete;
    Scope(Scope &&) = delete;
    Scope & operator=(Scope &&) = delete;

  private:
    StageProfiler * profiler_;
    Stage stage_;
    std::chrono::steady_clock::time_point start_;
  };

  // Returns a Scope that times `stage` until it leaves scope. No-op when
  // disabled. Usage: { auto s = prof.time(Stage::kRead); reader.next(msg); }
  [[nodiscard]] Scope time(Stage stage) noexcept { return Scope(enabled_ ? this : nullptr, stage); }

  // Direct accumulation (used by Scope and by tests).
  void add(Stage stage, std::chrono::nanoseconds elapsed) noexcept;
  void add_message(std::uint64_t in_bytes, std::uint64_t out_bytes) noexcept;

  [[nodiscard]] const StageTotals & totals() const noexcept { return totals_; }

  // Emit format_stage_report(command, totals()) via the logger. No-op when
  // disabled, so callers can invoke it unconditionally at command exit.
  void report(std::string_view command) const;

private:
  bool enabled_;
  StageTotals totals_;
};

}  // namespace bagwiz::core::pipeline

#endif  // BAGWIZ__CORE__PIPELINE__STAGE_PROFILER_HPP_
