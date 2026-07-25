// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef IO__READ_TUNING_HPP_
#define IO__READ_TUNING_HPP_

#include "bagwiz/core/base/logging.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <thread>

// Src-local tuning knobs shared by the parallel read paths. Both storage
// backends read the same BAGWIZ_READ_THREADS variable so a differential run
// (`0` = serial) switches every backend at once.
namespace bagwiz::io::detail
{

// Parse an integer environment variable and clamp it to [lo, hi]. An unset or
// empty variable yields `fallback`; an unparsable one logs a warning under
// `logger` and also yields `fallback`.
inline std::int64_t resolve_env_int(
  const char * name, std::int64_t fallback, std::int64_t lo, std::int64_t hi, const char * logger)
{
  const char * env = std::getenv(name);
  if (env == nullptr || *env == '\0') {
    return fallback;
  }
  char * end = nullptr;
  const long long parsed = std::strtoll(env, &end, 10);  // NOLINT(runtime/int) strtoll API
  if (end == env || *end != '\0') {
    BAGWIZ_LOG_WARN(logger, "ignoring unparsable %s='%s'", name, env);
    return fallback;
  }
  return std::clamp<std::int64_t>(parsed, lo, hi);
}

// Worker count for the parallel read paths (per-chunk decompression on mcap,
// per-slice scanning on db3). Defaults to 8, capped at the host's hardware
// concurrency so low-core machines keep a smaller worker count
// (docs/benchmarks/mcap-read-threads.md and docs/benchmarks/db3-read-threads.md
// have the sweeps the default is based on). BAGWIZ_READ_THREADS overrides the
// default, and 0 or 1 selects the serial read (the debugging escape hatch).
inline int resolve_read_threads(const char * logger)
{
  constexpr std::int64_t kDefault = 8;
  constexpr std::int64_t kMax = 16;
  const unsigned int hw = std::thread::hardware_concurrency();
  const std::int64_t fallback =
    hw == 0 ? kDefault : std::min<std::int64_t>(kDefault, static_cast<std::int64_t>(hw));
  return static_cast<int>(resolve_env_int("BAGWIZ_READ_THREADS", fallback, 0, kMax, logger));
}

}  // namespace bagwiz::io::detail

#endif  // IO__READ_TUNING_HPP_
