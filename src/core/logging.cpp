// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/logging.hpp"

#include <rcutils/logging.h>

#include <cstdlib>

namespace bagwiz::core
{

namespace
{
// Console line format without rcutils' default leading "[{time}]" field.
// bagwiz is an interactive CLI, so the wall-clock unix timestamp on every
// diagnostic line is noise rather than signal; dropping it keeps error
// output readable. The remaining fields match rcutils' default layout.
constexpr const char * kConsoleOutputFormat = "[{severity}] [{name}]: {message}";
}  // namespace

void init_logging()
{
  // Install the timestamp-free format as the default. The zero `overwrite`
  // flag leaves an explicit RCUTILS_CONSOLE_OUTPUT_FORMAT export untouched,
  // so users can still opt back into timestamps (or any layout) via the
  // environment. This must run before rcutils_logging_initialize(), which
  // reads the variable once at initialization time. setenv only fails on an
  // invalid name or allocation failure; on failure rcutils simply keeps its
  // own default, so the ignored return is safe.
  [[maybe_unused]] const int set_ret =
    setenv("RCUTILS_CONSOLE_OUTPUT_FORMAT", kConsoleOutputFormat, /*overwrite=*/0);

  // rcutils_logging_initialize() is idempotent and reads
  // RCUTILS_LOGGING_DEFAULT_LEVEL / RCUTILS_CONSOLE_OUTPUT_FORMAT from the
  // environment. If it fails we continue silently; the macros will fall
  // back to stderr via rcutils' own default path. The function is marked
  // warn_unused_result, so capture the value explicitly.
  [[maybe_unused]] const auto ret = rcutils_logging_initialize();
}

}  // namespace bagwiz::core
