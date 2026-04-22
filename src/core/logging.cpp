// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagcli/core/logging.hpp"

#include <rcutils/logging.h>

namespace bagcli::core
{

void init_logging()
{
  // rcutils_logging_initialize() is idempotent and reads
  // RCUTILS_LOGGING_DEFAULT_LEVEL / RCUTILS_CONSOLE_OUTPUT_FORMAT from the
  // environment. If it fails we continue silently; the macros will fall
  // back to stderr via rcutils' own default path. The function is marked
  // warn_unused_result, so capture the value explicitly.
  [[maybe_unused]] const auto ret = rcutils_logging_initialize();
}

}  // namespace bagcli::core
