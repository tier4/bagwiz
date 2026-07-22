// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "map_slam_threads.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <algorithm>
#include <thread>

namespace bagwiz::commands
{

int resolve_threads(const int num_threads)
{
  const unsigned int hardware = std::thread::hardware_concurrency();
  const int limit = hardware > 0 ? static_cast<int>(hardware) : 1;
  if (num_threads <= 0) {
    return limit;
  }
  return std::min(num_threads, limit);
}

}  // namespace bagwiz::commands
