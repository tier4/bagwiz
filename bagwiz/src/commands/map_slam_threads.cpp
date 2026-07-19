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

int cap_threads_at_hardware_limit(int num_threads)
{
  if (num_threads <= 0) {
    return num_threads;
  }
  const unsigned int hardware = std::thread::hardware_concurrency();
  if (hardware == 0) {
    return num_threads;
  }
  return std::min(num_threads, static_cast<int>(hardware));
}

}  // namespace bagwiz::commands
