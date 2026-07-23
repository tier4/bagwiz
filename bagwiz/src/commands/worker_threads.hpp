// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__WORKER_THREADS_HPP_
#define COMMANDS__WORKER_THREADS_HPP_

// Shared -j/--threads resolution for commands with a parallel pass
// (`pcd undistort`, `pcd concat`). CLI-internal: this header lives with the
// command sources and is not installed.
namespace bagwiz::commands
{

// Worker count for a parallel pass: --threads, clamped to the hardware
// concurrency when that is known. A non-positive value (0 = "auto") resolves
// to the hardware concurrency (1 when unknown); the CLI default of 8 is
// applied by the caller before this function sees the value.
[[nodiscard]] inline int resolve_num_threads(const int requested, const unsigned int hardware)
{
  int num_threads = (requested <= 0) ? static_cast<int>(hardware != 0u ? hardware : 1u) : requested;
  if (hardware > 0 && num_threads > static_cast<int>(hardware)) {
    num_threads = static_cast<int>(hardware);
  }
  return num_threads;
}

}  // namespace bagwiz::commands

#endif  // COMMANDS__WORKER_THREADS_HPP_
