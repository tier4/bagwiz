// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__MAP_SLAM_THREADS_HPP_
#define COMMANDS__MAP_SLAM_THREADS_HPP_

// The `map slam` thread-count rule, shared by the mapping run and the `--cam`
// colorize pass. Deliberately a LEAF header: it pulls in no SLAM types, so the
// colorize unit — which needs nothing else from the mapping internals — does
// not have to drag in the GLIM-coupled map_slam_mapping.hpp. CLI-internal:
// this header lives with the command sources and is not installed.
namespace bagwiz::commands
{

// Clamp an explicit --threads value to the host's hardware concurrency so the
// user cannot oversubscribe the machine. A value <= 0 or a concurrency that
// cannot be queried leaves the argument unchanged (the caller applies
// defaults).
[[nodiscard]] int cap_threads_at_hardware_limit(int num_threads);

}  // namespace bagwiz::commands

#endif  // COMMANDS__MAP_SLAM_THREADS_HPP_
