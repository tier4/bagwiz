// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__MAP_SLAM_THREADS_HPP_
#define COMMANDS__MAP_SLAM_THREADS_HPP_

// The `map slam` thread-count rule, shared by the mapping run and the `--color`
// colorize pass. Deliberately a LEAF header: it pulls in no SLAM types, so the
// colorize unit — which needs nothing else from the mapping internals — does
// not have to drag in the GLIM-coupled map_slam_mapping.hpp. CLI-internal:
// this header lives with the command sources and is not installed.
namespace bagwiz::commands
{

// Resolve --threads to an effective worker count. 0 means "auto": the host's
// hardware concurrency (1 when it cannot be queried). A positive value is
// clamped to the hardware concurrency so the user cannot oversubscribe the
// machine.
[[nodiscard]] int resolve_threads(int num_threads);

}  // namespace bagwiz::commands

#endif  // COMMANDS__MAP_SLAM_THREADS_HPP_
