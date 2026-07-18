// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__MAP_VIEWER_OPEN_HPP_
#define BAGWIZ__CORE__SLAM__MAP_VIEWER_OPEN_HPP_

#include <string>

// GLIM-free helper for `bagwiz map slam --viewer`. Kept separate from the
// httplib-backed viewer server so the (pure, dependency-free) command builder
// builds and is unit-tested in every configuration, not only when
// BAGWIZ_WITH_SLAM is on.
namespace bagwiz::core::slam
{

// Build a shell command that opens `url` in the host's default browser. The
// command is platform-dispatched (Linux: xdg-open, macOS: open, Windows: start)
// and, on POSIX, backgrounded so the caller can proceed to block on its own HTTP
// server. `url` is expected to be a bagwiz-constructed loopback URL
// (http://127.0.0.1:<port>/), not user input, so it is only wrapped in quotes
// rather than fully shell-escaped. Returned for use with std::system; kept as a
// pure string builder (no process launch) so it is unit-testable without a
// display.
std::string browser_open_command(const std::string & url);

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__MAP_VIEWER_OPEN_HPP_
