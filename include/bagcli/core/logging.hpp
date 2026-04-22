// Copyright 2026 Mineto Tsukada
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGCLI__CORE__LOGGING_HPP_
#define BAGCLI__CORE__LOGGING_HPP_

#include <rcutils/logging_macros.h>

namespace bagcli::core
{

// Initialize the ROS 2 (rcutils) logging subsystem. Safe to call multiple
// times; subsequent calls are no-ops.
void init_logging();

}  // namespace bagcli::core

// Logger name convention: "bagcli.<module>.<sub>", e.g. "bagcli.io.mcap".
// All diagnostic output MUST go through these macros; stdout is reserved for
// command data output so pipes remain clean.
#define BAGCLI_LOG_DEBUG(name, ...) RCUTILS_LOG_DEBUG_NAMED(name, __VA_ARGS__)
#define BAGCLI_LOG_INFO(name, ...) RCUTILS_LOG_INFO_NAMED(name, __VA_ARGS__)
#define BAGCLI_LOG_WARN(name, ...) RCUTILS_LOG_WARN_NAMED(name, __VA_ARGS__)
#define BAGCLI_LOG_ERROR(name, ...) RCUTILS_LOG_ERROR_NAMED(name, __VA_ARGS__)
#define BAGCLI_LOG_FATAL(name, ...) RCUTILS_LOG_FATAL_NAMED(name, __VA_ARGS__)

#endif  // BAGCLI__CORE__LOGGING_HPP_
