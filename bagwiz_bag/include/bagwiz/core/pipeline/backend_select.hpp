// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__PIPELINE__BACKEND_SELECT_HPP_
#define BAGWIZ__CORE__PIPELINE__BACKEND_SELECT_HPP_

#include "bagwiz/core/pipeline/rewrite_backend.hpp"

#include <memory>
#include <optional>

// Picks which Backend a rewrite runs on. A command supplies its preferred
// strategy (the pure-copy trio asks for Pipelined; everything else defaults to
// Sequential), and the BAGWIZ_BACKEND environment variable can override it at
// runtime. The override exists so both strategies can be benchmarked on one
// binary and as an escape hatch back to the Sequential oracle.
namespace bagwiz::core::pipeline
{

enum class BackendKind { Sequential, Pipelined };

// Parse a BAGWIZ_BACKEND value into an override. nullptr / empty / unrecognized
// yields std::nullopt so the caller keeps its own default. Accepted (case
// -insensitive): "sequential"/"seq" and "pipelined"/"pipeline"/"pipe". Pure and
// side-effect free for testability (mirrors profile_value_enabled).
[[nodiscard]] std::optional<BackendKind> parse_backend_override(const char * value) noexcept;

// Build the Backend the caller asked for, honoring a BAGWIZ_BACKEND override
// when present. Never returns null.
[[nodiscard]] std::unique_ptr<Backend> make_backend(BackendKind default_kind);

}  // namespace bagwiz::core::pipeline

#endif  // BAGWIZ__CORE__PIPELINE__BACKEND_SELECT_HPP_
