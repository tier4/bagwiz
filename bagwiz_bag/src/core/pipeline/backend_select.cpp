// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pipeline/backend_select.hpp"

#include "bagwiz/core/pipeline/pipelined_backend.hpp"
#include "bagwiz/core/pipeline/rewrite_backend.hpp"
#include "bagwiz/core/pipeline/sequential_backend.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>

namespace bagwiz::core::pipeline
{

std::optional<BackendKind> parse_backend_override(const char * value) noexcept
{
  if (value == nullptr) {
    return std::nullopt;
  }
  std::string v(value);
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  if (v == "sequential" || v == "seq") {
    return BackendKind::Sequential;
  }
  if (v == "pipelined" || v == "pipeline" || v == "pipe") {
    return BackendKind::Pipelined;
  }
  return std::nullopt;  // empty / unknown: keep the caller's default
}

std::unique_ptr<Backend> make_backend(BackendKind default_kind)
{
  const auto override_kind = parse_backend_override(std::getenv("BAGWIZ_BACKEND"));
  switch (override_kind.value_or(default_kind)) {
    case BackendKind::Pipelined:
      return std::make_unique<PipelinedBackend>();
    case BackendKind::Sequential:
      break;
  }
  return std::make_unique<SequentialBackend>();
}

}  // namespace bagwiz::core::pipeline
