// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pipeline/topic_router.hpp"

#include "bagwiz/core/pipeline/rewrite_backend.hpp"

#include <string>

namespace bagwiz::core::pipeline
{

Emit SuppressRouter::route(const std::string & in_topic) const
{
  if (suppress_.contains(in_topic)) {
    return Emit{false, {}};
  }
  return Emit{true, in_topic};
}

Emit RenameRouter::route(const std::string & in_topic) const
{
  const auto it = rename_.find(in_topic);
  if (it != rename_.end()) {
    return Emit{true, it->second};
  }
  return Emit{true, in_topic};
}

}  // namespace bagwiz::core::pipeline
