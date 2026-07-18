// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/msg_schema/schema_model.hpp"

#include <string>
#include <string_view>
#include <utility>

namespace bagwiz::core::msg_schema
{

const MessageDef * SchemaModel::find(std::string_view type_name) const noexcept
{
  // Try short form ("pkg/Type") first; fall through to canonical
  // ("pkg/msg/Type") so callers can pass either without preprocessing.
  if (auto it = by_short_name_.find(std::string(type_name)); it != by_short_name_.end()) {
    return &definitions_[it->second];
  }
  if (auto it = by_canonical_name_.find(std::string(type_name)); it != by_canonical_name_.end()) {
    return &definitions_[it->second];
  }
  return nullptr;
}

void SchemaModel::add(MessageDef def, bool is_root)
{
  const std::size_t index = definitions_.size();
  const std::string short_key = def.short_name();
  const std::string canonical_key = def.canonical_name();

  // De-dup: if the same short name was already registered, keep the first
  // definition and silently drop the duplicate. Real MCAP schemas should
  // not contain duplicates, but defensively tolerating them avoids hard
  // failures on hand-edited inputs.
  if (by_short_name_.count(short_key) != 0) {
    return;
  }

  definitions_.push_back(std::move(def));
  by_short_name_.emplace(short_key, index);
  by_canonical_name_.emplace(canonical_key, index);

  if (is_root && !root_index_.has_value()) {
    root_index_ = index;
    root_short_name_ = short_key;
  }
}

}  // namespace bagwiz::core::msg_schema
