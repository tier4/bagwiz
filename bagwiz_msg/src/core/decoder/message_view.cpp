// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/decoder/message_view.hpp"

#include "bagwiz/core/cdr_walker/value.hpp"

#include <cstddef>
#include <optional>
#include <string_view>
#include <variant>

namespace bagwiz::core::decoder
{

std::size_t MessageView::index_of(std::string_view name) const noexcept
{
  const std::size_t n = obj_->fields.size();
  for (std::size_t i = 0; i < n; ++i) {
    if (obj_->fields[i].first == name) {
      return i;
    }
  }
  return n;
}

std::optional<MessageView> MessageView::nested(std::string_view name) const noexcept
{
  const auto i = index_of(name);
  if (i == size()) {
    return std::nullopt;
  }
  if (const auto * obj = std::get_if<cdr_walker::Object>(&at(i).v)) {
    return MessageView{*obj};
  }
  return std::nullopt;
}

std::optional<std::size_t> MessageView::sequence_length(std::string_view name) const noexcept
{
  const auto i = index_of(name);
  if (i == size()) {
    return std::nullopt;
  }
  if (const auto * seq = std::get_if<cdr_walker::Sequence>(&at(i).v)) {
    return seq->elements.size();
  }
  return std::nullopt;
}

std::optional<MessageView> MessageView::nested_element(
  std::string_view name, std::size_t i) const noexcept
{
  const auto fi = index_of(name);
  if (fi == size()) {
    return std::nullopt;
  }
  const auto * seq = std::get_if<cdr_walker::Sequence>(&at(fi).v);
  if (seq == nullptr || i >= seq->elements.size()) {
    return std::nullopt;
  }
  if (const auto * obj = std::get_if<cdr_walker::Object>(&seq->elements[i].v)) {
    return MessageView{*obj};
  }
  return std::nullopt;
}

}  // namespace bagwiz::core::decoder
