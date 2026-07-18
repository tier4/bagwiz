// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__DECODER__MESSAGE_VIEW_HPP_
#define BAGWIZ__CORE__DECODER__MESSAGE_VIEW_HPP_

#include "bagwiz/core/cdr_walker/value.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <variant>

// Lightweight, non-owning view over a single decoded message. Wraps the
// Value tree the cdr_walker (or the introspection-to-Value adapter)
// produced and adds:
//   - field-name lookup (the underlying Object stores fields as ordered
//     pairs; lookups are O(N) but messages have single-digit field counts
//     in practice)
//   - typed primitive accessors that return std::nullopt rather than
//     throwing on type mismatch, so consumers can probe optional fields
//   - convenience entry points for nested objects and sequences
//
// MessageView holds a pointer into the underlying Value; the caller must
// keep the Value alive for the view's lifetime. Constructing a view does
// not allocate.
namespace bagwiz::core::decoder
{

class MessageView
{
public:
  explicit MessageView(const cdr_walker::Object & obj) noexcept : obj_(&obj) {}

  // Number of fields in declaration order.
  std::size_t size() const noexcept { return obj_->fields.size(); }

  // Field name at the given index; precondition: i < size().
  std::string_view name_at(std::size_t i) const noexcept { return obj_->fields[i].first; }

  // Raw value at the given index; precondition: i < size().
  const cdr_walker::Value & at(std::size_t i) const noexcept { return obj_->fields[i].second; }

  // Linear lookup. Returns size() when not found so range checks at the
  // call site can stay branch-light.
  std::size_t index_of(std::string_view name) const noexcept;

  // Typed primitive accessor. Returns std::nullopt when:
  //   - the field is missing
  //   - the field is not a primitive (Object / Sequence)
  //   - the underlying primitive type does not match T
  // T must be one of the variants in cdr_walker::Value.
  // `name` is taken by const-ref because cppcheck's passedByValue check
  // flags string_view-by-value in templated function bodies even though
  // by-value is the canonical idiom; const-ref sidesteps the warning
  // without changing semantics.
  template <typename T>
  std::optional<T> primitive(const std::string_view & name) const
  {
    const auto i = index_of(name);
    if (i == size()) {
      return std::nullopt;
    }
    if (const auto * p = std::get_if<T>(&at(i).v)) {
      return *p;
    }
    return std::nullopt;
  }

  // Nested object. Returns std::nullopt when the field is missing or is
  // not an Object. The returned view borrows from the underlying Value;
  // it must not outlive `*this`.
  std::optional<MessageView> nested(std::string_view name) const noexcept;

  // Length of the sequence at the named field. Returns std::nullopt when
  // the field is missing or is not a Sequence.
  std::optional<std::size_t> sequence_length(std::string_view name) const noexcept;

  // Element-of-nested-sequence accessor. Returns std::nullopt when the
  // field is missing, the field is not a Sequence, the index is out of
  // range, or the element is not an Object.
  std::optional<MessageView> nested_element(std::string_view name, std::size_t i) const noexcept;

  // Direct access to the underlying Object — used by streaming consumers
  // (the YAML formatter walks fields() directly to avoid per-field
  // lookups).
  const cdr_walker::Object & object() const noexcept { return *obj_; }

private:
  const cdr_walker::Object * obj_;
};

}  // namespace bagwiz::core::decoder

#endif  // BAGWIZ__CORE__DECODER__MESSAGE_VIEW_HPP_
