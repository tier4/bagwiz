// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "trim_stamp.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/msg_yaml/msg_definition_resolver.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <unordered_map>

namespace bagwiz::commands
{

namespace
{

std::string_view trim_ws(std::string_view s)
{
  while (!s.empty() && (s.front() == ' ' || s.front() == '\t' || s.front() == '\r')) {
    s.remove_prefix(1);
  }
  while (!s.empty() && (s.back() == ' ' || s.back() == '\t' || s.back() == '\r')) {
    s.remove_suffix(1);
  }
  return s;
}

}  // namespace

bool schema_leads_with_header(std::string_view schema_text)
{
  while (!schema_text.empty()) {
    const auto nl = schema_text.find('\n');
    const std::string_view line = trim_ws(schema_text.substr(0, nl));
    schema_text = (nl == std::string_view::npos) ? std::string_view{} : schema_text.substr(nl + 1);

    if (line.empty() || line.front() == '#') {
      continue;  // blank / comment
    }
    if (line.starts_with("===") || line.starts_with("MSG:")) {
      return false;  // end of the top-level block without a field line
    }
    if (line.find('=') != std::string_view::npos) {
      continue;  // constant declaration (TYPE NAME=VALUE); not a field
    }
    // First field line of the top-level type: `<type> <name> [default]`.
    const auto space = line.find_first_of(" \t");
    if (space == std::string_view::npos) {
      return false;  // malformed field line
    }
    const std::string_view type = line.substr(0, space);
    const std::string_view rest = trim_ws(line.substr(space));
    const auto name_end = rest.find_first_of(" \t");
    const std::string_view name = rest.substr(0, name_end);
    return (type == "std_msgs/Header" || type == "std_msgs/msg/Header" || type == "Header") &&
           name == "header";
  }
  return false;
}

std::optional<std::int64_t> read_leading_header_stamp_ns(std::span<const std::byte> payload)
{
  // CDR encapsulation: 2-byte representation id (byte 1's LSB: 1 = little
  // endian) + 2 options bytes; then int32 sec + uint32 nanosec, both 4-aligned
  // at offsets 4 and 8.
  constexpr std::size_t kStampEnd = 12;
  if (payload.size() < kStampEnd) {
    return std::nullopt;
  }
  const bool little_endian = (static_cast<unsigned char>(payload[1]) & 0x01U) != 0;
  const auto u32_at = [&](std::size_t off) {
    const auto b = [&](std::size_t i) {
      return static_cast<std::uint32_t>(static_cast<unsigned char>(payload[off + i]));
    };
    return little_endian ? (b(0) | (b(1) << 8U) | (b(2) << 16U) | (b(3) << 24U))
                         : (b(3) | (b(2) << 8U) | (b(1) << 16U) | (b(0) << 24U));
  };
  const auto sec = static_cast<std::int32_t>(u32_at(4));
  const std::uint32_t nanosec = u32_at(8);
  return static_cast<std::int64_t>(sec) * 1'000'000'000LL + nanosec;
}

HeaderedTopics classify_headered_topics(std::span<const io::TopicInfo> topics)
{
  HeaderedTopics result;
  // Per-type cache: nullopt = unresolvable, otherwise the classification.
  std::unordered_map<std::string, std::optional<bool>> by_type;
  for (const auto & t : topics) {
    auto it = by_type.find(t.type);
    if (it == by_type.end()) {
      std::optional<bool> headered;
      if (!t.schema_text.empty()) {
        headered = schema_leads_with_header(t.schema_text);
      } else {
        const auto resolved = core::resolve_message_definition(t.type);
        if (!resolved.text.empty()) {
          headered = schema_leads_with_header(resolved.text);
        } else {
          result.unresolved_types.push_back(t.type);
        }
      }
      it = by_type.emplace(t.type, headered).first;
    }
    if (it->second.value_or(false)) {
      result.topics.insert(t.name);
    }
  }
  return result;
}

}  // namespace bagwiz::commands
