// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tui/width.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace bagwiz::core::tui
{

namespace
{

// One parsed token: a codepoint, a CSI escape, an unrecognised control
// byte, or an incomplete tail that should be silently dropped.
struct Token
{
  enum class Kind {
    kCodepoint,  // a single UTF-8 codepoint
    kCsi,        // a complete CSI escape sequence (ESC [ ... final)
    kControl,    // a non-CSI control byte (width 0, kept verbatim)
    kIncomplete  // a partial CSI / partial UTF-8 at end of input
  };
  Kind kind = Kind::kIncomplete;
  std::size_t byte_len = 0;     // bytes consumed from the input
  std::uint32_t codepoint = 0;  // only valid when kind == kCodepoint
  int width = 0;
};

constexpr std::uint8_t kUtf8ContinuationMask = 0xC0;
constexpr std::uint8_t kUtf8ContinuationPrefix = 0x80;

bool is_cjk_wide(std::uint32_t cp) noexcept
{
  // Conservative ranges. Documented in width.hpp; future work can swap
  // this for the East-Asian Width table.
  return (cp >= 0x1100U && cp <= 0x115FU) || (cp >= 0x2E80U && cp <= 0x303EU) ||
         (cp >= 0x3041U && cp <= 0x33FFU) || (cp >= 0x3400U && cp <= 0x4DBFU) ||
         (cp >= 0x4E00U && cp <= 0x9FFFU) || (cp >= 0xA000U && cp <= 0xA4CFU) ||
         (cp >= 0xAC00U && cp <= 0xD7A3U) || (cp >= 0xF900U && cp <= 0xFAFFU) ||
         (cp >= 0xFE30U && cp <= 0xFE4FU) || (cp >= 0xFF00U && cp <= 0xFF60U) ||
         (cp >= 0xFFE0U && cp <= 0xFFE6U);
}

Token make_control(std::size_t byte_len) noexcept
{
  Token tok;
  tok.kind = Token::Kind::kControl;
  tok.byte_len = byte_len;
  tok.width = 0;
  return tok;
}

// CSI escape: ESC '[' ... <final 0x40..0x7E>. We accept the broadest
// VT100/ECMA-48 framing so SGR, cursor positioning, mode toggles, etc.
// all parse as a single zero-width unit.
Token parse_csi_escape(std::string_view s, std::size_t offset) noexcept
{
  Token tok;
  std::size_t i = offset + 2;
  while (i < s.size()) {
    const auto b = static_cast<std::uint8_t>(s[i]);
    if (b >= 0x40U && b <= 0x7EU) {
      tok.kind = Token::Kind::kCsi;
      tok.byte_len = (i - offset) + 1;
      tok.width = 0;
      return tok;
    }
    ++i;
  }
  tok.kind = Token::Kind::kIncomplete;
  tok.byte_len = s.size() - offset;
  return tok;
}

// Decode a multi-byte UTF-8 codepoint starting at `s[offset]`. Returns
// kControl(1) for stray continuation bytes or invalid lead bytes,
// kIncomplete for truncated tails, and kCodepoint otherwise.
Token parse_utf8_multibyte(std::string_view s, std::size_t offset, std::uint8_t first) noexcept
{
  std::size_t needed = 0;
  std::uint32_t cp = 0;
  if ((first & 0xE0U) == 0xC0U) {
    needed = 2;
    cp = first & 0x1FU;
  } else if ((first & 0xF0U) == 0xE0U) {
    needed = 3;
    cp = first & 0x0FU;
  } else if ((first & 0xF8U) == 0xF0U) {
    needed = 4;
    cp = first & 0x07U;
  } else {
    return make_control(1);
  }
  if (offset + needed > s.size()) {
    Token tok;
    tok.kind = Token::Kind::kIncomplete;
    tok.byte_len = s.size() - offset;
    return tok;
  }
  for (std::size_t k = 1; k < needed; ++k) {
    const auto b = static_cast<std::uint8_t>(s[offset + k]);
    if ((b & kUtf8ContinuationMask) != kUtf8ContinuationPrefix) {
      return make_control(1);
    }
    cp = (cp << 6U) | (b & 0x3FU);
  }
  Token tok;
  tok.kind = Token::Kind::kCodepoint;
  tok.byte_len = needed;
  tok.codepoint = cp;
  tok.width = is_cjk_wide(cp) ? 2 : 1;
  return tok;
}

// Parse one token starting at `s[offset]`. Never reads past `s.size()`.
// `byte_len == 0` only when offset is already at end (returns kIncomplete).
Token next_token(std::string_view s, std::size_t offset) noexcept
{
  if (offset >= s.size()) {
    return Token{};  // kIncomplete with byte_len 0
  }
  const auto first = static_cast<std::uint8_t>(s[offset]);

  if (first == 0x1B) {
    if (offset + 1 < s.size() && s[offset + 1] == '[') {
      return parse_csi_escape(s, offset);
    }
    return make_control(1);  // lone ESC
  }

  if (first < 0x80U) {
    if (first < 0x20U || first == 0x7FU) {
      return make_control(1);
    }
    Token tok;
    tok.kind = Token::Kind::kCodepoint;
    tok.byte_len = 1;
    tok.codepoint = first;
    tok.width = 1;
    return tok;
  }

  return parse_utf8_multibyte(s, offset, first);
}

}  // namespace

int display_width(std::string_view s) noexcept
{
  int width = 0;
  std::size_t i = 0;
  while (i < s.size()) {
    const Token tok = next_token(s, i);
    if (tok.byte_len == 0) {
      break;  // defensive
    }
    width += tok.width;
    i += tok.byte_len;
  }
  return width;
}

std::string truncate_to_width(std::string_view s, int max_cols)
{
  if (max_cols <= 0 || s.empty()) {
    return {};
  }
  std::string out;
  out.reserve(s.size());
  int used = 0;
  std::size_t i = 0;
  while (i < s.size()) {
    const Token tok = next_token(s, i);
    if (tok.byte_len == 0) {
      break;
    }
    if (tok.kind == Token::Kind::kIncomplete) {
      // Drop incomplete trailing CSI / UTF-8 sequences silently.
      break;
    }
    if (tok.kind == Token::Kind::kCodepoint && used + tok.width > max_cols) {
      // Including this codepoint would overflow. Stop without partial.
      break;
    }
    out.append(s.data() + i, tok.byte_len);
    used += tok.width;
    i += tok.byte_len;
  }
  return out;
}

std::vector<std::string> wrap_to_width(std::string_view s, int max_cols)
{
  std::vector<std::string> out;
  if (max_cols <= 0) {
    out.emplace_back(s);
    return out;
  }
  if (s.empty()) {
    out.emplace_back();
    return out;
  }

  // Leading ASCII whitespace is reused as the continuation prefix so
  // wrapped YAML keeps its visual nesting. Drop it only when it would
  // leave zero columns for content; otherwise narrow but non-empty
  // continuations are preferred to losing indent.
  std::size_t indent_end = 0;
  while (indent_end < s.size() && (s[indent_end] == ' ' || s[indent_end] == '\t')) {
    ++indent_end;
  }
  std::string indent(s.substr(0, indent_end));
  int indent_w = display_width(indent);
  if (indent_w >= max_cols) {
    indent.clear();
    indent_w = 0;
  }

  std::string current;
  current.reserve(s.size());
  int used = 0;

  std::size_t i = 0;
  while (i < s.size()) {
    const Token tok = next_token(s, i);
    if (tok.byte_len == 0 || tok.kind == Token::Kind::kIncomplete) {
      // Drop incomplete trailing CSI / UTF-8 sequences silently.
      break;
    }
    if (tok.kind != Token::Kind::kCodepoint) {
      // CSI / control bytes are zero-width and attach without changing
      // the wrap accounting.
      current.append(s.data() + i, tok.byte_len);
      i += tok.byte_len;
      continue;
    }
    // Only flush when the current segment already has non-indent
    // content; otherwise an over-long codepoint that cannot fit after
    // the indent would loop forever.
    if (used + tok.width > max_cols && used > indent_w) {
      out.push_back(std::move(current));
      current.clear();
      current.append(indent);
      used = indent_w;
    }
    current.append(s.data() + i, tok.byte_len);
    used += tok.width;
    i += tok.byte_len;
  }
  out.push_back(std::move(current));
  return out;
}

}  // namespace bagwiz::core::tui
