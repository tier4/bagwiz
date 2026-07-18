// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__WALK_CURSOR_HPP_
#define COMMANDS__WALK_CURSOR_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <span>
#include <string>
#include <vector>

// Lazy message cache + navigation cursor of `bagwiz walk`, split out of
// walk.cpp so the move semantics (wrap-around, boundary notices, the ~1s/~10s
// time steps) can be unit-tested without a TTY. CLI-internal: this header
// lives with the command sources and is not installed.
namespace bagwiz::commands
{

// Message-cursor moves shared by the YAML view and the image preview, so
// wrap-around, "at first message", and G's full-scan behave identically in both.
enum class MsgNav {
  kNext,
  kPrev,
  kFirst,
  kLast,
  kStepForward1s,
  kStepBackward1s,
  kStepForward10s,
  kStepBackward10s,
};

// Cached owning copy of a single bag message. RawMessage's span is
// invalidated by the next BagReader::next() call, so walk must take a
// copy to allow backward navigation.
struct OwnedMessage
{
  int64_t timestamp_ns = 0;
  std::vector<std::byte> payload;
};

std::vector<std::byte> copy_payload(std::span<const std::byte> src);

// Owns the lazy message cache (every message seen so far), the exhausted
// flag, and the cursor index. The YAML view and the image preview share one
// cursor so both navigate identically. The cache grows on demand: `prev`
// stays O(1) for anything already seen and kLast is the only move that forces
// a full-remaining scan.
class MessageCursor
{
public:
  // Pulls the next bag message into `msg`; returns false at EOF or on a read
  // error (the source is expected to log read errors itself).
  using Source = std::function<bool(OwnedMessage &)>;

  // `status` is shared with the surrounding UI: navigate() clears it on entry
  // and sets the boundary notices ("(wrapped to first)", ...) into it.
  MessageCursor(Source source, std::string & status) : source_(std::move(source)), status_(status)
  {
  }

  // Load one more message into the cache. Returns false once the source is
  // exhausted (or on read error); further calls are cheap no-ops.
  bool load_next();

  // Move the cursor. Returns whether the cursor actually moved, so callers
  // can skip work that only matters on a real move (resetting the pager
  // scroll offset; re-decoding the preview frame). Callers must have loaded
  // at least one message first (walk does so before the pager starts).
  bool navigate(MsgNav move);

  [[nodiscard]] std::size_t index() const noexcept { return index_; }
  [[nodiscard]] const std::vector<OwnedMessage> & cache() const noexcept { return cache_; }
  [[nodiscard]] bool exhausted() const noexcept { return exhausted_; }

private:
  Source source_;
  std::string & status_;
  std::vector<OwnedMessage> cache_;
  bool exhausted_ = false;
  std::size_t index_ = 0;
};

}  // namespace bagwiz::commands

#endif  // COMMANDS__WALK_CURSOR_HPP_
