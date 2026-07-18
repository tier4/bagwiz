// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "walk_cursor.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <algorithm>
#include <iterator>
#include <utility>

namespace bagwiz::commands
{

namespace
{

constexpr int64_t kOneSecondNs = 1'000'000'000;
constexpr int64_t kTenSecondNs = 10 * kOneSecondNs;

}  // namespace

std::vector<std::byte> copy_payload(std::span<const std::byte> src)
{
  return std::vector<std::byte>(src.begin(), src.end());
}

bool MessageCursor::load_next()
{
  if (exhausted_) {
    return false;
  }
  OwnedMessage msg;
  if (!source_(msg)) {
    exhausted_ = true;
    return false;
  }
  cache_.push_back(std::move(msg));
  return true;
}

bool MessageCursor::navigate(MsgNav move)
{
  status_.clear();
  const std::size_t before = index_;
  switch (move) {
    case MsgNav::kNext:
      if (index_ + 1 < cache_.size()) {
        ++index_;
      } else if (load_next()) {
        index_ = cache_.size() - 1;
      } else {
        index_ = 0;
        status_ = "(wrapped to first)";
      }
      break;
    case MsgNav::kPrev:
      if (index_ > 0) {
        --index_;
      } else {
        status_ = "(at first message)";
      }
      break;
    case MsgNav::kFirst:
      index_ = 0;
      break;
    case MsgNav::kLast: {
      std::size_t loaded = 0;
      while (load_next()) {
        ++loaded;
      }
      index_ = cache_.size() - 1;
      if (loaded == 0 && exhausted_) {
        status_ = "(already at last message)";
      }
      break;
    }
    case MsgNav::kStepForward1s:
    case MsgNav::kStepForward10s: {
      const int64_t delta_ns = move == MsgNav::kStepForward10s ? kTenSecondNs : kOneSecondNs;
      const int64_t target_ns = cache_[index_].timestamp_ns + delta_ns;
      if (exhausted_ && cache_.back().timestamp_ns < target_ns) {
        if (index_ == cache_.size() - 1) {
          status_ = "(already at last message)";
        } else {
          index_ = cache_.size() - 1;
          status_ = "(reached end)";
        }
        break;
      }
      while (!exhausted_ && cache_.back().timestamp_ns < target_ns) {
        load_next();
      }
      auto it = std::lower_bound(
        cache_.begin(), cache_.end(), target_ns,
        [](const OwnedMessage & m, int64_t t) { return m.timestamp_ns < t; });
      if (it != cache_.end()) {
        index_ = static_cast<std::size_t>(std::distance(cache_.begin(), it));
      } else {
        index_ = cache_.size() - 1;
        status_ = "(reached end)";
      }
      break;
    }
    case MsgNav::kStepBackward1s:
    case MsgNav::kStepBackward10s: {
      const int64_t delta_ns = move == MsgNav::kStepBackward10s ? kTenSecondNs : kOneSecondNs;
      const int64_t target_ns = cache_[index_].timestamp_ns - delta_ns;
      if (target_ns <= cache_.front().timestamp_ns) {
        if (index_ == 0) {
          status_ = "(at first message)";
        } else {
          index_ = 0;
        }
        break;
      }
      auto it = std::upper_bound(
        cache_.begin(), cache_.end(), target_ns,
        [](int64_t t, const OwnedMessage & m) { return t < m.timestamp_ns; });
      // it is the first message strictly after target_ns; step back one
      // to land on the last message at or before target_ns.
      --it;
      index_ = static_cast<std::size_t>(std::distance(cache_.begin(), it));
      break;
    }
  }
  return index_ != before;
}

}  // namespace bagwiz::commands
