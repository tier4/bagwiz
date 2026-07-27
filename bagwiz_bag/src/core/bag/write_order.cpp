// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/bag/write_order.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace bagwiz::core
{

namespace
{

// Copy a payload span into an owned buffer. Reader spans stay valid only until
// the next message, so anything held past the current write must be copied.
std::vector<std::byte> own(std::span<const std::byte> payload)
{
  return {payload.begin(), payload.end()};
}

}  // namespace

// ---------------------------------------------------------------------------
// InjectingWriter
// ---------------------------------------------------------------------------

InjectingWriter::InjectingWriter(io::BagWriter & inner, std::vector<OrderedMessage> injected)
: inner_(inner), injected_(std::move(injected))
{
  // Stable sort so injections sharing a timestamp keep the caller's order —
  // for traj join that is trajectory order, which is what a reader should see.
  std::stable_sort(injected_.begin(), injected_.end(), [](const auto & a, const auto & b) {
    return a.timestamp_ns < b.timestamp_ns;
  });
}

void InjectingWriter::declare_topic(const io::TopicInfo & topic)
{
  inner_.declare_topic(topic);
}

void InjectingWriter::flush_upto(std::int64_t upto_ns)
{
  while (next_ < injected_.size() && injected_[next_].timestamp_ns <= upto_ns) {
    const auto & m = injected_[next_];
    inner_.write(m.topic, m.timestamp_ns, std::span<const std::byte>(m.payload));
    ++next_;
  }
}

void InjectingWriter::write(
  std::string_view topic, std::int64_t timestamp_ns, std::span<const std::byte> payload)
{
  flush_upto(timestamp_ns);
  inner_.write(topic, timestamp_ns, payload);
}

void InjectingWriter::close()
{
  if (closed_) {
    return;
  }
  // Anything stamped after the last copied message still has to land.
  flush_upto(std::numeric_limits<std::int64_t>::max());
  closed_ = true;
  // Closing the wrapped writer stays the caller's job: it owns the writer and
  // may need its close() error separately (see io::close_writer_or_log).
}

// ---------------------------------------------------------------------------
// ReorderWriter
// ---------------------------------------------------------------------------

ReorderWriter::ReorderWriter(io::BagWriter & inner, std::vector<std::int64_t> reserved_ns)
: inner_(inner), reserved_ns_(std::move(reserved_ns))
{
  std::sort(reserved_ns_.begin(), reserved_ns_.end());
}

void ReorderWriter::declare_topic(const io::TopicInfo & topic)
{
  inner_.declare_topic(topic);
}

// Emit buffered messages nothing is waiting on any more: everything stamped at
// or before the earliest still-unfilled reservation.
void ReorderWriter::release_ready()
{
  const std::int64_t barrier = next_reserved_ < reserved_ns_.size()
                                 ? reserved_ns_[next_reserved_]
                                 : std::numeric_limits<std::int64_t>::max();
  // Buffered messages arrive in timestamp order (the out-of-order ones are the
  // reserved stamps, which are written straight through), so a stable sort is
  // a cheap guard that also keeps ties in arrival order.
  std::stable_sort(buffered_.begin(), buffered_.end(), [](const auto & a, const auto & b) {
    return a.timestamp_ns < b.timestamp_ns;
  });
  auto it = buffered_.begin();
  for (; it != buffered_.end() && it->timestamp_ns <= barrier; ++it) {
    inner_.write(it->topic, it->timestamp_ns, std::span<const std::byte>(it->payload));
  }
  buffered_.erase(buffered_.begin(), it);
}

void ReorderWriter::write(
  std::string_view topic, std::int64_t timestamp_ns, std::span<const std::byte> payload)
{
  const bool has_reservation = next_reserved_ < reserved_ns_.size();

  // The reservation being delivered. Write it now — it is what the buffer was
  // waiting on — then let everything it was blocking go.
  if (has_reservation && timestamp_ns == reserved_ns_[next_reserved_]) {
    inner_.write(topic, timestamp_ns, payload);
    ++next_reserved_;
    release_ready();
    return;
  }

  // Nothing outstanding can still land before this message, so it is already in
  // order. This is the common path: with no reservations left, or the next one
  // still ahead of us, nothing is buffered and nothing is copied.
  if (!has_reservation || timestamp_ns < reserved_ns_[next_reserved_]) {
    inner_.write(topic, timestamp_ns, payload);
    return;
  }

  buffered_.push_back(OrderedMessage{std::string(topic), timestamp_ns, own(payload)});
  peak_buffered_ = std::max(peak_buffered_, buffered_.size());
}

void ReorderWriter::drop_reservation()
{
  if (next_reserved_ < reserved_ns_.size()) {
    ++next_reserved_;
    release_ready();
  }
}

void ReorderWriter::close()
{
  if (closed_) {
    return;
  }
  // A reservation that never arrived (a group that produced no payload, say)
  // must not strand the buffer.
  next_reserved_ = reserved_ns_.size();
  release_ready();
  closed_ = true;
}

}  // namespace bagwiz::core
