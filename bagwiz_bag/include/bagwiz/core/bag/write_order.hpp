// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__BAG__WRITE_ORDER_HPP_
#define BAGWIZ__CORE__BAG__WRITE_ORDER_HPP_

#include "bagwiz/io/bag_io.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

// Keeping a bag's stored message order equal to its timestamp order.
//
// Commands that synthesize messages (traj join's trajectory, pcd concat's
// concatenated cloud) produce them at a different point in the stream than
// where they belong in time. Writing them where they are produced leaves rows
// whose storage position disagrees with their timestamp, and a consumer that
// reads in physical order rather than sorting — Foxglove's .db3 readers issue
// their message query with no `ORDER BY` — then receives them late.
//
// rosbag2's own reader sorts (`ORDER BY messages.timestamp, messages.id`), so
// `ros2 bag play` is unaffected either way. These decorators exist so bagwiz
// output does not depend on the consumer sorting.
//
// Both are `io::BagWriter` decorators rather than hooks inside the copy
// pipeline, so no existing `bag_copy_filtered` caller changes behaviour.
namespace bagwiz::core
{

// One buffered or synthesized message. Owns its payload, because a reader's
// span is only valid until its next message.
struct OrderedMessage
{
  std::string topic;
  std::int64_t timestamp_ns = 0;
  std::vector<std::byte> payload;
};

// Merges a set of messages that are known in full before the copy starts into
// a stream that is already in timestamp order.
//
// Each `write()` first emits every pending injected message stamped at or
// before the incoming message, so the combined output stays ordered; `close()`
// drains whatever remains (injections after the last copied message). Ties go
// to the injected message, matching the storage order of a bag whose
// synthesized rows were recorded first.
//
// The wrapped writer must outlive this object, and its topics must already be
// declared — this only orders `write()` calls.
class InjectingWriter final : public io::BagWriter
{
public:
  // `injected` is sorted by timestamp on construction; the caller need not
  // pre-sort it.
  InjectingWriter(io::BagWriter & inner, std::vector<OrderedMessage> injected);

  void declare_topic(const io::TopicInfo & topic) override;
  void write(
    std::string_view topic, std::int64_t timestamp_ns, std::span<const std::byte> payload) override;
  void close() override;

  // Injected messages emitted so far. Equals the constructor input's size once
  // close() has run.
  [[nodiscard]] std::uint64_t injected_count() const { return next_; }

private:
  // Emit every injected message stamped at or before `upto_ns`.
  void flush_upto(std::int64_t upto_ns);

  io::BagWriter & inner_;
  std::vector<OrderedMessage> injected_;
  std::size_t next_ = 0;
  bool closed_ = false;
};

// Restores timestamp order when a synthesized message's payload only becomes
// available *after* the point it belongs at. pcd concat stamps its output with
// the reference scan's capture time, but cannot assemble it until every
// contributing cloud has been read, which happens later in receive time.
//
// The caller reserves each synthesized timestamp up front (pcd concat plans
// every sync group before streaming, so it knows them all). Writes at or after
// the earliest unfilled reservation are buffered; delivering that reservation
// releases them. With no reservation earlier than the incoming message, writes
// pass straight through.
//
// The held volume is bounded by the reservation lag — the spread between a
// synthesized message's timestamp and the point its inputs complete — not by
// the bag size.
class ReorderWriter final : public io::BagWriter
{
public:
  // `reserved_ns` are the timestamps that will be delivered late, in any order.
  ReorderWriter(io::BagWriter & inner, std::vector<std::int64_t> reserved_ns);

  void declare_topic(const io::TopicInfo & topic) override;

  // A `timestamp_ns` equal to the earliest unfilled reservation is taken as
  // that reservation being delivered: it is written immediately and whatever it
  // was blocking is released.
  void write(
    std::string_view topic, std::int64_t timestamp_ns, std::span<const std::byte> payload) override;
  void close() override;

  // A reservation that will produce no message after all — pcd concat skips a
  // sync group whose concatenated payload came back empty. Advances past it so
  // whatever it was blocking is released now, instead of being held all the way
  // to close() and growing the buffer without bound.
  void drop_reservation();

  // Largest number of messages held at once, so a caller can log what the
  // ordering cost and a test can assert the buffer stays bounded.
  [[nodiscard]] std::size_t peak_buffered() const { return peak_buffered_; }

private:
  void release_ready();

  io::BagWriter & inner_;
  std::vector<std::int64_t> reserved_ns_;  // ascending; consumed from the front
  std::size_t next_reserved_ = 0;
  std::vector<OrderedMessage> buffered_;
  std::size_t peak_buffered_ = 0;
  bool closed_ = false;
};

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__BAG__WRITE_ORDER_HPP_
