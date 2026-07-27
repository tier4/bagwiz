// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/bag/write_order.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace
{

using bagwiz::core::InjectingWriter;
using bagwiz::core::OrderedMessage;
using bagwiz::core::ReorderWriter;

// Records what reached the real writer, in the order it arrived.
class RecordingWriter final : public bagwiz::io::BagWriter
{
public:
  struct Entry
  {
    std::string topic;
    std::int64_t timestamp_ns;
    std::byte first_byte;
  };

  void declare_topic(const bagwiz::io::TopicInfo & topic) override
  {
    declared.push_back(topic.name);
  }

  void write(
    std::string_view topic, std::int64_t timestamp_ns, std::span<const std::byte> payload) override
  {
    entries.push_back(
      {std::string(topic), timestamp_ns, payload.empty() ? std::byte{0} : payload[0]});
  }

  void close() override { ++closes; }

  std::vector<Entry> entries;
  std::vector<std::string> declared;
  int closes = 0;
};

std::vector<std::byte> tag(std::uint8_t id)
{
  return {std::byte{id}};
}

std::vector<std::int64_t> stamps(const RecordingWriter & w)
{
  std::vector<std::int64_t> out;
  out.reserve(w.entries.size());
  for (const auto & e : w.entries) {
    out.push_back(e.timestamp_ns);
  }
  return out;
}

std::vector<std::string> topics(const RecordingWriter & w)
{
  std::vector<std::string> out;
  out.reserve(w.entries.size());
  for (const auto & e : w.entries) {
    out.push_back(e.topic);
  }
  return out;
}

void expect_non_decreasing(const RecordingWriter & w)
{
  for (std::size_t i = 1; i < w.entries.size(); ++i) {
    EXPECT_LE(w.entries[i - 1].timestamp_ns, w.entries[i].timestamp_ns)
      << "entry " << i << " (" << w.entries[i].topic << ") is out of timestamp order";
  }
}

}  // namespace

TEST(InjectingWriterTest, MergesInjectedMessagesIntoTheCopyInTimeOrder)
{
  RecordingWriter inner;
  std::vector<OrderedMessage> injected = {
    {"/traj", 25, tag(2)},
    {"/traj", 5, tag(1)},  // deliberately unsorted: the writer sorts
    {"/traj", 45, tag(3)},
  };
  InjectingWriter w(inner, std::move(injected));

  for (const std::int64_t ts : {10, 20, 30, 40}) {
    w.write("/copy", ts, std::span<const std::byte>(tag(9)));
  }
  w.close();

  EXPECT_EQ(stamps(inner), (std::vector<std::int64_t>{5, 10, 20, 25, 30, 40, 45}));
  EXPECT_EQ(
    topics(inner),
    (std::vector<std::string>{"/traj", "/copy", "/copy", "/traj", "/copy", "/copy", "/traj"}));
  expect_non_decreasing(inner);
  EXPECT_EQ(w.injected_count(), 3U);
}

TEST(InjectingWriterTest, DrainsInjectionsPastTheLastCopiedMessage)
{
  // The tail case: injections stamped after everything in the bag would be
  // silently dropped if close() did not flush.
  RecordingWriter inner;
  InjectingWriter w(inner, {{"/traj", 100, tag(1)}, {"/traj", 200, tag(2)}});
  w.write("/copy", 10, std::span<const std::byte>(tag(9)));
  w.close();

  EXPECT_EQ(stamps(inner), (std::vector<std::int64_t>{10, 100, 200}));
  EXPECT_EQ(w.injected_count(), 2U);
}

TEST(InjectingWriterTest, InjectionLeadsOnATiedTimestamp)
{
  // tf static cp stamps its synthesized message at the bag's start time, which
  // ties with the first real message. The synthesized row leads, matching a bag
  // whose static TF was recorded first.
  RecordingWriter inner;
  InjectingWriter w(inner, {{"/tf_static", 10, tag(1)}});
  w.write("/copy", 10, std::span<const std::byte>(tag(9)));
  w.close();

  EXPECT_EQ(topics(inner), (std::vector<std::string>{"/tf_static", "/copy"}));
}

TEST(InjectingWriterTest, CloseIsIdempotentAndDoesNotCloseTheWrappedWriter)
{
  RecordingWriter inner;
  InjectingWriter w(inner, {{"/traj", 100, tag(1)}});
  w.close();
  w.close();

  EXPECT_EQ(inner.entries.size(), 1U) << "the tail injection must not be emitted twice";
  EXPECT_EQ(inner.closes, 0) << "the caller owns the wrapped writer's close()";
}

TEST(ReorderWriterTest, HoldsPassThroughUntilTheLateMessageArrives)
{
  // pcd concat's shape: the synthesized message is stamped at the reference
  // scan's capture time (100) but cannot be assembled until its last input has
  // been read (arriving after receive times 150/160/180).
  RecordingWriter inner;
  ReorderWriter w(inner, {100});

  w.write("/front", 150, std::span<const std::byte>(tag(1)));
  w.write("/other", 160, std::span<const std::byte>(tag(2)));
  w.write("/rear", 180, std::span<const std::byte>(tag(3)));
  w.write("/concat", 100, std::span<const std::byte>(tag(4)));  // the reservation
  w.close();

  EXPECT_EQ(stamps(inner), (std::vector<std::int64_t>{100, 150, 160, 180}));
  EXPECT_EQ(topics(inner), (std::vector<std::string>{"/concat", "/front", "/other", "/rear"}));
  expect_non_decreasing(inner);
}

TEST(ReorderWriterTest, WritesStraightThroughWhenNothingIsOutstanding)
{
  // The common path must not buffer: with no reservation earlier than the
  // incoming message there is nothing to wait for, so no payload is copied.
  RecordingWriter inner;
  ReorderWriter w(inner, {1000});

  for (const std::int64_t ts : {10, 20, 30}) {
    w.write("/copy", ts, std::span<const std::byte>(tag(9)));
  }
  EXPECT_EQ(inner.entries.size(), 3U) << "messages before the reservation must not be held";
  EXPECT_EQ(w.peak_buffered(), 0U);

  w.write("/concat", 1000, std::span<const std::byte>(tag(1)));
  w.close();
  expect_non_decreasing(inner);
}

TEST(ReorderWriterTest, HandlesSeveralReservationsInSequence)
{
  RecordingWriter inner;
  ReorderWriter w(inner, {100, 200});

  w.write("/in", 150, std::span<const std::byte>(tag(1)));
  w.write("/concat", 100, std::span<const std::byte>(tag(2)));
  w.write("/in", 250, std::span<const std::byte>(tag(3)));
  w.write("/concat", 200, std::span<const std::byte>(tag(4)));
  w.close();

  EXPECT_EQ(stamps(inner), (std::vector<std::int64_t>{100, 150, 200, 250}));
  expect_non_decreasing(inner);
  EXPECT_LE(w.peak_buffered(), 1U) << "only the messages spanning one reservation are held";
}

TEST(ReorderWriterTest, FlushesBufferWhenAReservationNeverArrives)
{
  // A planned group can end up producing nothing (an empty payload is skipped
  // by the caller). The buffer must still drain rather than swallow messages.
  RecordingWriter inner;
  ReorderWriter w(inner, {100});

  w.write("/in", 150, std::span<const std::byte>(tag(1)));
  w.write("/in", 160, std::span<const std::byte>(tag(2)));
  w.close();

  EXPECT_EQ(stamps(inner), (std::vector<std::int64_t>{150, 160}));
}

TEST(ReorderWriterTest, BufferedPayloadsSurviveTheirSource)
{
  // Reader spans are only valid until the next message, so a held message must
  // own its bytes. Overwrite the source buffer before releasing.
  RecordingWriter inner;
  ReorderWriter w(inner, {100});

  std::vector<std::byte> scratch = tag(7);
  w.write("/in", 150, std::span<const std::byte>(scratch));
  scratch[0] = std::byte{99};  // the reader moved on

  w.write("/concat", 100, std::span<const std::byte>(tag(1)));
  w.close();

  ASSERT_EQ(inner.entries.size(), 2U);
  EXPECT_EQ(inner.entries[1].topic, "/in");
  EXPECT_EQ(inner.entries[1].first_byte, std::byte{7}) << "the buffered payload was not copied";
}
