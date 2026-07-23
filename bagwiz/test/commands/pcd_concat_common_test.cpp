// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "pcd_concat_common.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{

using bagwiz::commands::ConcatAssembler;
using bagwiz::commands::ConcatCounterMerger;
using bagwiz::commands::ConcatGroupJob;
using bagwiz::commands::ConcatGroupTracker;
using bagwiz::commands::ConcatPickOutcome;
using bagwiz::commands::parse_stamp_offsets;
using bagwiz::commands::process_concat_group;
using bagwiz::commands::TopicState;
namespace pc = bagwiz::core::pointcloud;

constexpr const char * kLogger = "bagwiz.test.pcd_concat_common";

// ---------------------------------------------------------------------------
// parse_stamp_offsets
// ---------------------------------------------------------------------------

TEST(ParseStampOffsets, NoEntriesYieldZeroOffsets)
{
  const std::unordered_map<std::string, std::size_t> index{{"/a", 0}, {"/b", 1}};
  const auto offsets = parse_stamp_offsets({}, index, kLogger);
  ASSERT_TRUE(offsets.has_value());
  EXPECT_EQ(*offsets, (std::vector<std::int64_t>{0, 0}));
}

TEST(ParseStampOffsets, EntriesLandInTheirTopicSlots)
{
  const std::unordered_map<std::string, std::size_t> index{{"/a", 0}, {"/b", 1}, {"/c", 2}};
  const auto offsets = parse_stamp_offsets({"/b=50ms", "/a=-500ns"}, index, kLogger);
  ASSERT_TRUE(offsets.has_value());
  EXPECT_EQ(*offsets, (std::vector<std::int64_t>{-500, 50'000'000, 0}));
}

TEST(ParseStampOffsets, MissingEqualsIsRejected)
{
  const std::unordered_map<std::string, std::size_t> index{{"/a", 0}, {"/b", 1}};
  EXPECT_EQ(parse_stamp_offsets({"/a50ms"}, index, kLogger), std::nullopt);
}

TEST(ParseStampOffsets, TopicOutsidePcdListIsRejected)
{
  const std::unordered_map<std::string, std::size_t> index{{"/a", 0}, {"/b", 1}};
  EXPECT_EQ(parse_stamp_offsets({"/nope=50ms"}, index, kLogger), std::nullopt);
}

TEST(ParseStampOffsets, UnparseableValueIsRejected)
{
  const std::unordered_map<std::string, std::size_t> index{{"/a", 0}, {"/b", 1}};
  EXPECT_EQ(parse_stamp_offsets({"/a=soon"}, index, kLogger), std::nullopt);
}

// ---------------------------------------------------------------------------
// ConcatAssembler
// ---------------------------------------------------------------------------

// One input topic with `message_count` messages and an identity extrinsic.
TopicState input(std::size_t message_count)
{
  TopicState ts;
  ts.stamps_ns.assign(message_count, 0);
  return ts;
}

// A base_link-framed xyz float32 cloud (point_step 12) with one point at x.
std::vector<std::byte> xyz_payload(std::int64_t stamp_ns, float x)
{
  pc::PointCloud2 c;
  c.timestamp_ns = stamp_ns;
  c.frame_id = "base_link";
  c.height = 1;
  c.width = 1;
  c.fields = {
    {"x", 0, pc::PointFieldType::kFloat32, 1},
    {"y", 4, pc::PointFieldType::kFloat32, 1},
    {"z", 8, pc::PointFieldType::kFloat32, 1},
  };
  c.point_step = 12;
  c.row_step = 12;
  c.is_dense = true;
  c.data.assign(12, std::byte{0});
  std::memcpy(c.data.data(), &x, sizeof(float));
  return pc::serialize_pointcloud2(c);
}

// An xyz+intensity cloud: transforms fine, but its layout cannot concatenate
// with an xyz-only cloud.
std::vector<std::byte> xyzi_payload(std::int64_t stamp_ns)
{
  pc::PointCloud2 c;
  c.timestamp_ns = stamp_ns;
  c.frame_id = "base_link";
  c.height = 1;
  c.width = 1;
  c.fields = {
    {"x", 0, pc::PointFieldType::kFloat32, 1},
    {"y", 4, pc::PointFieldType::kFloat32, 1},
    {"z", 8, pc::PointFieldType::kFloat32, 1},
    {"intensity", 12, pc::PointFieldType::kFloat32, 1},
  };
  c.point_step = 16;
  c.row_step = 16;
  c.is_dense = true;
  c.data.assign(16, std::byte{0});
  return pc::serialize_pointcloud2(c);
}

// A cloud without a z field: parses fine but fails the extrinsic transform.
std::vector<std::byte> xy_payload(std::int64_t stamp_ns)
{
  pc::PointCloud2 c;
  c.timestamp_ns = stamp_ns;
  c.frame_id = "base_link";
  c.height = 1;
  c.width = 1;
  c.fields = {
    {"x", 0, pc::PointFieldType::kFloat32, 1},
    {"y", 4, pc::PointFieldType::kFloat32, 1},
  };
  c.point_step = 8;
  c.row_step = 8;
  c.is_dense = true;
  c.data.assign(8, std::byte{0});
  return pc::serialize_pointcloud2(c);
}

// Not a PointCloud2 payload at all.
std::vector<std::byte> garbage_payload()
{
  return {std::byte{0}, std::byte{1}, std::byte{2}, std::byte{3}};
}

TEST(ConcatAssembler, FiresGroupWhenLastPickArrives)
{
  ConcatAssembler a({input(1), input(1)}, {{1000, {0, 0}}}, "base_link");

  const auto waiting = a.on_message(0, 0, xyz_payload(1000, 1.0F));
  EXPECT_TRUE(waiting.fired.empty());
  EXPECT_TRUE(waiting.error.empty());

  const auto done = a.on_message(1, 0, xyz_payload(1000, 2.0F));
  EXPECT_TRUE(done.error.empty());
  ASSERT_EQ(done.fired.size(), 1u);
  EXPECT_EQ(done.fired[0].stamp_ns, 1000);

  const auto merged = pc::parse_pointcloud2(done.fired[0].payload);
  ASSERT_TRUE(merged.ok());
  EXPECT_EQ(merged.cloud->width, 2u);  // both one-point clouds joined
  EXPECT_EQ(merged.cloud->timestamp_ns, 1000);
  EXPECT_EQ(merged.cloud->frame_id, "base_link");

  EXPECT_EQ(a.counters().written_groups, 1);
  EXPECT_EQ(a.counters().partial_groups, 0);
  EXPECT_EQ(a.counters().matched, (std::vector<std::int64_t>{1, 1}));
  EXPECT_EQ(a.counters().parse_fail, (std::vector<std::int64_t>{0, 0}));
  EXPECT_EQ(a.counters().transform_fail, (std::vector<std::int64_t>{0, 0}));
}

TEST(ConcatAssembler, ArrivalOrderDoesNotMatter)
{
  ConcatAssembler a({input(1), input(1)}, {{1000, {0, 0}}}, "base_link");

  EXPECT_TRUE(a.on_message(1, 0, xyz_payload(1000, 2.0F)).fired.empty());
  const auto done = a.on_message(0, 0, xyz_payload(1000, 1.0F));
  ASSERT_EQ(done.fired.size(), 1u);
  EXPECT_EQ(done.fired[0].stamp_ns, 1000);
}

TEST(ConcatAssembler, UnpickedMessageIsNotEvenParsed)
{
  // Two reference messages but only one group: reference message 1 is unpicked.
  ConcatAssembler a({input(2), input(1)}, {{1000, {0, 0}}}, "base_link");

  // Garbage bytes at the unpicked index must not count as a parse failure.
  const auto ignored = a.on_message(0, 1, garbage_payload());
  EXPECT_TRUE(ignored.fired.empty());
  EXPECT_EQ(a.counters().parse_fail, (std::vector<std::int64_t>{0, 0}));

  EXPECT_TRUE(a.on_message(1, 0, xyz_payload(1000, 2.0F)).fired.empty());
  const auto done = a.on_message(0, 0, xyz_payload(1000, 1.0F));
  ASSERT_EQ(done.fired.size(), 1u);
  EXPECT_EQ(a.counters().matched, (std::vector<std::int64_t>{1, 1}));
}

TEST(ConcatAssembler, ParseFailureLeavesGroupPartial)
{
  ConcatAssembler a({input(1), input(1)}, {{1000, {0, 0}}}, "base_link");

  EXPECT_TRUE(a.on_message(0, 0, garbage_payload()).fired.empty());
  const auto done = a.on_message(1, 0, xyz_payload(1000, 2.0F));
  EXPECT_TRUE(done.error.empty());
  ASSERT_EQ(done.fired.size(), 1u);  // still fires, with the surviving pick only

  const auto merged = pc::parse_pointcloud2(done.fired[0].payload);
  ASSERT_TRUE(merged.ok());
  EXPECT_EQ(merged.cloud->width, 1u);

  EXPECT_EQ(a.counters().written_groups, 1);
  EXPECT_EQ(a.counters().partial_groups, 1);
  EXPECT_EQ(a.counters().matched, (std::vector<std::int64_t>{0, 1}));
  EXPECT_EQ(a.counters().parse_fail, (std::vector<std::int64_t>{1, 0}));
}

TEST(ConcatAssembler, AllPicksFailedEmitsNothing)
{
  ConcatAssembler a({input(1), input(1)}, {{1000, {0, 0}}}, "base_link");

  EXPECT_TRUE(a.on_message(0, 0, garbage_payload()).fired.empty());
  const auto done = a.on_message(1, 0, garbage_payload());
  EXPECT_TRUE(done.error.empty());
  EXPECT_TRUE(done.fired.empty());  // nothing survived to concatenate

  EXPECT_EQ(a.counters().written_groups, 0);
  EXPECT_EQ(a.counters().partial_groups, 1);
  EXPECT_EQ(a.counters().matched, (std::vector<std::int64_t>{0, 0}));
  EXPECT_EQ(a.counters().parse_fail, (std::vector<std::int64_t>{1, 1}));
}

TEST(ConcatAssembler, SharedPickServesEveryReferencingGroup)
{
  // Topic 1's only message is picked by BOTH reference-driven groups: it must
  // stay cached until the second group fires (the refcount's whole point).
  ConcatAssembler a({input(2), input(1)}, {{1000, {0, 0}}, {2000, {1, 0}}}, "base_link");

  EXPECT_TRUE(a.on_message(1, 0, xyz_payload(1000, 2.0F)).fired.empty());

  const auto first = a.on_message(0, 0, xyz_payload(1000, 1.0F));
  ASSERT_EQ(first.fired.size(), 1u);
  EXPECT_EQ(first.fired[0].stamp_ns, 1000);

  const auto second = a.on_message(0, 1, xyz_payload(2000, 1.0F));
  ASSERT_EQ(second.fired.size(), 1u);
  EXPECT_EQ(second.fired[0].stamp_ns, 2000);
  const auto merged = pc::parse_pointcloud2(second.fired[0].payload);
  ASSERT_TRUE(merged.ok());
  EXPECT_EQ(merged.cloud->width, 2u);  // the shared pick was still cached

  EXPECT_EQ(a.counters().written_groups, 2);
  EXPECT_EQ(a.counters().partial_groups, 0);
  EXPECT_EQ(a.counters().matched, (std::vector<std::int64_t>{2, 2}));
}

TEST(ConcatAssembler, TransformFailureLeavesGroupPartial)
{
  ConcatAssembler a({input(1), input(1)}, {{1000, {0, 0}}}, "base_link");

  EXPECT_TRUE(a.on_message(0, 0, xy_payload(1000)).fired.empty());  // no z field
  const auto done = a.on_message(1, 0, xyz_payload(1000, 2.0F));
  ASSERT_EQ(done.fired.size(), 1u);

  EXPECT_EQ(a.counters().written_groups, 1);
  EXPECT_EQ(a.counters().partial_groups, 1);
  EXPECT_EQ(a.counters().matched, (std::vector<std::int64_t>{0, 1}));
  EXPECT_EQ(a.counters().parse_fail, (std::vector<std::int64_t>{0, 0}));
  EXPECT_EQ(a.counters().transform_fail, (std::vector<std::int64_t>{1, 0}));
}

TEST(ConcatAssembler, LayoutMismatchFailsTheConcat)
{
  ConcatAssembler a({input(1), input(1)}, {{1000, {0, 0}}}, "base_link");

  EXPECT_TRUE(a.on_message(0, 0, xyz_payload(1000, 1.0F)).fired.empty());
  const auto done = a.on_message(1, 0, xyzi_payload(1000));  // extra intensity field
  EXPECT_FALSE(done.error.empty());
  EXPECT_TRUE(done.fired.empty());

  // Both picks were gathered before concat_clouds rejected the layout.
  EXPECT_EQ(a.counters().matched, (std::vector<std::int64_t>{1, 1}));
  EXPECT_EQ(a.counters().written_groups, 0);
  EXPECT_EQ(a.counters().partial_groups, 0);
}

// ---------------------------------------------------------------------------
// ConcatGroupTracker (parallel Pass B: matching-only arrival bookkeeping)
// ---------------------------------------------------------------------------

TEST(ConcatGroupTracker, FiresJobWhenLastPickArrives)
{
  ConcatGroupTracker t({input(1), input(1)}, {{1000, {0, 0}}});

  const auto payload_a = xyz_payload(1000, 1.0F);
  EXPECT_TRUE(t.on_message(0, 0, payload_a).empty());

  const auto payload_b = xyz_payload(1000, 2.0F);
  const auto jobs = t.on_message(1, 0, payload_b);
  ASSERT_EQ(jobs.size(), 1u);
  EXPECT_EQ(jobs[0].output_stamp_ns, 1000);
  ASSERT_EQ(jobs[0].picks.size(), 2u);
  EXPECT_EQ(jobs[0].picks[0].topic, 0u);
  EXPECT_EQ(jobs[0].picks[1].topic, 1u);
  ASSERT_NE(jobs[0].picks[0].payload, nullptr);
  ASSERT_NE(jobs[0].picks[1].payload, nullptr);
  EXPECT_EQ(*jobs[0].picks[0].payload, payload_a);  // raw bytes intact, --pcd order
  EXPECT_EQ(*jobs[0].picks[1].payload, payload_b);
}

TEST(ConcatGroupTracker, UnpickedMessageIsIgnored)
{
  // Two reference messages but only one group: reference message 1 is unpicked.
  ConcatGroupTracker t({input(2), input(1)}, {{1000, {0, 0}}});

  EXPECT_TRUE(t.on_message(0, 1, garbage_payload()).empty());
  EXPECT_TRUE(t.on_message(1, 0, xyz_payload(1000, 2.0F)).empty());
  EXPECT_EQ(t.on_message(0, 0, xyz_payload(1000, 1.0F)).size(), 1u);
}

TEST(ConcatGroupTracker, SharedPickServesEveryReferencingGroup)
{
  // Topic 1's only message is picked by BOTH groups: its payload must stay
  // cached until the second group's job has been built.
  ConcatGroupTracker t({input(2), input(1)}, {{1000, {0, 0}}, {2000, {1, 0}}});

  const auto shared = xyz_payload(1000, 2.0F);
  EXPECT_TRUE(t.on_message(1, 0, shared).empty());

  const auto first = t.on_message(0, 0, xyz_payload(1000, 1.0F));
  ASSERT_EQ(first.size(), 1u);
  EXPECT_EQ(first[0].output_stamp_ns, 1000);

  const auto second = t.on_message(0, 1, xyz_payload(2000, 1.0F));
  ASSERT_EQ(second.size(), 1u);
  EXPECT_EQ(second[0].output_stamp_ns, 2000);
  ASSERT_EQ(second[0].picks.size(), 2u);
  ASSERT_NE(second[0].picks[1].payload, nullptr);
  EXPECT_EQ(*second[0].picks[1].payload, shared);  // still served after group 0
}

TEST(ConcatGroupTracker, OneArrivalCanFireMultipleGroups)
{
  // Topic 0's message 0 is the last missing pick of BOTH groups: they must
  // fire together, in group order.
  ConcatGroupTracker t({input(1), input(2)}, {{1000, {0, 0}}, {2000, {0, 1}}});

  EXPECT_TRUE(t.on_message(1, 0, xyz_payload(1000, 2.0F)).empty());
  EXPECT_TRUE(t.on_message(1, 1, xyz_payload(2000, 3.0F)).empty());

  const auto jobs = t.on_message(0, 0, xyz_payload(1000, 1.0F));
  ASSERT_EQ(jobs.size(), 2u);
  EXPECT_EQ(jobs[0].output_stamp_ns, 1000);
  EXPECT_EQ(jobs[1].output_stamp_ns, 2000);
}

// ---------------------------------------------------------------------------
// process_concat_group (parallel Pass B: per-group worker computation)
// ---------------------------------------------------------------------------

std::shared_ptr<const std::vector<std::byte>> shared_payload(std::vector<std::byte> bytes)
{
  return std::make_shared<const std::vector<std::byte>>(std::move(bytes));
}

std::vector<pc::RigidTransform> identities(std::size_t n)
{
  return std::vector<pc::RigidTransform>(n);
}

TEST(ProcessConcatGroup, MergesPicksInPcdOrder)
{
  ConcatGroupJob job;
  job.output_stamp_ns = 1000;
  job.picks = {
    {0, 0, shared_payload(xyz_payload(1000, 1.0F))},
    {1, 0, shared_payload(xyz_payload(1000, 2.0F))},
  };
  const auto res = process_concat_group(job, identities(2), "base_link", 2);
  EXPECT_TRUE(res.error.empty());
  EXPECT_FALSE(res.partial);
  EXPECT_EQ(res.stamp_ns, 1000);

  const auto merged = pc::parse_pointcloud2(res.payload);
  ASSERT_TRUE(merged.ok());
  EXPECT_EQ(merged.cloud->width, 2u);
  EXPECT_EQ(merged.cloud->timestamp_ns, 1000);
  EXPECT_EQ(merged.cloud->frame_id, "base_link");

  ASSERT_EQ(res.picks.size(), 2u);
  EXPECT_EQ(res.picks[0].status, ConcatPickOutcome::Status::kOk);
  EXPECT_EQ(res.picks[1].status, ConcatPickOutcome::Status::kOk);
}

TEST(ProcessConcatGroup, ParseFailureYieldsPartialOutput)
{
  ConcatGroupJob job;
  job.output_stamp_ns = 1000;
  job.picks = {
    {0, 0, shared_payload(garbage_payload())},
    {1, 0, shared_payload(xyz_payload(1000, 2.0F))},
  };
  const auto res = process_concat_group(job, identities(2), "base_link", 2);
  EXPECT_TRUE(res.error.empty());
  EXPECT_TRUE(res.partial);

  const auto merged = pc::parse_pointcloud2(res.payload);
  ASSERT_TRUE(merged.ok());
  EXPECT_EQ(merged.cloud->width, 1u);  // only the surviving pick

  ASSERT_EQ(res.picks.size(), 2u);
  EXPECT_EQ(res.picks[0].status, ConcatPickOutcome::Status::kParseFail);
  EXPECT_EQ(res.picks[1].status, ConcatPickOutcome::Status::kOk);
}

TEST(ProcessConcatGroup, TransformFailureYieldsPartialOutput)
{
  ConcatGroupJob job;
  job.output_stamp_ns = 1000;
  job.picks = {
    {0, 0, shared_payload(xy_payload(1000))},  // no z field -> transform fails
    {1, 0, shared_payload(xyz_payload(1000, 2.0F))},
  };
  const auto res = process_concat_group(job, identities(2), "base_link", 2);
  EXPECT_TRUE(res.error.empty());
  EXPECT_TRUE(res.partial);
  ASSERT_EQ(res.picks.size(), 2u);
  EXPECT_EQ(res.picks[0].status, ConcatPickOutcome::Status::kTransformFail);
}

TEST(ProcessConcatGroup, AllPicksFailedEmitsNoPayload)
{
  ConcatGroupJob job;
  job.output_stamp_ns = 1000;
  job.picks = {
    {0, 0, shared_payload(garbage_payload())},
    {1, 0, shared_payload(garbage_payload())},
  };
  const auto res = process_concat_group(job, identities(2), "base_link", 2);
  EXPECT_TRUE(res.error.empty());
  EXPECT_TRUE(res.partial);
  EXPECT_TRUE(res.payload.empty());
}

TEST(ProcessConcatGroup, LayoutMismatchSetsError)
{
  ConcatGroupJob job;
  job.output_stamp_ns = 1000;
  job.picks = {
    {0, 0, shared_payload(xyz_payload(1000, 1.0F))},
    {1, 0, shared_payload(xyzi_payload(1000))},  // extra intensity field
  };
  const auto res = process_concat_group(job, identities(2), "base_link", 2);
  EXPECT_FALSE(res.error.empty());
  EXPECT_TRUE(res.payload.empty());
  // Both picks parsed and transformed before concat_clouds rejected the layout.
  ASSERT_EQ(res.picks.size(), 2u);
  EXPECT_EQ(res.picks[0].status, ConcatPickOutcome::Status::kOk);
  EXPECT_EQ(res.picks[1].status, ConcatPickOutcome::Status::kOk);
}

// ---------------------------------------------------------------------------
// ConcatCounterMerger (parallel Pass B: collector-side counter accumulation)
// ---------------------------------------------------------------------------

TEST(ConcatCounterMerger, ReplicatesSerialCounters)
{
  ConcatCounterMerger m(2);

  // A full group: written, both picks matched.
  ConcatGroupJob full;
  full.output_stamp_ns = 1000;
  full.picks = {
    {0, 0, shared_payload(xyz_payload(1000, 1.0F))},
    {1, 0, shared_payload(xyz_payload(1000, 2.0F))},
  };
  m.merge(process_concat_group(full, identities(2), "base_link", 2));

  // A partial group: topic 0's message 1 is garbage.
  ConcatGroupJob partial;
  partial.output_stamp_ns = 2000;
  partial.picks = {
    {0, 1, shared_payload(garbage_payload())},
    {1, 1, shared_payload(xyz_payload(2000, 2.0F))},
  };
  m.merge(process_concat_group(partial, identities(2), "base_link", 2));

  EXPECT_EQ(m.counters().written_groups, 2);
  EXPECT_EQ(m.counters().partial_groups, 1);
  EXPECT_EQ(m.counters().matched, (std::vector<std::int64_t>{1, 2}));
  EXPECT_EQ(m.counters().parse_fail, (std::vector<std::int64_t>{1, 0}));
  EXPECT_EQ(m.counters().transform_fail, (std::vector<std::int64_t>{0, 0}));
}

TEST(ConcatCounterMerger, SharedPickFailureCountsOnce)
{
  // The serial path parses each message once, so a failing message shared by
  // two groups counts one parse failure. The parallel path re-parses it per
  // group; the merger must dedupe the failure back to one.
  ConcatCounterMerger m(2);
  for (const std::int64_t stamp : {1000, 2000}) {
    ConcatGroupJob job;
    job.output_stamp_ns = stamp;
    job.picks = {
      {0, static_cast<std::size_t>(stamp == 1000 ? 0 : 1),
       shared_payload(xyz_payload(stamp, 1.0F))},
      {1, 0, shared_payload(garbage_payload())},  // the shared failing pick
    };
    m.merge(process_concat_group(job, identities(2), "base_link", 2));
  }

  EXPECT_EQ(m.counters().parse_fail, (std::vector<std::int64_t>{0, 1}));
  EXPECT_EQ(m.counters().partial_groups, 2);
  EXPECT_EQ(m.counters().written_groups, 2);
  EXPECT_EQ(m.counters().matched, (std::vector<std::int64_t>{2, 0}));
}

}  // namespace
