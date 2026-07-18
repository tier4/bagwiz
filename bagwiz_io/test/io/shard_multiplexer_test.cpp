// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "io/shard_multiplexer.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{

using bagwiz::io::BagMetadata;
using bagwiz::io::BagReader;
using bagwiz::io::RawMessage;
using bagwiz::io::ReadFilter;
using bagwiz::io::TopicInfo;
using bagwiz::io::detail::ShardMultiplexer;

TopicInfo make_topic(std::string name, std::string schema_text = "")
{
  TopicInfo info;
  info.name = std::move(name);
  info.type = "std_msgs/msg/String";
  info.serialization_format = "cdr";
  if (!schema_text.empty()) {
    info.schema_encoding = "ros2msg";
    info.schema_text = std::move(schema_text);
  }
  return info;
}

// Scriptable per-shard reader: canned topics / messages / stats / counts /
// extent, recording how the multiplexer drives it. Messages reference the
// canned topics by index so the RawMessage topic pointers stay stable.
class FakeFileReader : public BagReader
{
public:
  std::vector<TopicInfo> canned_topics;
  std::vector<std::pair<std::size_t, std::int64_t>> messages;  // (topic idx, stamp ns)
  Stats canned_stats;
  std::unordered_map<std::string, std::int64_t> canned_counts;
  TimeExtent canned_extent;

  int filter_calls = 0;
  ReadFilter last_filter;

  std::span<const TopicInfo> topics() const override { return canned_topics; }

  void set_filter(const ReadFilter & f) override
  {
    ++filter_calls;
    last_filter = f;
  }

  bool next(RawMessage & out) override
  {
    if (pos_ >= messages.size()) {
      return false;
    }
    const auto & [topic_idx, stamp_ns] = messages[pos_++];
    out.topic = &canned_topics[topic_idx];
    out.timestamp_ns = stamp_ns;
    return true;
  }

  Stats compute_stats() override { return canned_stats; }

  std::unordered_map<std::string, std::int64_t> compute_topic_counts(
    std::span<const std::string> topics) override
  {
    std::unordered_map<std::string, std::int64_t> result;
    for (const auto & name : topics) {
      if (auto it = canned_counts.find(name); it != canned_counts.end()) {
        result[name] = it->second;
      }
    }
    return result;
  }

  TimeExtent compute_time_extent() override { return canned_extent; }

private:
  std::size_t pos_ = 0;
};

// Multiplexer with pre-scripted shards and observable hooks. The remap hook
// mimics McapShardReader's schema backfill so the per-message fixup path is
// pinned; scan_from_summary is scripted per test.
class FakeMultiplexer : public ShardMultiplexer<FakeFileReader>
{
public:
  FakeMultiplexer(
    std::filesystem::path dir, std::vector<std::filesystem::path> shard_rel_paths,
    std::vector<TopicInfo> topics, BagMetadata metadata)
  : ShardMultiplexer(
      std::move(dir), std::move(shard_rel_paths), std::move(topics), std::move(metadata))
  {
  }

  // Shards handed out (in order) by open_shard; the test scripts them upfront.
  // mutable because open_shard is const per the multiplexer hook contract.
  mutable std::vector<std::unique_ptr<FakeFileReader>> scripted;
  mutable int open_calls = 0;
  mutable int remap_calls = 0;
  bool scan_flag_result = false;
  mutable std::vector<std::vector<bool>> scan_flag_calls;

  // Read-only view of shard i's script after it was opened (for assertions on
  // filter application). Valid only for i < shards_opened().
  const FakeFileReader & opened_shard(std::size_t i) const { return *shards_[i]; }
  std::size_t shards_opened() const
  {
    std::size_t n = 0;
    for (const auto & s : shards_) {
      if (s) {
        ++n;
      }
    }
    return n;
  }

private:
  std::unique_ptr<FakeFileReader> open_shard(
    const std::filesystem::path & /*shard_path*/) const override
  {
    ++open_calls;
    return std::move(scripted[opened_++]);
  }

  void on_topic_remapped(TopicInfo & bag_topic, const TopicInfo & shard_topic) const override
  {
    ++remap_calls;
    if (bag_topic.schema_text.empty() && !shard_topic.schema_text.empty()) {
      bag_topic.schema_text = shard_topic.schema_text;
    }
    if (bag_topic.schema_encoding.empty() && !shard_topic.schema_encoding.empty()) {
      bag_topic.schema_encoding = shard_topic.schema_encoding;
    }
  }

  bool scan_from_summary(const std::vector<bool> & shard_flags) const override
  {
    scan_flag_calls.push_back(shard_flags);
    return scan_flag_result;
  }

  mutable std::size_t opened_ = 0;
};

BagMetadata make_metadata(
  bool has_summary, std::vector<TopicInfo> topics = {},
  std::unordered_map<std::string, std::int64_t> per_topic_counts = {})
{
  BagMetadata md;
  md.storage_identifier = "fake";
  md.topics = std::move(topics);
  md.has_summary = has_summary;
  md.total_messages = 7;
  md.start_ns = 100;
  md.end_ns = 900;
  md.per_topic_counts = std::move(per_topic_counts);
  return md;
}

TEST(ShardMultiplexer, TopicsFromMetadataWithoutOpeningShards)
{
  FakeMultiplexer mux(
    "/bag", {"shard_0", "shard_1"}, {make_topic("/a"), make_topic("/b")}, make_metadata(false));
  const auto topics = mux.topics();
  ASSERT_EQ(topics.size(), 2U);
  EXPECT_EQ(topics[0].name, "/a");
  EXPECT_EQ(topics[1].name, "/b");
  EXPECT_EQ(mux.open_calls, 0);
}

TEST(ShardMultiplexer, TopicsFallbackToFirstShardAndCache)
{
  FakeMultiplexer mux("/bag", {"shard_0"}, {}, make_metadata(false));
  auto shard0 = std::make_unique<FakeFileReader>();
  shard0->canned_topics = {make_topic("/from_shard")};
  mux.scripted.push_back(std::move(shard0));

  const auto topics = mux.topics();
  ASSERT_EQ(topics.size(), 1U);
  EXPECT_EQ(topics[0].name, "/from_shard");
  EXPECT_EQ(mux.open_calls, 1);
  // Second call must not re-open: the derived list is cached.
  EXPECT_EQ(mux.topics().size(), 1U);
  EXPECT_EQ(mux.open_calls, 1);
}

TEST(ShardMultiplexer, NextConcatenatesShardsAndRemapsTopicPointers)
{
  std::vector<TopicInfo> bag_topics{make_topic("/a"), make_topic("/b")};
  FakeMultiplexer mux("/bag", {"s0", "s1"}, bag_topics, make_metadata(false));

  auto s0 = std::make_unique<FakeFileReader>();
  s0->canned_topics = {make_topic("/a", "shard schema"), make_topic("/b", "shard schema")};
  s0->messages = {{0, 10}, {1, 20}};
  auto s1 = std::make_unique<FakeFileReader>();
  s1->canned_topics = {make_topic("/a", "shard schema")};
  s1->messages = {{0, 30}};
  mux.scripted.push_back(std::move(s0));
  mux.scripted.push_back(std::move(s1));

  RawMessage raw;
  std::vector<std::int64_t> stamps;
  while (mux.next(raw)) {
    stamps.push_back(raw.timestamp_ns);
    // The topic pointer must be the multiplexer's owned TopicInfo (stable for
    // the whole bag), never the shard's.
    EXPECT_GE(raw.topic, &mux.topics().front());
    EXPECT_LE(raw.topic, &mux.topics().back());
  }
  EXPECT_EQ(stamps, (std::vector<std::int64_t>{10, 20, 30}));
  EXPECT_EQ(mux.remap_calls, 3);
  // The remap hook backfilled the metadata-derived topics from the shards.
  EXPECT_EQ(mux.topics()[0].schema_text, "shard schema");
  EXPECT_EQ(mux.topics()[0].schema_encoding, "ros2msg");
  EXPECT_EQ(mux.topics()[1].schema_text, "shard schema");
  // EOF stays sticky: further next() calls keep returning false.
  EXPECT_FALSE(mux.next(raw));
}

TEST(ShardMultiplexer, NextAppliesPendingFilterLazilyPerShard)
{
  FakeMultiplexer mux("/bag", {"s0", "s1"}, {make_topic("/a")}, make_metadata(false));
  auto s0 = std::make_unique<FakeFileReader>();
  s0->canned_topics = {make_topic("/a")};
  s0->messages = {{0, 1}};
  auto s1 = std::make_unique<FakeFileReader>();
  s1->canned_topics = {make_topic("/a")};
  s1->messages = {{0, 2}};
  mux.scripted.push_back(std::move(s0));
  mux.scripted.push_back(std::move(s1));

  ReadFilter filter;
  filter.topics = {"/a"};
  filter.start_ns = 5;
  mux.set_filter(filter);

  RawMessage raw;
  ASSERT_TRUE(mux.next(raw));
  EXPECT_EQ(mux.opened_shard(0).filter_calls, 1);
  EXPECT_EQ(mux.shards_opened(), 1U);  // shard 1 still untouched
  ASSERT_TRUE(mux.next(raw));
  EXPECT_EQ(mux.opened_shard(1).filter_calls, 1);
  EXPECT_EQ(mux.opened_shard(1).last_filter.topics, filter.topics);
  EXPECT_EQ(mux.opened_shard(1).last_filter.start_ns, filter.start_ns);
  EXPECT_FALSE(mux.next(raw));
  // The filter is applied once per shard, not per message.
  EXPECT_EQ(mux.opened_shard(0).filter_calls, 1);
  EXPECT_EQ(mux.opened_shard(1).filter_calls, 1);
}

TEST(ShardMultiplexer, SetFilterAfterIterationThrows)
{
  FakeMultiplexer mux("/bag", {"s0"}, {make_topic("/a")}, make_metadata(false));
  auto s0 = std::make_unique<FakeFileReader>();
  s0->canned_topics = {make_topic("/a")};
  s0->messages = {{0, 1}};
  mux.scripted.push_back(std::move(s0));

  RawMessage raw;
  ASSERT_TRUE(mux.next(raw));
  EXPECT_THROW(
    {
      try {
        mux.set_filter(ReadFilter{});
      } catch (const std::runtime_error & e) {
        EXPECT_STREQ(e.what(), "BagReader::set_filter called after iteration started");
        throw;
      }
    },
    std::runtime_error);
}

TEST(ShardMultiplexer, StatsFromMetadataSummary)
{
  auto md = make_metadata(true, {make_topic("/a")}, {{"/a", 4}, {"/b", 3}});
  FakeMultiplexer mux("/bag", {"s0"}, md.topics, md);

  const auto stats = mux.compute_stats();
  EXPECT_TRUE(stats.from_summary);
  EXPECT_EQ(stats.total_messages, 7);
  EXPECT_EQ(stats.start_ns, 100);
  EXPECT_EQ(stats.end_ns, 900);
  EXPECT_EQ(stats.per_topic, (std::unordered_map<std::string, std::int64_t>{{"/a", 4}, {"/b", 3}}));
  EXPECT_EQ(mux.open_calls, 0);
}

TEST(ShardMultiplexer, StatsFromShardScans)
{
  FakeMultiplexer mux(
    "/bag", {"s0", "s1"}, {make_topic("/a"), make_topic("/b")}, make_metadata(false));
  mux.scan_flag_result = true;  // MCAP-style policy answer, scripted

  auto s0 = std::make_unique<FakeFileReader>();
  s0->canned_stats.from_summary = true;
  s0->canned_stats.total_messages = 3;
  s0->canned_stats.start_ns = 50;
  s0->canned_stats.end_ns = 400;
  s0->canned_stats.per_topic = {{"/a", 2}, {"/b", 1}};
  auto s1 = std::make_unique<FakeFileReader>();
  s1->canned_stats.from_summary = false;
  s1->canned_stats.total_messages = 5;
  s1->canned_stats.start_ns = 10;
  s1->canned_stats.end_ns = 300;
  s1->canned_stats.per_topic = {{"/a", 4}};
  mux.scripted.push_back(std::move(s0));
  mux.scripted.push_back(std::move(s1));

  const auto stats = mux.compute_stats();
  EXPECT_EQ(stats.total_messages, 8);
  EXPECT_EQ(stats.start_ns, 10);  // min across shards
  EXPECT_EQ(stats.end_ns, 400);   // max across shards
  EXPECT_EQ(stats.per_topic.at("/a"), 6);
  EXPECT_EQ(stats.per_topic.at("/b"), 1);
  // The per-shard flags reached the format-specific policy, whose scripted
  // answer became the combined flag.
  ASSERT_EQ(mux.scan_flag_calls.size(), 1U);
  EXPECT_EQ(mux.scan_flag_calls[0], (std::vector<bool>{true, false}));
  EXPECT_TRUE(stats.from_summary);
}

TEST(ShardMultiplexer, TopicCountsEmptyRequestOpensNothing)
{
  FakeMultiplexer mux("/bag", {"s0"}, {make_topic("/a")}, make_metadata(false));
  EXPECT_TRUE(mux.compute_topic_counts({}).empty());
  EXPECT_EQ(mux.open_calls, 0);
}

TEST(ShardMultiplexer, TopicCountsFromMetadataSummary)
{
  auto md = make_metadata(true, {make_topic("/a")}, {{"/a", 4}, {"/b", 3}});
  FakeMultiplexer mux("/bag", {"s0"}, md.topics, md);

  const std::vector<std::string> request{"/a", "/absent"};
  const auto counts = mux.compute_topic_counts(request);
  EXPECT_EQ(counts, (std::unordered_map<std::string, std::int64_t>{{"/a", 4}}));
  EXPECT_EQ(mux.open_calls, 0);
}

TEST(ShardMultiplexer, TopicCountsFromShardScans)
{
  FakeMultiplexer mux("/bag", {"s0", "s1"}, {make_topic("/a")}, make_metadata(false));
  auto s0 = std::make_unique<FakeFileReader>();
  s0->canned_counts = {{"/a", 2}, {"/b", 9}};
  auto s1 = std::make_unique<FakeFileReader>();
  s1->canned_counts = {{"/a", 5}};
  mux.scripted.push_back(std::move(s0));
  mux.scripted.push_back(std::move(s1));

  const std::vector<std::string> request{"/a", "/b", "/absent"};
  const auto counts = mux.compute_topic_counts(request);
  EXPECT_EQ(counts, (std::unordered_map<std::string, std::int64_t>{{"/a", 7}, {"/b", 9}}));
}

TEST(ShardMultiplexer, TimeExtentFromMetadataSummary)
{
  auto md = make_metadata(true, {make_topic("/a")});
  FakeMultiplexer mux("/bag", {"s0"}, md.topics, md);

  const auto extent = mux.compute_time_extent();
  EXPECT_TRUE(extent.has_data);
  EXPECT_EQ(extent.start_ns, 100);
  EXPECT_EQ(extent.end_ns, 900);
  EXPECT_EQ(mux.open_calls, 0);
}

TEST(ShardMultiplexer, TimeExtentFromShardScansSkipsEmptyShards)
{
  FakeMultiplexer mux("/bag", {"s0", "s1", "s2"}, {make_topic("/a")}, make_metadata(false));
  auto s0 = std::make_unique<FakeFileReader>();
  s0->canned_extent.has_data = false;
  auto s1 = std::make_unique<FakeFileReader>();
  s1->canned_extent.has_data = true;
  s1->canned_extent.start_ns = 200;
  s1->canned_extent.end_ns = 500;
  auto s2 = std::make_unique<FakeFileReader>();
  s2->canned_extent.has_data = true;
  s2->canned_extent.start_ns = 100;
  s2->canned_extent.end_ns = 400;
  mux.scripted.push_back(std::move(s0));
  mux.scripted.push_back(std::move(s1));
  mux.scripted.push_back(std::move(s2));

  const auto extent = mux.compute_time_extent();
  EXPECT_TRUE(extent.has_data);
  EXPECT_EQ(extent.start_ns, 100);
  EXPECT_EQ(extent.end_ns, 500);
}

TEST(ShardMultiplexer, EmptyShardList)
{
  FakeMultiplexer mux("/bag", {}, {}, make_metadata(false));
  mux.scan_flag_result = false;  // sqlite-style policy on an empty flag list

  RawMessage raw;
  EXPECT_FALSE(mux.next(raw));
  EXPECT_TRUE(mux.topics().empty());
  EXPECT_EQ(mux.open_calls, 0);

  const auto stats = mux.compute_stats();
  EXPECT_EQ(stats.total_messages, 0);
  EXPECT_TRUE(stats.per_topic.empty());
  EXPECT_FALSE(stats.from_summary);
  ASSERT_EQ(mux.scan_flag_calls.size(), 1U);
  EXPECT_TRUE(mux.scan_flag_calls[0].empty());

  EXPECT_FALSE(mux.compute_time_extent().has_data);
}

}  // namespace
