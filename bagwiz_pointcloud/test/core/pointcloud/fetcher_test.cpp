// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/fetcher.hpp"

#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/core/pointcloud/property.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace
{
using bagwiz::core::pointcloud::build_point_cloud_index;
using bagwiz::core::pointcloud::PointCloud2;
using bagwiz::core::pointcloud::PointCloudFetcher;
using bagwiz::core::pointcloud::PointCloudIndexEntry;
using bagwiz::core::pointcloud::PointCloudMatchKey;
using bagwiz::core::pointcloud::PointCloudProperty;

// Little-endian CDR-1 builder (see raw_image_test.cpp for the alignment rule:
// alignment is relative to the 4-byte encapsulation header).
class CdrBuilder
{
public:
  CdrBuilder()
  {
    for (int b : {0x00, 0x01, 0x00, 0x00}) {
      buf_.push_back(static_cast<std::byte>(b));
    }
  }
  void u8(std::uint8_t v) { buf_.push_back(static_cast<std::byte>(v)); }
  void u32(std::uint32_t v)
  {
    align(4);
    for (int i = 0; i < 4; ++i) {
      buf_.push_back(static_cast<std::byte>((v >> (8 * i)) & 0xFFU));
    }
  }
  void i32(std::int32_t v) { u32(static_cast<std::uint32_t>(v)); }
  void str(const std::string & s)
  {
    u32(static_cast<std::uint32_t>(s.size() + 1));
    for (char c : s) {
      buf_.push_back(static_cast<std::byte>(static_cast<unsigned char>(c)));
    }
    buf_.push_back(std::byte{0});
  }
  void byte_seq(std::span<const std::byte> b)
  {
    u32(static_cast<std::uint32_t>(b.size()));
    for (auto x : b) {
      buf_.push_back(x);
    }
  }
  [[nodiscard]] std::vector<std::byte> take() const { return buf_; }

private:
  void align(std::size_t n)
  {
    while ((buf_.size() - 4) % n != 0) {
      buf_.push_back(std::byte{0});
    }
  }
  std::vector<std::byte> buf_;
};

// Serialize a single-point sensor_msgs/msg/PointCloud2 with float32 x/y/z, whose
// header.stamp is `stamp_ns` and whose one point is at (x, 0, 0). The x value is
// used as a per-message fingerprint the test reads back.
std::vector<std::byte> make_pointcloud2_payload(std::int64_t stamp_ns, float x)
{
  constexpr std::uint32_t kPointStep = 12;
  std::vector<std::byte> data(kPointStep, std::byte{0});
  std::memcpy(data.data(), &x, sizeof(x));

  CdrBuilder b;
  b.i32(static_cast<std::int32_t>(stamp_ns / 1'000'000'000LL));   // header.stamp.sec
  b.u32(static_cast<std::uint32_t>(stamp_ns % 1'000'000'000LL));  // header.stamp.nanosec
  b.str("lidar");                                                 // header.frame_id
  b.u32(1);                                                       // height
  b.u32(1);                                                       // width (1 point)
  b.u32(3);                                                       // fields length
  b.str("x");
  b.u32(0);
  b.u8(7);  // float32
  b.u32(1);
  b.str("y");
  b.u32(4);
  b.u8(7);
  b.u32(1);
  b.str("z");
  b.u32(8);
  b.u8(7);
  b.u32(1);
  b.u8(0);                                         // is_bigendian
  b.u32(kPointStep);                               // point_step
  b.u32(static_cast<std::uint32_t>(data.size()));  // row_step
  b.byte_seq({data.data(), data.size()});
  b.u8(1);  // is_dense
  return b.take();
}

bagwiz::io::TopicInfo make_topic(std::string name, std::string type)
{
  bagwiz::io::TopicInfo t;
  t.name = std::move(name);
  t.type = std::move(type);
  t.serialization_format = "cdr";
  return t;
}

bagwiz::io::CreateOptions mcap_dir_opts()
{
  bagwiz::io::CreateOptions opts;
  opts.format = bagwiz::io::Format::Mcap;
  opts.layout = bagwiz::io::Layout::Directory;
  opts.mcap_compression = "none";
  return opts;
}

// One (record_ns, header_stamp_ns, fingerprint x) triple for a cloud message.
struct CloudSpec
{
  std::int64_t record_ns;
  std::int64_t stamp_ns;
  float x;
};

// Write an MCAP bag with a single PointCloud2 topic carrying the given clouds in
// ascending record-time order (as a recorder would).
std::filesystem::path write_cloud_bag(
  const std::filesystem::path & dir, const std::string & topic,
  const std::vector<CloudSpec> & specs)
{
  const auto path = dir / "input";
  auto writer = bagwiz::io::open_write(path, mcap_dir_opts());
  writer->declare_topic(make_topic(topic, "sensor_msgs/msg/PointCloud2"));
  for (const auto & s : specs) {
    const auto payload = make_pointcloud2_payload(s.stamp_ns, s.x);
    writer->write(topic, s.record_ns, {payload.data(), payload.size()});
  }
  writer->close();
  return path;
}

// Read back the x of the (single) point in a cloud — the message fingerprint.
float first_point_x(const PointCloud2 & cloud)
{
  float x = 0.0f;
  std::memcpy(&x, cloud.data.data(), sizeof(x));
  return x;
}

class PointCloudFetcherTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ =
      std::filesystem::temp_directory_path() /
      ("bagwiz_fetcher_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) + "_" +
       std::to_string(
         reinterpret_cast<std::uintptr_t>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
           this)));
    std::filesystem::create_directories(tmp_dir_);
  }
  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }

  std::vector<PointCloudIndexEntry> build(
    const std::filesystem::path & bag, const std::string & topic)
  {
    std::string error;
    auto idx = build_point_cloud_index(
      bag, topic, PointCloudProperty::kDistance, std::nullopt, std::nullopt, error);
    EXPECT_TRUE(idx.has_value()) << error;
    return idx.has_value() ? std::move(idx->entries) : std::vector<PointCloudIndexEntry>{};
  }

  std::filesystem::path tmp_dir_;
};

// The fetcher matches on header.stamp, not bag record time. Header stamps are
// deliberately out of record order so a record-time match would pick a different
// cloud; the message fingerprint (point x) proves which cloud was selected.
TEST_F(PointCloudFetcherTest, MatchesByHeaderStampNotRecordTime)
{
  const std::string topic = "/points";
  // record order A,B,C; header stamps 3e9,1e9,2e9 (not sorted by record time).
  const std::vector<CloudSpec> specs{
    {1'000'000'000, 3'000'000'000, 10.0f},  // A
    {2'000'000'000, 1'000'000'000, 20.0f},  // B
    {3'000'000'000, 2'000'000'000, 30.0f},  // C
  };
  const auto bag = write_cloud_bag(tmp_dir_, topic, specs);
  auto entries = build(bag, topic);
  ASSERT_EQ(entries.size(), 3U);

  PointCloudFetcher fetcher(bag, topic, std::move(entries));

  std::string error;
  // target == B's header.stamp -> B (x=20). A record-time match would give A.
  const auto * b = fetcher.fetch(1'000'000'000, PointCloudMatchKey::kHeaderStamp, error);
  ASSERT_NE(b, nullptr) << error;
  EXPECT_FLOAT_EQ(first_point_x(*b), 20.0f);

  // target == A's header.stamp -> A (x=10), loaded via A's record time.
  const auto * a = fetcher.fetch(3'000'000'000, PointCloudMatchKey::kHeaderStamp, error);
  ASSERT_NE(a, nullptr) << error;
  EXPECT_FLOAT_EQ(first_point_x(*a), 10.0f);

  // target == C's header.stamp -> C (x=30).
  const auto * c = fetcher.fetch(2'000'000'000, PointCloudMatchKey::kHeaderStamp, error);
  ASSERT_NE(c, nullptr) << error;
  EXPECT_FLOAT_EQ(first_point_x(*c), 30.0f);
}

// The record-time key matches by bag record time regardless of header.stamp.
// Same clouds as above (header stamps out of record order); fetching by record
// time must select by record_ns, i.e. the opposite cloud from the stamp key.
TEST_F(PointCloudFetcherTest, MatchesByRecordTimeWhenRequested)
{
  const std::string topic = "/points";
  const std::vector<CloudSpec> specs{
    {1'000'000'000, 3'000'000'000, 10.0f},  // A: record 1e9, stamp 3e9
    {2'000'000'000, 1'000'000'000, 20.0f},  // B: record 2e9, stamp 1e9
    {3'000'000'000, 2'000'000'000, 30.0f},  // C: record 3e9, stamp 2e9
  };
  const auto bag = write_cloud_bag(tmp_dir_, topic, specs);
  auto entries = build(bag, topic);
  ASSERT_EQ(entries.size(), 3U);

  PointCloudFetcher fetcher(bag, topic, std::move(entries));

  std::string error;
  // target == A's record time -> A (x=10). A stamp match on 1e9 would give B.
  const auto * a = fetcher.fetch(1'000'000'000, PointCloudMatchKey::kRecordTime, error);
  ASSERT_NE(a, nullptr) << error;
  EXPECT_FLOAT_EQ(first_point_x(*a), 10.0f);

  // target == C's record time -> C (x=30).
  const auto * c = fetcher.fetch(3'000'000'000, PointCloudMatchKey::kRecordTime, error);
  ASSERT_NE(c, nullptr) << error;
  EXPECT_FLOAT_EQ(first_point_x(*c), 30.0f);
}

// When a cloud's header.stamp is unset (0), the entry falls back to bag record
// time so matching still works. Two unset-stamp clouds are distinguished only if
// the fallback keyed each entry by its record time.
TEST_F(PointCloudFetcherTest, FallsBackToRecordTimeWhenHeaderStampUnset)
{
  const std::string topic = "/points";
  const std::vector<CloudSpec> specs{
    {1'000'000'000, 0, 10.0f},  // A, unset stamp
    {2'000'000'000, 0, 20.0f},  // B, unset stamp
  };
  const auto bag = write_cloud_bag(tmp_dir_, topic, specs);
  auto entries = build(bag, topic);
  ASSERT_EQ(entries.size(), 2U);

  PointCloudFetcher fetcher(bag, topic, std::move(entries));

  std::string error;
  // With every stamp unset the topic can't be matched by capture time, so the
  // caller uses the record-time key; each entry resolves by its record time.
  const auto * a = fetcher.fetch(1'000'000'000, PointCloudMatchKey::kRecordTime, error);
  ASSERT_NE(a, nullptr) << error;
  EXPECT_FLOAT_EQ(first_point_x(*a), 10.0f);

  const auto * b = fetcher.fetch(2'000'000'000, PointCloudMatchKey::kRecordTime, error);
  ASSERT_NE(b, nullptr) << error;
  EXPECT_FLOAT_EQ(first_point_x(*b), 20.0f);
}

// scan_point_cloud (used by `walk`'s preview overlay) must also key the index by
// header.stamp, so walk pairs frames with clouds by capture time.
TEST_F(PointCloudFetcherTest, ScanPointCloudMatchesByHeaderStamp)
{
  const std::string topic = "/points";
  const std::vector<CloudSpec> specs{
    {1'000'000'000, 3'000'000'000, 10.0f},  // A
    {2'000'000'000, 1'000'000'000, 20.0f},  // B
    {3'000'000'000, 2'000'000'000, 30.0f},  // C
  };
  const auto bag = write_cloud_bag(tmp_dir_, topic, specs);

  std::string error;
  auto scan = bagwiz::core::pointcloud::scan_point_cloud(bag, topic, error);
  ASSERT_TRUE(scan.has_value()) << error;
  ASSERT_EQ(scan->entries.size(), 3U);

  PointCloudFetcher fetcher(bag, topic, std::move(scan->entries));
  // target == B's header.stamp -> B (x=20), not A (record-time nearest).
  const auto * b = fetcher.fetch(1'000'000'000, PointCloudMatchKey::kHeaderStamp, error);
  ASSERT_NE(b, nullptr) << error;
  EXPECT_FLOAT_EQ(first_point_x(*b), 20.0f);
}

// header_stamps_present decides whether a topic can be matched by capture time.
// It is true only when *every* cloud carried a real header.stamp.
TEST_F(PointCloudFetcherTest, HeaderStampsPresentWhenAllStampsSet)
{
  const std::string topic = "/points";
  const std::vector<CloudSpec> specs{
    {1'000'000'000, 5'000'000'000, 10.0f},
    {2'000'000'000, 6'000'000'000, 20.0f},
  };
  const auto bag = write_cloud_bag(tmp_dir_, topic, specs);

  std::string error;
  auto idx = build_point_cloud_index(
    bag, topic, PointCloudProperty::kDistance, std::nullopt, std::nullopt, error);
  ASSERT_TRUE(idx.has_value()) << error;
  EXPECT_TRUE(idx->header_stamps_present);

  auto scan = bagwiz::core::pointcloud::scan_point_cloud(bag, topic, error);
  ASSERT_TRUE(scan.has_value()) << error;
  EXPECT_TRUE(scan->header_stamps_present);
}

// A single unset header.stamp makes the stamp axis mixed, so the flag is false
// and callers must match this topic by record time.
TEST_F(PointCloudFetcherTest, HeaderStampsAbsentWhenAnyUnset)
{
  const std::string topic = "/points";
  const std::vector<CloudSpec> specs{
    {1'000'000'000, 5'000'000'000, 10.0f},
    {2'000'000'000, 0, 20.0f},  // one unset -> whole topic falls back
  };
  const auto bag = write_cloud_bag(tmp_dir_, topic, specs);

  std::string error;
  auto idx = build_point_cloud_index(
    bag, topic, PointCloudProperty::kDistance, std::nullopt, std::nullopt, error);
  ASSERT_TRUE(idx.has_value()) << error;
  EXPECT_FALSE(idx->header_stamps_present);

  auto scan = bagwiz::core::pointcloud::scan_point_cloud(bag, topic, error);
  ASSERT_TRUE(scan.has_value()) << error;
  EXPECT_FALSE(scan->header_stamps_present);
}

}  // namespace
