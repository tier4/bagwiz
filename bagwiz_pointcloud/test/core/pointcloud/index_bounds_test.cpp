// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/fetcher.hpp"
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
#include <tuple>
#include <utility>
#include <vector>

namespace
{

using bagwiz::core::pointcloud::build_point_cloud_index;
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

void write_field(CdrBuilder & b, const std::string & name, std::uint32_t offset)
{
  b.str(name);
  b.u32(offset);
  b.u8(7);  // float32
  b.u32(1);
}

// Serialize a single-point sensor_msgs/msg/PointCloud2 with float32 x/y/z (and
// optionally intensity), whose header.stamp is `stamp_ns` and whose one point is
// at (x, 0, 0). The x value drives both the distance property and the
// fingerprint.
std::vector<std::byte> make_pointcloud2_payload(std::int64_t stamp_ns, float x, bool with_intensity)
{
  const float intensity = 42.0f;
  const std::uint32_t point_step = with_intensity ? 16 : 12;
  std::vector<std::byte> data(point_step, std::byte{0});
  std::memcpy(data.data(), &x, sizeof(x));
  if (with_intensity) {
    std::memcpy(data.data() + 12, &intensity, sizeof(intensity));
  }

  CdrBuilder b;
  b.i32(static_cast<std::int32_t>(stamp_ns / 1'000'000'000LL));   // header.stamp.sec
  b.u32(static_cast<std::uint32_t>(stamp_ns % 1'000'000'000LL));  // header.stamp.nanosec
  b.str("lidar");                                                 // header.frame_id
  b.u32(1);                                                       // height
  b.u32(1);                                                       // width (1 point)
  b.u32(with_intensity ? 4 : 3);                                  // fields length
  write_field(b, "x", 0);
  write_field(b, "y", 4);
  write_field(b, "z", 8);
  if (with_intensity) {
    write_field(b, "intensity", 12);
  }
  b.u8(0);                                         // is_bigendian
  b.u32(point_step);                               // point_step
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

// Write a one-topic MCAP bag with the given (record_ns, stamp_ns, x) clouds in
// ascending record-time order.
std::filesystem::path write_cloud_bag(
  const std::filesystem::path & dir, const std::string & topic,
  const std::vector<std::tuple<std::int64_t, std::int64_t, float>> & clouds, bool with_intensity)
{
  const auto path = dir / "input";
  auto writer = bagwiz::io::open_write(path, mcap_dir_opts());
  writer->declare_topic(make_topic(topic, "sensor_msgs/msg/PointCloud2"));
  for (const auto & [record_ns, stamp_ns, x] : clouds) {
    const auto payload = make_pointcloud2_payload(stamp_ns, x, with_intensity);
    writer->write(topic, record_ns, {payload.data(), payload.size()});
  }
  writer->close();
  return path;
}

class IndexBoundsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ =
      std::filesystem::temp_directory_path() /
      ("bagwiz_index_bounds_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) +
       "_" +
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

  std::filesystem::path tmp_dir_;
};

// When both bounds are supplied manually the value scan is skipped entirely:
// the manual values win, yet entries and the stamp bookkeeping are still built
// (here: one cloud with a real header.stamp and one without, which falls back
// to record time).
TEST_F(IndexBoundsTest, ManualBoundsSkipValueScan)
{
  const std::string topic = "/points";
  const auto bag = write_cloud_bag(
    tmp_dir_, topic, {{100, 1'000'000'000, 5.0f}, {200, 0, -3.0f}}, /*with_intensity=*/false);

  std::string error;
  const auto idx = build_point_cloud_index(
    bag, topic, PointCloudProperty::kDistance, /*manual_min=*/-100.0, /*manual_max=*/100.0, error);
  ASSERT_TRUE(idx.has_value()) << error;
  EXPECT_DOUBLE_EQ(idx->property_min, -100.0);
  EXPECT_DOUBLE_EQ(idx->property_max, 100.0);

  ASSERT_EQ(idx->entries.size(), 2U);
  EXPECT_EQ(idx->entries[0].stamp_ns, 1'000'000'000);
  EXPECT_EQ(idx->entries[0].record_ns, 100);
  EXPECT_EQ(idx->entries[1].stamp_ns, 200);  // header.stamp unset -> record time
  EXPECT_EQ(idx->entries[1].record_ns, 200);
  EXPECT_FALSE(idx->header_stamps_present);  // the second message fell back
  EXPECT_FALSE(idx->has_intensity);
}

// A single manual bound pins only its side; the other side is still computed
// from the per-point scan. Distances of (5,0,0) and (-3,0,0) are 5 and 3.
TEST_F(IndexBoundsTest, SingleManualBoundLeavesOtherSideAuto)
{
  const std::string topic = "/points";
  const auto bag = write_cloud_bag(
    tmp_dir_, topic, {{100, 1'000'000'000, 5.0f}, {200, 2'000'000'000, -3.0f}},
    /*with_intensity=*/false);

  std::string error;
  const auto min_pinned = build_point_cloud_index(
    bag, topic, PointCloudProperty::kDistance, /*manual_min=*/-100.0, /*manual_max=*/std::nullopt,
    error);
  ASSERT_TRUE(min_pinned.has_value()) << error;
  EXPECT_DOUBLE_EQ(min_pinned->property_min, -100.0);
  EXPECT_DOUBLE_EQ(min_pinned->property_max, 5.0);

  const auto max_pinned = build_point_cloud_index(
    bag, topic, PointCloudProperty::kDistance, /*manual_min=*/std::nullopt, /*manual_max=*/100.0,
    error);
  ASSERT_TRUE(max_pinned.has_value()) << error;
  EXPECT_DOUBLE_EQ(max_pinned->property_min, 3.0);
  EXPECT_DOUBLE_EQ(max_pinned->property_max, 100.0);
}

// Selecting the intensity property on a cloud without an intensity field is a
// hard error (unlike the other properties, intensity has no neutral fallback).
TEST_F(IndexBoundsTest, IntensityPropertyWithoutFieldErrors)
{
  const std::string topic = "/points";
  const auto bag =
    write_cloud_bag(tmp_dir_, topic, {{100, 1'000'000'000, 5.0f}}, /*with_intensity=*/false);

  std::string error;
  const auto idx = build_point_cloud_index(
    bag, topic, PointCloudProperty::kIntensity, std::nullopt, std::nullopt, error);
  EXPECT_FALSE(idx.has_value());
  EXPECT_EQ(error, "point cloud has no intensity field");
}

// Intensity detection reads only the header, so it works on the header-only
// path too (both bounds manual); the selected property's auto range is then
// the intensity values' when the intensity property is chosen.
TEST_F(IndexBoundsTest, IntensityDetectedAndScannedWhenFieldPresent)
{
  const std::string topic = "/points";
  const auto bag =
    write_cloud_bag(tmp_dir_, topic, {{100, 1'000'000'000, 5.0f}}, /*with_intensity=*/true);

  std::string error;
  const auto header_only = build_point_cloud_index(
    bag, topic, PointCloudProperty::kDistance, /*manual_min=*/0.0, /*manual_max=*/1.0, error);
  ASSERT_TRUE(header_only.has_value()) << error;
  EXPECT_TRUE(header_only->has_intensity);

  const auto intensity = build_point_cloud_index(
    bag, topic, PointCloudProperty::kIntensity, std::nullopt, std::nullopt, error);
  ASSERT_TRUE(intensity.has_value()) << error;
  EXPECT_TRUE(intensity->has_intensity);
  EXPECT_DOUBLE_EQ(intensity->property_min, 42.0);
  EXPECT_DOUBLE_EQ(intensity->property_max, 42.0);
}

}  // namespace
