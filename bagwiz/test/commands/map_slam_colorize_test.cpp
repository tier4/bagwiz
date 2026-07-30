// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "map_slam_colorize.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include "map_slam_threads.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <thread>
#include <vector>

namespace
{
using bagwiz::commands::build_camera_colorizers;
using bagwiz::commands::build_shared_colorize_geometry;
using bagwiz::commands::colorize_thread_count;
using bagwiz::commands::parse_camera_info_overrides;
using bagwiz::commands::resolve_threads;
using bagwiz::core::TrajectoryPose;

TrajectoryPose make_pose(std::int64_t stamp_ns, double tx = 0.0)
{
  TrajectoryPose pose;
  pose.timestamp_ns = stamp_ns;
  pose.tx = tx;
  pose.qw = 1.0;
  return pose;
}

constexpr std::int64_t kSecondNs = 1'000'000'000;
constexpr const char * kImageType = "sensor_msgs/msg/Image";

// Little-endian CDR-1 builder mirroring the wire layout the production
// CdrReader consumes (same helper as bagwiz_image's packed_raster_test.cpp).
class CdrBuilder
{
public:
  CdrBuilder()
  {
    for (int b : {0x00, 0x01, 0x00, 0x00}) {  // rep_id high=0, low=1 (LE), options=0
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
    u32(static_cast<std::uint32_t>(s.size() + 1));  // length includes the trailing NUL
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

// A raw bgr8 sensor_msgs/msg/Image payload matching make_pinhole()'s 100x100
// resolution, stamped `stamp_sec` seconds (header.stamp.nanosec = 0).
std::vector<std::byte> make_image_payload(std::int32_t stamp_sec, std::uint8_t level = 0x7F)
{
  const std::vector<std::byte> data(
    static_cast<std::size_t>(100) * 3U * 100U, static_cast<std::byte>(level));
  CdrBuilder b;
  b.i32(stamp_sec);  // header.stamp.sec
  b.u32(0);          // header.stamp.nanosec
  b.str("cam");      // header.frame_id
  b.u32(100);        // height
  b.u32(100);        // width
  b.str("bgr8");
  b.u8(0);     // is_bigendian
  b.u32(300);  // step
  b.byte_seq({data.data(), data.size()});
  return b.take();
}

// A 100x100 pinhole with fx = fy = 100 and the principal point at the image
// center; no distortion. A point at (0, 0, z) projects to pixel (50, 50).
bagwiz::core::image::CameraInfo make_pinhole()
{
  bagwiz::core::image::CameraInfo info;
  info.width = 100;
  info.height = 100;
  info.distortion_model = "plumb_bob";
  info.k = {100.0, 0.0, 50.0, 0.0, 100.0, 50.0, 0.0, 0.0, 1.0};
  return info;
}

TEST(ColorizeThreadCount, PositivePassesThroughTheHardwareCap)
{
  EXPECT_EQ(colorize_thread_count(1), 1);
  const unsigned int hardware = std::thread::hardware_concurrency();
  if (hardware > 0) {
    EXPECT_EQ(colorize_thread_count(std::numeric_limits<int>::max()), static_cast<int>(hardware));
  }
}

TEST(ResolveThreads, ZeroResolvesToTheHardwareConcurrency)
{
  const unsigned int hardware = std::thread::hardware_concurrency();
  const int expected = hardware > 0 ? static_cast<int>(hardware) : 1;
  EXPECT_EQ(resolve_threads(0), expected);
  EXPECT_EQ(resolve_threads(-3), expected);
}

TEST(ResolveThreads, PositiveValuesPassThroughUnderTheCap)
{
  EXPECT_EQ(resolve_threads(1), 1);
  const unsigned int hardware = std::thread::hardware_concurrency();
  if (hardware > 1) {
    EXPECT_EQ(resolve_threads(static_cast<int>(hardware) + 1), static_cast<int>(hardware));
  }
}

TEST(BuildSharedColorizeGeometry, CoversEveryPoint)
{
  const std::vector<std::array<float, 3>> points = {
    {0.0F, 0.0F, 5.0F}, {0.1F, 0.0F, 5.0F}, {0.0F, 0.1F, 5.0F}, {0.1F, 0.1F, 5.0F}};
  const auto geometry = build_shared_colorize_geometry(points, 1);
  EXPECT_EQ(geometry->normals.size(), points.size());
  EXPECT_EQ(geometry->spacings.size(), points.size());
}

TEST(BuildCameraColorizers, BuildsOneWorkingColorizerPerCamera)
{
  const std::vector<bagwiz::core::image::CameraInfo> camera_infos = {
    make_pinhole(), make_pinhole()};
  const std::vector<bagwiz::core::slam::SensorTransform> t_cloud_cams = {{}, {}};
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 5.0F}};
  const std::vector<TrajectoryPose> trajectory = {make_pose(0), make_pose(1'000'000'000)};
  const auto geometry = build_shared_colorize_geometry(points, 1);

  auto colorizers = build_camera_colorizers(
    camera_infos, t_cloud_cams, 100.0, 1, false, geometry, points, trajectory);
  ASSERT_EQ(colorizers.size(), 2U);

  // Wiring smoke: an in-span, correctly-sized image is accepted and reduces.
  const std::vector<std::byte> raster(
    static_cast<std::size_t>(100) * 3U * 100U, static_cast<std::byte>(0x7F));
  EXPECT_TRUE(colorizers[0]->add_image(0, raster, 100, 100));
  const auto result = colorizers[0]->finish();
  EXPECT_EQ(result.colors.size(), points.size());
  EXPECT_EQ(result.images_used, 1U);
  EXPECT_EQ(result.images_skipped, 0U);
}

// --- colorize_one_image / colorize_flush_keyframes -----------------------------
// The per-frame routing step shared by the serial path and the per-camera
// worker threads in map_slam.cpp's colorize pass — tested directly here so a
// refactor of the threading shells cannot silently change how frames reach
// the colorizer.

// One camera's colorizer over a 10 s straight-line trajectory (1 m/s along
// +x), matching make_image_payload's 100x100 rasters.
struct RoutingFixture
{
  std::vector<bagwiz::core::image::CameraInfo> camera_infos = {make_pinhole()};
  std::vector<bagwiz::core::slam::SensorTransform> t_cloud_cams = {{}};
  std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 5.0F}};
  std::vector<TrajectoryPose> trajectory;
  std::shared_ptr<const bagwiz::core::slam::ColorizeGeometry> geometry;
  std::vector<std::unique_ptr<bagwiz::core::slam::MapColorizer>> colorizers;

  RoutingFixture()
  {
    for (int s = 0; s <= 10; ++s) {
      trajectory.push_back(make_pose(s * kSecondNs, static_cast<double>(s)));
    }
    geometry = build_shared_colorize_geometry(points, 1);
    colorizers = build_camera_colorizers(
      camera_infos, t_cloud_cams, 100.0, 1, false, geometry, points, trajectory);
  }
};

TEST(ColorizeOneImage, DecodeFailureReturnsFalseAndFeedsNothing)
{
  RoutingFixture fx;
  const std::vector<std::byte> garbage(16, std::byte{0x42});
  EXPECT_FALSE(
    bagwiz::commands::colorize_one_image(*fx.colorizers[0], nullptr, kImageType, garbage, 0, {}));
  EXPECT_EQ(fx.colorizers[0]->finish().images_used, 0U);
}

TEST(ColorizeOneImage, DirectPathFeedsTheColorizer)
{
  RoutingFixture fx;
  const auto payload = make_image_payload(1);
  EXPECT_TRUE(
    bagwiz::commands::colorize_one_image(*fx.colorizers[0], nullptr, kImageType, payload, 0, {}));
  EXPECT_EQ(fx.colorizers[0]->finish().images_used, 1U);
}

TEST(ColorizeOneImage, BlurPathDispatchesPerBucketAndFlushEmitsTheLast)
{
  RoutingFixture fx;
  // The blur-configured picker the worker/serial shells hand in under
  // --color-keyframe-blur: 2 m gate over the fixture's 1 m/s trajectory.
  bagwiz::core::slam::ColorizeKeyframePicker picker({.min_dist = 2.0, .blur = true}, fx.trajectory);

  // Bucket 1: frames at 0 s and 1 s (equal sharpness; the earlier wins the
  // tie). Nothing reaches the colorizer while the bucket is collecting.
  const auto frame_a = make_image_payload(0);
  const auto frame_b = make_image_payload(1);
  EXPECT_TRUE(
    bagwiz::commands::colorize_one_image(*fx.colorizers[0], &picker, kImageType, frame_a, 0, {}));
  EXPECT_TRUE(
    bagwiz::commands::colorize_one_image(*fx.colorizers[0], &picker, kImageType, frame_b, 0, {}));

  // The frame at 3 s (3 m) opens bucket 2 and dispatches bucket 1's best.
  const auto frame_c = make_image_payload(3);
  EXPECT_TRUE(
    bagwiz::commands::colorize_one_image(*fx.colorizers[0], &picker, kImageType, frame_c, 0, {}));

  // End of stream: the final bucket's frame is flushed to the colorizer.
  bagwiz::commands::colorize_flush_keyframes(*fx.colorizers[0], &picker);

  EXPECT_EQ(fx.colorizers[0]->finish().images_used, 2U);
  EXPECT_EQ(picker.kept(), 2U);
  EXPECT_EQ(picker.skipped(), 1U);
}

TEST(ColorizeFlushKeyframes, NullPickerIsANoop)
{
  RoutingFixture fx;
  bagwiz::commands::colorize_flush_keyframes(*fx.colorizers[0], nullptr);
  EXPECT_EQ(fx.colorizers[0]->finish().images_used, 0U);
}

// No --cam-info entries: every camera auto-resolves (empty map, no error).
TEST(ParseCameraInfoOverrides, EmptyEntriesYieldEmptyMap)
{
  const std::vector<std::string> topics{"/cam0/image_raw", "/cam1/image_raw"};
  const auto parsed = parse_camera_info_overrides({}, topics);
  EXPECT_TRUE(parsed.error.empty());
  EXPECT_TRUE(parsed.by_image_topic.empty());
}

// A subset of cameras may carry an override; the rest are absent from the map
// (they auto-resolve).
TEST(ParseCameraInfoOverrides, SubsetOfCamerasMayBeKeyed)
{
  const std::vector<std::string> topics{"/cam0/image_raw", "/cam1/image_raw"};
  const std::vector<std::string> entries{"/cam1/image_raw=/cam1/info"};
  const auto parsed = parse_camera_info_overrides(entries, topics);
  EXPECT_TRUE(parsed.error.empty());
  ASSERT_EQ(parsed.by_image_topic.size(), 1U);
  EXPECT_EQ(parsed.by_image_topic.at("/cam1/image_raw"), "/cam1/info");
}

// Every camera keyed, in any order relative to the topic list.
TEST(ParseCameraInfoOverrides, AllCamerasKeyedInAnyOrder)
{
  const std::vector<std::string> topics{"/cam0/image_raw", "/cam1/image_raw"};
  const std::vector<std::string> entries{
    "/cam1/image_raw=/cam1/info", "/cam0/image_raw=/cam0/info"};
  const auto parsed = parse_camera_info_overrides(entries, topics);
  EXPECT_TRUE(parsed.error.empty());
  ASSERT_EQ(parsed.by_image_topic.size(), 2U);
  EXPECT_EQ(parsed.by_image_topic.at("/cam0/image_raw"), "/cam0/info");
  EXPECT_EQ(parsed.by_image_topic.at("/cam1/image_raw"), "/cam1/info");
}

// An entry without '=' — and one with an empty half — is malformed.
TEST(ParseCameraInfoOverrides, MalformedEntriesError)
{
  const std::vector<std::string> topics{"/cam0/image_raw"};
  for (const char * bad : {"/cam0/image_raw", "=/cam0/info", "/cam0/image_raw="}) {
    const auto parsed = parse_camera_info_overrides(std::vector<std::string>{bad}, topics);
    EXPECT_NE(parsed.error.find("<image_topic>=<info_topic>"), std::string::npos) << bad;
  }
}

// A key naming no listed camera topic is an error.
TEST(ParseCameraInfoOverrides, UnknownImageTopicErrors)
{
  const std::vector<std::string> topics{"/cam0/image_raw"};
  const std::vector<std::string> entries{"/other/image_raw=/other/info"};
  const auto parsed = parse_camera_info_overrides(entries, topics);
  EXPECT_NE(parsed.error.find("/other/image_raw"), std::string::npos);
}

// The same camera keyed twice is an error, even with an identical value.
TEST(ParseCameraInfoOverrides, DuplicateKeyErrors)
{
  const std::vector<std::string> topics{"/cam0/image_raw"};
  const std::vector<std::string> entries{
    "/cam0/image_raw=/cam0/info", "/cam0/image_raw=/cam0/info"};
  const auto parsed = parse_camera_info_overrides(entries, topics);
  EXPECT_NE(parsed.error.find("more than once"), std::string::npos);
}

}  // namespace
