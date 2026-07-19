// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "cam_info_common.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include "bagwiz/core/introspection/introspection_loader.hpp"

#include <gtest/gtest.h>
#include <rcutils/allocator.h>
#include <rcutils/error_handling.h>
#include <rmw/rmw.h>
#include <rmw/serialized_message.h>
#include <rmw/types.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <string>
#include <vector>

namespace
{

using bagwiz::commands::camera_info_topic_names;
using bagwiz::commands::CameraInfoProcessor;
using bagwiz::commands::deserialize_camera_info;
using bagwiz::commands::kCameraInfoType;
using bagwiz::commands::serialize_camera_info;
using bagwiz::commands::take_rmw_error;
using bagwiz::commands::validate_camera_info_targets;
using bagwiz::core::pipeline::TransformAction;
using bagwiz::io::TopicInfo;

constexpr const char * kLogger = "bagwiz.test.cam_info_common";

TopicInfo topic(const std::string & name, const std::string & type)
{
  TopicInfo t;
  t.name = name;
  t.type = type;
  return t;
}

// A CameraInfoProcessor whose mutation only counts invocations, so the shared
// skeleton can be exercised without a real rmw typesupport.
class CountingProcessor : public CameraInfoProcessor
{
public:
  using CameraInfoProcessor::CameraInfoProcessor;

  [[nodiscard]] int mutations() const { return mutations_; }

protected:
  void mutate(const std::string &, sensor_msgs::msg::CameraInfo &) const override { ++mutations_; }

private:
  mutable int mutations_ = 0;
};

TEST(CameraInfoTopicNames, FiltersAndPreservesBagOrder)
{
  const std::vector<TopicInfo> topics{
    topic("/lidar/points", "sensor_msgs/msg/PointCloud2"),
    topic("/cam1/camera_info", kCameraInfoType),
    topic("/image", "sensor_msgs/msg/Image"),
    topic("/cam2/camera_info", kCameraInfoType),
  };
  const auto names = camera_info_topic_names(topics);
  EXPECT_EQ(names, (std::vector<std::string>{"/cam1/camera_info", "/cam2/camera_info"}));
}

TEST(CameraInfoTopicNames, EmptyWhenNoCameraInfoTopic)
{
  const std::vector<TopicInfo> topics{topic("/image", "sensor_msgs/msg/Image")};
  EXPECT_TRUE(camera_info_topic_names(topics).empty());
}

TEST(ValidateCameraInfoTargets, DedupsAndPreservesRequestOrder)
{
  const std::vector<TopicInfo> topics{
    topic("/cam1/camera_info", kCameraInfoType),
    topic("/cam2/camera_info", kCameraInfoType),
  };
  const auto result = validate_camera_info_targets(
    topics, {"/cam2/camera_info", "/cam1/camera_info", "/cam2/camera_info"}, "/tmp/in.bag",
    kLogger);
  EXPECT_TRUE(result.all_valid);
  EXPECT_EQ(result.topics, (std::vector<std::string>{"/cam2/camera_info", "/cam1/camera_info"}));
}

TEST(ValidateCameraInfoTargets, MissingTopicFailsButKeepsValidOnes)
{
  const std::vector<TopicInfo> topics{topic("/cam1/camera_info", kCameraInfoType)};
  const auto result =
    validate_camera_info_targets(topics, {"/cam1/camera_info", "/nope"}, "/tmp/in.bag", kLogger);
  EXPECT_FALSE(result.all_valid);
  EXPECT_EQ(result.topics, (std::vector<std::string>{"/cam1/camera_info"}));
}

TEST(ValidateCameraInfoTargets, WrongTypeFails)
{
  const std::vector<TopicInfo> topics{topic("/image", "sensor_msgs/msg/Image")};
  const auto result = validate_camera_info_targets(topics, {"/image"}, "/tmp/in.bag", kLogger);
  EXPECT_FALSE(result.all_valid);
  EXPECT_TRUE(result.topics.empty());
}

TEST(ValidateCameraInfoTargets, RepeatedMissingTopicIsOneFailure)
{
  // A duplicated request is validated on its first occurrence only; the second
  // occurrence must not add it to the targets nor change the outcome.
  const std::vector<TopicInfo> topics{topic("/cam1/camera_info", kCameraInfoType)};
  const auto result = validate_camera_info_targets(
    topics, {"/nope", "/nope", "/cam1/camera_info"}, "/tmp/in.bag", kLogger);
  EXPECT_FALSE(result.all_valid);
  EXPECT_EQ(result.topics, (std::vector<std::string>{"/cam1/camera_info"}));
}

TEST(TakeRmwError, ReturnsMessageThenResets)
{
  RCUTILS_SET_ERROR_MSG("boom");
  EXPECT_EQ(take_rmw_error(), "boom");
  EXPECT_EQ(take_rmw_error(), "(no error message)");
}

TEST(CameraInfoProcessorSkeleton, RoutesEverythingVerbatimAndTransforms)
{
  CountingProcessor processor({"/ci"}, nullptr);
  const auto emit = processor.route("/anything");
  EXPECT_TRUE(emit.keep);
  EXPECT_EQ(emit.out_topic, "/anything");
  EXPECT_TRUE(processor.transforms());
}

TEST(CameraInfoProcessorSkeleton, NonTargetPassesThroughWithoutDeserializing)
{
  // typesupport is null: reaching the deserialize path would fail, so a clean
  // passthrough proves non-target topics never touch the rmw round-trip.
  CountingProcessor processor({"/ci"}, nullptr);
  const std::vector<std::byte> payload{std::byte{0x01}, std::byte{0x02}};
  std::vector<std::byte> out;
  EXPECT_EQ(processor.transform("/other", payload, out), TransformAction::kPassthrough);
  EXPECT_TRUE(out.empty());
  EXPECT_EQ(processor.mutations(), 0);
}

TEST(CameraInfoProcessorSkeleton, RewrittenCountStartsAtZero)
{
  CountingProcessor processor({"/ci", "/ci2"}, nullptr);
  EXPECT_EQ(processor.rewritten_count("/ci"), 0U);
  EXPECT_EQ(processor.rewritten_count("/ci2"), 0U);
  EXPECT_EQ(processor.rewritten_count("/not-a-target"), 0U);
}

// ---------------------------------------------------------------------------
// serialize_camera_info
//
// The production encoder hand-emits CDR through CdrWriter to skip the
// per-message rmw_serialize cost, so its field order, alignment, and the
// variable-length `d` sequence are only as correct as they are pinned here.
// The cam-info command tests cover it end-to-end through a bag; these exercise
// it directly against rmw, which is the format's reference encoder.
// ---------------------------------------------------------------------------

// A CameraInfo with every field set to a distinct, non-default value so a
// field that is dropped, or written out of order, cannot coincide with the
// expected one. `d` has 5 entries (plumb_bob), the sequence length the
// fixed-size arrays follow.
sensor_msgs::msg::CameraInfo make_full_camera_info()
{
  sensor_msgs::msg::CameraInfo msg;
  msg.header.stamp.sec = 1234;
  msg.header.stamp.nanosec = 567'890'123U;
  msg.header.frame_id = "camera_optical_frame";
  msg.height = 1080;
  msg.width = 1920;
  msg.distortion_model = "plumb_bob";
  msg.d = {-0.31, 0.12, 0.001, -0.002, 0.045};
  msg.k = {1000.5, 0.0, 960.25, 0.0, 1001.5, 540.75, 0.0, 0.0, 1.0};
  msg.r = {1.0, 0.001, 0.002, -0.001, 1.0, 0.003, -0.002, -0.003, 1.0};
  msg.p = {1000.5, 0.0, 960.25, 1.5, 0.0, 1001.5, 540.75, -2.5, 0.0, 0.0, 1.0, 0.25};
  msg.binning_x = 2;
  msg.binning_y = 3;
  msg.roi.x_offset = 16;
  msg.roi.y_offset = 32;
  msg.roi.height = 480;
  msg.roi.width = 640;
  msg.roi.do_rectify = true;
  return msg;
}

// rmw's own encoder, the reference the hand-rolled CdrWriter must agree with.
std::vector<std::byte> rmw_serialize_camera_info(const sensor_msgs::msg::CameraInfo & msg)
{
  auto intro = bagwiz::core::load_introspection(kCameraInfoType);
  EXPECT_TRUE(intro.ok()) << intro.error;

  rmw_serialized_message_t serialized = rmw_get_zero_initialized_serialized_message();
  rcutils_allocator_t alloc = rcutils_get_default_allocator();
  EXPECT_EQ(rmw_serialized_message_init(&serialized, 0, &alloc), RMW_RET_OK);
  EXPECT_EQ(rmw_serialize(&msg, intro.typesupport, &serialized), RMW_RET_OK);
  std::vector<std::byte> out(serialized.buffer_length);
  if (serialized.buffer_length > 0) {
    std::memcpy(out.data(), serialized.buffer, serialized.buffer_length);
  }
  rmw_serialized_message_fini(&serialized);
  return out;
}

void expect_camera_info_eq(
  const sensor_msgs::msg::CameraInfo & actual, const sensor_msgs::msg::CameraInfo & expected)
{
  EXPECT_EQ(actual.header.stamp.sec, expected.header.stamp.sec);
  EXPECT_EQ(actual.header.stamp.nanosec, expected.header.stamp.nanosec);
  EXPECT_EQ(actual.header.frame_id, expected.header.frame_id);
  EXPECT_EQ(actual.height, expected.height);
  EXPECT_EQ(actual.width, expected.width);
  EXPECT_EQ(actual.distortion_model, expected.distortion_model);
  ASSERT_EQ(actual.d.size(), expected.d.size());
  for (std::size_t i = 0; i < expected.d.size(); ++i) {
    EXPECT_DOUBLE_EQ(actual.d[i], expected.d[i]) << "d[" << i << "]";
  }
  for (std::size_t i = 0; i < expected.k.size(); ++i) {
    EXPECT_DOUBLE_EQ(actual.k[i], expected.k[i]) << "k[" << i << "]";
  }
  for (std::size_t i = 0; i < expected.r.size(); ++i) {
    EXPECT_DOUBLE_EQ(actual.r[i], expected.r[i]) << "r[" << i << "]";
  }
  for (std::size_t i = 0; i < expected.p.size(); ++i) {
    EXPECT_DOUBLE_EQ(actual.p[i], expected.p[i]) << "p[" << i << "]";
  }
  EXPECT_EQ(actual.binning_x, expected.binning_x);
  EXPECT_EQ(actual.binning_y, expected.binning_y);
  EXPECT_EQ(actual.roi.x_offset, expected.roi.x_offset);
  EXPECT_EQ(actual.roi.y_offset, expected.roi.y_offset);
  EXPECT_EQ(actual.roi.height, expected.roi.height);
  EXPECT_EQ(actual.roi.width, expected.roi.width);
  EXPECT_EQ(actual.roi.do_rectify, expected.roi.do_rectify);
}

TEST(SerializeCameraInfo, MatchesRmwByteForByte)
{
  const auto msg = make_full_camera_info();
  EXPECT_EQ(serialize_camera_info(msg), rmw_serialize_camera_info(msg));
}

TEST(SerializeCameraInfo, RoundTripPreservesTheMutationAndEveryOtherField)
{
  // The shape every cam-info rewrite takes: read the bag's CDR, deserialize,
  // mutate, re-serialize. Start from rmw's bytes so the input is real wire
  // format rather than the encoder's own output.
  const auto original = make_full_camera_info();
  const auto wire = rmw_serialize_camera_info(original);

  auto intro = bagwiz::core::load_introspection(kCameraInfoType);
  ASSERT_TRUE(intro.ok()) << intro.error;

  auto decoded = deserialize_camera_info(wire, intro.typesupport, "/camera/camera_info");
  expect_camera_info_eq(decoded, original);

  // `p` is what `cam-info recompute-p` rewrites; frame_id stands in for the
  // header edits `cam-info replace` makes.
  decoded.p = {2000.0, 0.0, 100.5, 0.0, 0.0, 2001.0, 200.5, 0.0, 0.0, 0.0, 1.0, 0.0};
  decoded.header.frame_id = "rectified_frame";

  const auto round_tripped = serialize_camera_info(decoded);
  const auto final_msg =
    deserialize_camera_info(round_tripped, intro.typesupport, "/camera/camera_info");
  expect_camera_info_eq(final_msg, decoded);
  EXPECT_NE(final_msg.p, original.p);
}

TEST(SerializeCameraInfo, HandlesAnEmptyDistortionVector)
{
  // `d` is the message's only variable-length field, so an empty sequence is
  // the case where a mis-emitted length would corrupt everything after it.
  auto msg = make_full_camera_info();
  msg.d.clear();
  msg.distortion_model = "";

  EXPECT_EQ(serialize_camera_info(msg), rmw_serialize_camera_info(msg));

  auto intro = bagwiz::core::load_introspection(kCameraInfoType);
  ASSERT_TRUE(intro.ok()) << intro.error;
  const auto decoded =
    deserialize_camera_info(serialize_camera_info(msg), intro.typesupport, "/camera/camera_info");
  expect_camera_info_eq(decoded, msg);
}

TEST(SerializeCameraInfo, HandlesAnEightElementDistortionVector)
{
  // rational_polynomial calibrations carry 8 coefficients; the sequence length
  // shifts the alignment of every field that follows.
  auto msg = make_full_camera_info();
  msg.distortion_model = "rational_polynomial";
  msg.d = {-0.31, 0.12, 0.001, -0.002, 0.045, 0.5, -0.25, 0.125};

  EXPECT_EQ(serialize_camera_info(msg), rmw_serialize_camera_info(msg));

  auto intro = bagwiz::core::load_introspection(kCameraInfoType);
  ASSERT_TRUE(intro.ok()) << intro.error;
  const auto decoded =
    deserialize_camera_info(serialize_camera_info(msg), intro.typesupport, "/camera/camera_info");
  expect_camera_info_eq(decoded, msg);
}

}  // namespace
