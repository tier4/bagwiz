// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "cam_info_common.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include <gtest/gtest.h>
#include <rcutils/error_handling.h>

#include <cstddef>
#include <span>
#include <string>
#include <vector>

namespace
{

using bagwiz::commands::camera_info_topic_names;
using bagwiz::commands::CameraInfoProcessor;
using bagwiz::commands::kCameraInfoType;
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

}  // namespace
