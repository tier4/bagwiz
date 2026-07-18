// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__TF__TF_VALUE_EXTRACT_HPP_
#define BAGWIZ__CORE__TF__TF_VALUE_EXTRACT_HPP_

#include "bagwiz/core/cdr_walker/value.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <optional>
#include <vector>

// Bridge between the schema-driven decoder (Phases C/D) and tf2's
// BufferCore: walk a decoded tf2_msgs/msg/TFMessage Value tree and emit
// a vector of geometry_msgs::msg::TransformStamped suitable for
// `tf2::BufferCore::setTransform`.
//
// The TF buffer expects the C++ struct (header-only definition from
// geometry_msgs); without this helper, callers either had to use the
// rosidl introspection path (bringing the tf2_msgs typesupport `.so`
// into the runtime dep set) or open-code the field walk inline.
namespace bagwiz::core
{

// Extract every TransformStamped from a decoded tf2_msgs/msg/TFMessage.
// Returns an empty vector when the Value does not have the expected
// shape (top-level Object with a `transforms` Sequence).
//
// Tolerant of writer quirks: accepts float32 or float64 for translation
// / rotation components, and accepts either int32 or uint32 for the
// header.stamp.sec field (matching the Python mcap-ros2-support
// reference, which inadvertently emits sec as uint32).
std::vector<geometry_msgs::msg::TransformStamped> extract_tf_message(
  const cdr_walker::Value & message);

// Decode a single geometry_msgs/msg/PoseStamped from a schema-decoded
// message Value. Returns std::nullopt when the tree does not match.
std::optional<geometry_msgs::msg::PoseStamped> extract_pose_stamped_message(
  const cdr_walker::Value & message);

// Decode geometry_msgs/msg/PoseWithCovarianceStamped. Only header and
// pose.pose (inner geometry_msgs/Pose) are required for trajectory use.
std::optional<geometry_msgs::msg::PoseWithCovarianceStamped>
extract_pose_with_covariance_stamped_message(const cdr_walker::Value & message);

// Decode nav_msgs/msg/Odometry (header, child_frame_id, pose.pose).
// Twist is ignored by callers that only need trajectory pose samples.
std::optional<nav_msgs::msg::Odometry> extract_odometry_message(const cdr_walker::Value & message);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__TF__TF_VALUE_EXTRACT_HPP_
