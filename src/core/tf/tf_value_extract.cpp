// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tf/tf_value_extract.hpp"

#include "bagwiz/core/cdr_walker/value.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/header.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace bagwiz::core
{

namespace
{

namespace cdr = bagwiz::core::cdr_walker;

// cppcheck-suppress passedByValue
const cdr::Value * find_field(const cdr::Object & obj, std::string_view name) noexcept
{
  for (const auto & entry : obj.fields) {
    if (entry.first == name) {
      return &entry.second;
    }
  }
  return nullptr;
}

const cdr::Object * find_object(const cdr::Value & v) noexcept
{
  return std::get_if<cdr::Object>(&v.v);
}

bool to_double(const cdr::Value & v, double & out) noexcept
{
  if (const auto * d = std::get_if<double>(&v.v)) {
    out = *d;
    return true;
  }
  if (const auto * f = std::get_if<float>(&v.v)) {
    out = static_cast<double>(*f);
    return true;
  }
  return false;
}

bool read_xyz(const cdr::Object & obj, double & x, double & y, double & z) noexcept
{
  const auto * fx = find_field(obj, "x");
  const auto * fy = find_field(obj, "y");
  const auto * fz = find_field(obj, "z");
  if (fx == nullptr || fy == nullptr || fz == nullptr) {
    return false;
  }
  return to_double(*fx, x) && to_double(*fy, y) && to_double(*fz, z);
}

bool read_xyzw(const cdr::Object & obj, double & x, double & y, double & z, double & w) noexcept
{
  const auto * fw = find_field(obj, "w");
  if (fw == nullptr || !read_xyz(obj, x, y, z)) {
    return false;
  }
  return to_double(*fw, w);
}

// Fill a std_msgs::msg::Header from a decoded Object. Returns false on
// any missing or wrong-shaped field; the caller skips the element in
// that case.
bool fill_std_msgs_header(const cdr::Object & header_obj, std_msgs::msg::Header & out)
{
  const auto * stamp_v = find_field(header_obj, "stamp");
  if (stamp_v == nullptr) {
    return false;
  }
  const auto * stamp = find_object(*stamp_v);
  if (stamp == nullptr) {
    return false;
  }
  const auto * sec_v = find_field(*stamp, "sec");
  const auto * nsec_v = find_field(*stamp, "nanosec");
  if (sec_v == nullptr || nsec_v == nullptr) {
    return false;
  }
  if (const auto * sec_i32 = std::get_if<std::int32_t>(&sec_v->v)) {
    out.stamp.sec = *sec_i32;
  } else if (const auto * sec_u32 = std::get_if<std::uint32_t>(&sec_v->v)) {
    out.stamp.sec = static_cast<std::int32_t>(*sec_u32);
  } else {
    return false;
  }
  if (const auto * nsec_u32 = std::get_if<std::uint32_t>(&nsec_v->v)) {
    out.stamp.nanosec = *nsec_u32;
  } else if (const auto * nsec_i32 = std::get_if<std::int32_t>(&nsec_v->v)) {
    out.stamp.nanosec = static_cast<std::uint32_t>(*nsec_i32);
  } else {
    return false;
  }

  if (const auto * fid_v = find_field(header_obj, "frame_id")) {
    if (const auto * fid = std::get_if<std::string>(&fid_v->v)) {
      out.frame_id = *fid;
    }
  }
  return true;
}

bool fill_transform_stamped(const cdr::Object & ts_obj, geometry_msgs::msg::TransformStamped & out)
{
  // header { stamp { sec, nanosec }, frame_id }
  const auto * header_v = find_field(ts_obj, "header");
  if (header_v == nullptr) {
    return false;
  }
  const auto * header = find_object(*header_v);
  if (header == nullptr) {
    return false;
  }

  if (!fill_std_msgs_header(*header, out.header)) {
    return false;
  }

  if (const auto * cf_v = find_field(ts_obj, "child_frame_id")) {
    if (const auto * cf = std::get_if<std::string>(&cf_v->v)) {
      out.child_frame_id = *cf;
    }
  }

  // transform { translation, rotation }
  const auto * tf_v = find_field(ts_obj, "transform");
  if (tf_v == nullptr) {
    return false;
  }
  const auto * tf_obj = find_object(*tf_v);
  if (tf_obj == nullptr) {
    return false;
  }
  const auto * trans_v = find_field(*tf_obj, "translation");
  const auto * rot_v = find_field(*tf_obj, "rotation");
  if (trans_v == nullptr || rot_v == nullptr) {
    return false;
  }
  const auto * trans_obj = find_object(*trans_v);
  const auto * rot_obj = find_object(*rot_v);
  if (trans_obj == nullptr || rot_obj == nullptr) {
    return false;
  }
  if (!read_xyz(
        *trans_obj, out.transform.translation.x, out.transform.translation.y,
        out.transform.translation.z)) {
    return false;
  }
  return read_xyzw(
    *rot_obj, out.transform.rotation.x, out.transform.rotation.y, out.transform.rotation.z,
    out.transform.rotation.w);
}

bool fill_pose_object(const cdr::Object & pose_obj, geometry_msgs::msg::Pose & out)
{
  const auto * pos_v = find_field(pose_obj, "position");
  const auto * ori_v = find_field(pose_obj, "orientation");
  if (pos_v == nullptr || ori_v == nullptr) {
    return false;
  }
  const auto * pos_obj = find_object(*pos_v);
  const auto * ori_obj = find_object(*ori_v);
  if (pos_obj == nullptr || ori_obj == nullptr) {
    return false;
  }
  if (!read_xyz(*pos_obj, out.position.x, out.position.y, out.position.z)) {
    return false;
  }
  return read_xyzw(
    *ori_obj, out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w);
}

bool fill_pose_stamped_root(const cdr::Object & root, geometry_msgs::msg::PoseStamped & out)
{
  const auto * header_v = find_field(root, "header");
  if (header_v == nullptr) {
    return false;
  }
  const auto * header_obj = find_object(*header_v);
  if (header_obj == nullptr) {
    return false;
  }
  if (!fill_std_msgs_header(*header_obj, out.header)) {
    return false;
  }
  const auto * pose_v = find_field(root, "pose");
  if (pose_v == nullptr) {
    return false;
  }
  const auto * pose_obj = find_object(*pose_v);
  if (pose_obj == nullptr) {
    return false;
  }
  return fill_pose_object(*pose_obj, out.pose);
}

bool fill_pose_with_covariance_pose_only(
  const cdr::Object & pwc_obj, geometry_msgs::msg::PoseWithCovariance & out)
{
  const auto * inner_pose_v = find_field(pwc_obj, "pose");
  if (inner_pose_v == nullptr) {
    return false;
  }
  const auto * inner_pose_obj = find_object(*inner_pose_v);
  if (inner_pose_obj == nullptr) {
    return false;
  }
  return fill_pose_object(*inner_pose_obj, out.pose);
}

bool fill_odometry_root(const cdr::Object & root, nav_msgs::msg::Odometry & out)
{
  const auto * header_v = find_field(root, "header");
  if (header_v == nullptr) {
    return false;
  }
  const auto * header_obj = find_object(*header_v);
  if (header_obj == nullptr) {
    return false;
  }
  if (!fill_std_msgs_header(*header_obj, out.header)) {
    return false;
  }

  if (const auto * cf_v = find_field(root, "child_frame_id")) {
    if (const auto * s = std::get_if<std::string>(&cf_v->v)) {
      out.child_frame_id = *s;
    }
  }

  const auto * pose_v = find_field(root, "pose");
  if (pose_v == nullptr) {
    return false;
  }
  const auto * pose_obj = find_object(*pose_v);
  if (pose_obj == nullptr) {
    return false;
  }
  return fill_pose_with_covariance_pose_only(*pose_obj, out.pose);
}

bool fill_pose_with_covariance_stamped_root(
  const cdr::Object & root, geometry_msgs::msg::PoseWithCovarianceStamped & out)
{
  const auto * header_v = find_field(root, "header");
  if (header_v == nullptr) {
    return false;
  }
  const auto * header_obj = find_object(*header_v);
  if (header_obj == nullptr) {
    return false;
  }
  if (!fill_std_msgs_header(*header_obj, out.header)) {
    return false;
  }
  const auto * pwc_v = find_field(root, "pose");
  if (pwc_v == nullptr) {
    return false;
  }
  const auto * pwc_obj = find_object(*pwc_v);
  if (pwc_obj == nullptr) {
    return false;
  }
  return fill_pose_with_covariance_pose_only(*pwc_obj, out.pose);
}

std::optional<geometry_msgs::msg::PoseStamped> extract_pose_stamped_impl(
  const cdr_walker::Value & message)
{
  const auto * root = find_object(message);
  if (root == nullptr) {
    return std::nullopt;
  }
  geometry_msgs::msg::PoseStamped out;
  if (!fill_pose_stamped_root(*root, out)) {
    return std::nullopt;
  }
  return out;
}

std::optional<geometry_msgs::msg::PoseWithCovarianceStamped>
extract_pose_with_covariance_stamped_impl(const cdr_walker::Value & message)
{
  const auto * root = find_object(message);
  if (root == nullptr) {
    return std::nullopt;
  }
  geometry_msgs::msg::PoseWithCovarianceStamped out;
  if (!fill_pose_with_covariance_stamped_root(*root, out)) {
    return std::nullopt;
  }
  return out;
}

std::optional<nav_msgs::msg::Odometry> extract_odometry_impl(const cdr_walker::Value & message)
{
  const auto * root = find_object(message);
  if (root == nullptr) {
    return std::nullopt;
  }
  nav_msgs::msg::Odometry out;
  if (!fill_odometry_root(*root, out)) {
    return std::nullopt;
  }
  return out;
}

}  // namespace

std::vector<geometry_msgs::msg::TransformStamped> extract_tf_message(
  const cdr_walker::Value & message)
{
  std::vector<geometry_msgs::msg::TransformStamped> out;
  const auto * root = find_object(message);
  if (root == nullptr) {
    return out;
  }
  const auto * transforms_v = find_field(*root, "transforms");
  if (transforms_v == nullptr) {
    return out;
  }
  const auto * seq = std::get_if<cdr::Sequence>(&transforms_v->v);
  if (seq == nullptr) {
    return out;
  }
  out.reserve(seq->elements.size());
  for (const auto & elem : seq->elements) {
    const auto * elem_obj = find_object(elem);
    if (elem_obj == nullptr) {
      continue;
    }
    geometry_msgs::msg::TransformStamped ts;
    if (fill_transform_stamped(*elem_obj, ts)) {
      out.push_back(std::move(ts));
    }
  }
  return out;
}

std::optional<geometry_msgs::msg::PoseStamped> extract_pose_stamped_message(
  const cdr_walker::Value & message)
{
  return extract_pose_stamped_impl(message);
}

std::optional<geometry_msgs::msg::PoseWithCovarianceStamped>
extract_pose_with_covariance_stamped_message(const cdr_walker::Value & message)
{
  return extract_pose_with_covariance_stamped_impl(message);
}

std::optional<nav_msgs::msg::Odometry> extract_odometry_message(const cdr_walker::Value & message)
{
  return extract_odometry_impl(message);
}

}  // namespace bagwiz::core
