// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/trajectory.hpp"

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ios>
#include <istream>
#include <optional>
#include <ostream>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace bagwiz::core
{

namespace
{

// Trim ASCII whitespace from both ends in place.
void trim(std::string & s)
{
  const auto not_space = [](unsigned char c) { return std::isspace(c) == 0; };
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
  s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
}

// Parse "<sec>.<nsec>" with integer arithmetic so the value round-trips
// bit-exactly with write_tum. Returns false on malformed input.
//
// Accepted forms:
//   "1773211197.937418279"  -> ns = 1773211197 * 1e9 + 937418279
//   "1773211197"            -> ns = 1773211197 * 1e9 (no fractional part)
//   "1773211197.93"         -> ns = 1773211197 * 1e9 + 930000000 (right-pad to 9)
// Rejected:
//   negative timestamps, scientific notation, multiple dots, non-digit chars,
//   fractional parts longer than 9 digits (would silently truncate sub-ns
//   information without warning).
bool parse_tum_timestamp_ns(std::string_view token, std::int64_t & out_ns)
{
  if (token.empty()) {
    return false;
  }
  const auto dot = token.find('.');
  const std::string_view sec_part = (dot == std::string_view::npos) ? token : token.substr(0, dot);
  const std::string_view nsec_part =
    (dot == std::string_view::npos) ? std::string_view{} : token.substr(dot + 1);
  if (sec_part.empty()) {
    return false;
  }
  if (nsec_part.size() > 9) {
    return false;
  }
  // A second '.' anywhere in the fractional part is malformed.
  if (nsec_part.find('.') != std::string_view::npos) {
    return false;
  }
  for (const char c : sec_part) {
    if (std::isdigit(static_cast<unsigned char>(c)) == 0) {
      return false;
    }
  }
  for (const char c : nsec_part) {
    if (std::isdigit(static_cast<unsigned char>(c)) == 0) {
      return false;
    }
  }
  std::int64_t sec = 0;
  for (const char c : sec_part) {
    sec = sec * 10 + (c - '0');
  }
  // Right-pad to exactly 9 digits, then parse as an integer.
  std::array<char, 10> nsec_buf{};
  for (std::size_t i = 0; i < 9; ++i) {
    nsec_buf[i] = (i < nsec_part.size()) ? nsec_part[i] : '0';
  }
  std::int64_t nsec = 0;
  for (std::size_t i = 0; i < 9; ++i) {
    nsec = nsec * 10 + (nsec_buf[i] - '0');
  }
  out_ns = sec * 1'000'000'000LL + nsec;
  return true;
}

// Build a tf2::Transform from a geometry_msgs Transform (translation +
// quaternion rotation, ROS / Hamilton convention).
tf2::Transform to_tf2(const geometry_msgs::msg::Transform & t)
{
  tf2::Transform out;
  out.setOrigin(tf2::Vector3(t.translation.x, t.translation.y, t.translation.z));
  out.setRotation(tf2::Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w));
  return out;
}

// Build a tf2::Transform from a geometry_msgs Pose (position + orientation).
tf2::Transform to_tf2(const geometry_msgs::msg::Pose & p)
{
  tf2::Transform out;
  out.setOrigin(tf2::Vector3(p.position.x, p.position.y, p.position.z));
  out.setRotation(
    tf2::Quaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w));
  return out;
}

bool parse_double(const std::string & token, double & out)
{
  if (token.empty()) {
    return false;
  }
  errno = 0;
  char * end = nullptr;
  const double v = std::strtod(token.c_str(), &end);
  if (end == token.c_str() || *end != '\0' || errno == ERANGE) {
    return false;
  }
  out = v;
  return true;
}

}  // namespace

void write_tum(std::ostream & os, std::span<const TrajectoryPose> poses)
{
  const auto prev_flags = os.flags();
  const auto prev_prec = os.precision();
  os.setf(std::ios::fixed, std::ios::floatfield);
  os.precision(9);
  for (const auto & p : poses) {
    // Format the timestamp from the integer ns value directly. Going
    // through double would round to the nearest representable value;
    // around year-2026 magnitudes (~1.77e18 ns) the double ULP is ~256,
    // so 9-digit output silently drifts from the source header.stamp.
    const std::int64_t ns = p.timestamp_ns;
    const std::int64_t sec = ns / 1'000'000'000LL;
    const std::int64_t nsec = ns % 1'000'000'000LL;
    char ts_buf[32];
    std::snprintf(ts_buf, sizeof(ts_buf), "%" PRId64 ".%09" PRId64, sec, nsec);
    os << ts_buf << ' ' << p.tx << ' ' << p.ty << ' ' << p.tz << ' ' << p.qx << ' ' << p.qy << ' '
       << p.qz << ' ' << p.qw << '\n';
  }
  os.flags(prev_flags);
  os.precision(prev_prec);
}

TrajectoryReadResult read_tum(std::istream & is)
{
  TrajectoryReadResult result;
  std::string line;
  while (std::getline(is, line)) {
    trim(line);
    if (line.empty() || line.front() == '#') {
      continue;
    }
    std::istringstream ls(line);
    std::vector<std::string> tokens;
    tokens.reserve(8);
    for (std::string tok; ls >> tok;) {
      tokens.push_back(std::move(tok));
    }
    if (tokens.size() != 8) {
      ++result.skipped_lines;
      continue;
    }
    TrajectoryPose p;
    if (!parse_tum_timestamp_ns(tokens[0], p.timestamp_ns)) {
      ++result.skipped_lines;
      continue;
    }
    bool ok = true;
    ok = ok && parse_double(tokens[1], p.tx);
    ok = ok && parse_double(tokens[2], p.ty);
    ok = ok && parse_double(tokens[3], p.tz);
    ok = ok && parse_double(tokens[4], p.qx);
    ok = ok && parse_double(tokens[5], p.qy);
    ok = ok && parse_double(tokens[6], p.qz);
    ok = ok && parse_double(tokens[7], p.qw);
    if (!ok) {
      ++result.skipped_lines;
      continue;
    }
    result.poses.push_back(p);
  }
  return result;
}

geometry_msgs::msg::TransformStamped pose_to_transform_stamped(
  const TrajectoryPose & pose, std::string_view frame_id, std::string_view child_frame_id)
{
  geometry_msgs::msg::TransformStamped ts;
  ts.header.stamp.sec = static_cast<std::int32_t>(pose.timestamp_ns / 1'000'000'000LL);
  ts.header.stamp.nanosec = static_cast<std::uint32_t>(pose.timestamp_ns % 1'000'000'000LL);
  ts.header.frame_id.assign(frame_id.begin(), frame_id.end());
  ts.child_frame_id.assign(child_frame_id.begin(), child_frame_id.end());
  ts.transform.translation.x = pose.tx;
  ts.transform.translation.y = pose.ty;
  ts.transform.translation.z = pose.tz;
  ts.transform.rotation.x = pose.qx;
  ts.transform.rotation.y = pose.qy;
  ts.transform.rotation.z = pose.qz;
  ts.transform.rotation.w = pose.qw;
  return ts;
}

geometry_msgs::msg::Pose compose_trajectory_pose(
  const std::optional<geometry_msgs::msg::Transform> & from_header,
  const geometry_msgs::msg::Pose & body_pose,
  const std::optional<geometry_msgs::msg::Transform> & body_to)
{
  // No bridge on either side: return the pose verbatim. Routing through
  // tf2 would renormalise the quaternion and perturb the translation in
  // the last ULPs, breaking the "values as stored in the bag" guarantee
  // for a no-transform dump.
  if (!from_header.has_value() && !body_to.has_value()) {
    return body_pose;
  }

  tf2::Transform t = to_tf2(body_pose);  // T_header_body
  if (from_header.has_value()) {
    t = to_tf2(*from_header) * t;  // T_from_header * T_header_body
  }
  if (body_to.has_value()) {
    t = t * to_tf2(*body_to);  // ... * T_body_to
  }

  geometry_msgs::msg::Pose out;
  const tf2::Vector3 o = t.getOrigin();
  out.position.x = o.x();
  out.position.y = o.y();
  out.position.z = o.z();
  const tf2::Quaternion q = t.getRotation();
  out.orientation.x = q.x();
  out.orientation.y = q.y();
  out.orientation.z = q.z();
  out.orientation.w = q.w();
  return out;
}

}  // namespace bagwiz::core
