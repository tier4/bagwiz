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
#include <cmath>
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

// Gaps wider than this many median inter-sample intervals are treated as sensor
// dropouts and left un-interpolated (used when the caller passes no explicit
// threshold). A few-x the typical spacing distinguishes a real dropout from
// ordinary timing jitter.
constexpr double kGapMedianFactor = 3.0;

// Parse a strictly-positive, finite double that consumes the ENTIRE token (no
// trailing junk). Rejects empty, non-numeric, infinite, NaN, and <= 0 values.
bool parse_positive_double(std::string_view token, double & out)
{
  if (token.empty()) {
    return false;
  }
  const std::string buf(token);  // strtod needs a null-terminated string
  errno = 0;
  char * end = nullptr;
  const double v = std::strtod(buf.c_str(), &end);
  if (end != buf.c_str() + buf.size() || errno == ERANGE) {
    return false;
  }
  if (!std::isfinite(v) || v <= 0.0) {
    return false;
  }
  out = v;
  return true;
}

// Interpolate a pose at `stamp_ns` between `a` and `b`. Position is linear;
// orientation is shortest-path SLERP (the second quaternion is negated when the
// dot product is negative, since q and -q are the same rotation) with a linear
// fallback for nearly-parallel quaternions to avoid dividing by ~0.
TrajectoryPose interpolate_pose(
  const TrajectoryPose & a, const TrajectoryPose & b, double t, std::int64_t stamp_ns)
{
  TrajectoryPose out;
  out.timestamp_ns = stamp_ns;
  out.tx = a.tx + t * (b.tx - a.tx);
  out.ty = a.ty + t * (b.ty - a.ty);
  out.tz = a.tz + t * (b.tz - a.tz);

  const double ax = a.qx;
  const double ay = a.qy;
  const double az = a.qz;
  const double aw = a.qw;
  double bx = b.qx;
  double by = b.qy;
  double bz = b.qz;
  double bw = b.qw;
  double dot = ax * bx + ay * by + az * bz + aw * bw;
  if (dot < 0.0) {
    bx = -bx;
    by = -by;
    bz = -bz;
    bw = -bw;
    dot = -dot;
  }

  double s0 = 0.0;
  double s1 = 0.0;
  constexpr double kParallelDot = 0.9995;
  if (dot > kParallelDot) {
    // Nearly identical orientations: SLERP degenerates, fall back to LERP.
    s0 = 1.0 - t;
    s1 = t;
  } else {
    const double theta0 = std::acos(dot);
    const double sin0 = std::sin(theta0);
    const double theta = theta0 * t;
    s0 = std::sin(theta0 - theta) / sin0;
    s1 = std::sin(theta) / sin0;
  }
  double qx = s0 * ax + s1 * bx;
  double qy = s0 * ay + s1 * by;
  double qz = s0 * az + s1 * bz;
  double qw = s0 * aw + s1 * bw;
  const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (norm > 0.0) {
    qx /= norm;
    qy /= norm;
    qz /= norm;
    qw /= norm;
  }
  out.qx = qx;
  out.qy = qy;
  out.qz = qz;
  out.qw = qw;
  return out;
}

// Median of the consecutive inter-sample intervals (seconds). Robust to large
// gaps, so it captures the "typical" spacing for the dropout threshold.
// Precondition: poses.size() >= 2.
double median_interval_seconds(std::span<const TrajectoryPose> poses)
{
  std::vector<double> dts;
  dts.reserve(poses.size() - 1);
  for (std::size_t i = 1; i < poses.size(); ++i) {
    dts.push_back(static_cast<double>(poses[i].timestamp_ns - poses[i - 1].timestamp_ns) / 1e9);
  }
  std::sort(dts.begin(), dts.end());
  const std::size_t m = dts.size();
  if (m % 2 == 1) {
    return dts[m / 2];
  }
  return 0.5 * (dts[m / 2 - 1] + dts[m / 2]);
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

std::optional<UpsampleSpec> parse_upsample_spec(std::string_view text)
{
  // Trim ASCII whitespace from both ends.
  while (!text.empty() && std::isspace(static_cast<unsigned char>(text.front())) != 0) {
    text.remove_prefix(1);
  }
  while (!text.empty() && std::isspace(static_cast<unsigned char>(text.back())) != 0) {
    text.remove_suffix(1);
  }
  if (text.empty()) {
    return std::nullopt;
  }

  UpsampleMode mode = UpsampleMode::kFrequencyHz;
  std::string_view number = text;
  const char last = text.back();
  if (last == 'x' || last == 'X') {
    mode = UpsampleMode::kMultiplier;
    number = text.substr(0, text.size() - 1);
  } else if (text.size() >= 2) {
    const char c1 =
      static_cast<char>(std::tolower(static_cast<unsigned char>(text[text.size() - 2])));
    const char c0 = static_cast<char>(std::tolower(static_cast<unsigned char>(last)));
    if (c1 == 'h' && c0 == 'z') {
      mode = UpsampleMode::kFrequencyHz;  // explicit "hz" suffix == bare number
      number = text.substr(0, text.size() - 2);
    }
  }

  double value = 0.0;
  if (!parse_positive_double(number, value)) {
    return std::nullopt;
  }
  return UpsampleSpec{mode, value};
}

TrajectoryUpsampleResult upsample_trajectory(
  std::span<const TrajectoryPose> poses, const UpsampleSpec & spec, double gap_threshold_s)
{
  TrajectoryUpsampleResult result;

  const std::size_t n = poses.size();
  // Two strictly time-increasing samples are the minimum to define a rate and
  // bracket an interpolation; otherwise hand the input back unchanged.
  if (n < 2) {
    result.poses.assign(poses.begin(), poses.end());
    return result;
  }
  const std::int64_t t_start = poses.front().timestamp_ns;
  const std::int64_t t_end = poses.back().timestamp_ns;
  if (t_end <= t_start) {
    result.poses.assign(poses.begin(), poses.end());
    return result;
  }

  const double span_s = static_cast<double>(t_end - t_start) / 1e9;
  result.native_rate_hz = static_cast<double>(n - 1) / span_s;

  const double target_hz =
    (spec.mode == UpsampleMode::kMultiplier) ? spec.value * result.native_rate_hz : spec.value;
  result.target_rate_hz = target_hz;

  // Never down-sample: at or below the native rate, return the input unchanged
  // (the caller decides whether to warn).
  if (target_hz <= result.native_rate_hz) {
    result.poses.assign(poses.begin(), poses.end());
    return result;
  }

  const double threshold_s =
    (gap_threshold_s > 0.0) ? gap_threshold_s : kGapMedianFactor * median_interval_seconds(poses);
  result.gap_threshold_s = threshold_s;

  // Densify by SUBDIVIDING each consecutive segment instead of laying a fresh
  // grid from t_start: every input pose is emitted verbatim (timestamp and value
  // bit-exact) and bounds a segment, so the original samples are always retained
  // and the inserted points only fill the spans between them.
  result.poses.reserve(static_cast<std::size_t>(span_s * target_hz) + n + 1);
  result.poses.push_back(poses.front());

  for (std::size_t i = 0; i + 1 < n; ++i) {
    const TrajectoryPose & a = poses[i];
    const TrajectoryPose & b = poses[i + 1];
    const std::int64_t dt = b.timestamp_ns - a.timestamp_ns;
    const double dt_s = static_cast<double>(dt) / 1e9;

    // Equal sub-intervals that bring this segment to ~target_hz; >= 1 means the
    // segment itself with no interior points (subdivisions - 1 of those).
    std::int64_t subdivisions = std::llround(dt_s * target_hz);
    if (subdivisions < 1) {
      subdivisions = 1;
    }

    if (dt_s > threshold_s) {
      // An over-threshold segment is a sensor dropout: keep the real endpoints
      // but fabricate nothing across it. Report the interior points declined.
      if (subdivisions > 1) {
        ++result.skipped_gap_count;
        result.skipped_point_count += subdivisions - 1;
      }
    } else {
      for (std::int64_t j = 1; j < subdivisions; ++j) {
        const std::int64_t tk =
          a.timestamp_ns + std::llround(static_cast<double>(dt) * j / subdivisions);
        // Skip a rounded stamp that would collide with either endpoint.
        if (tk <= a.timestamp_ns || tk >= b.timestamp_ns) {
          continue;
        }
        const double alpha = static_cast<double>(tk - a.timestamp_ns) / static_cast<double>(dt);
        result.poses.push_back(interpolate_pose(a, b, alpha, tk));
      }
    }
    result.poses.push_back(b);  // segment endpoint, verbatim
  }

  result.resampled = true;
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
