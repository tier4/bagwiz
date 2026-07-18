// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/msg_convert/geo_pose_convert.hpp"

#include "bagwiz/core/cdr_walker/cdr_writer.hpp"
#include "bagwiz/core/introspection/introspection_loader.hpp"

#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/UTMUPS.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <rmw/rmw.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace bagwiz::core::msg_convert
{

namespace
{

namespace cdr = bagwiz::core::cdr_walker;

// --- decoded-Value field access (mirrors tf_value_extract.cpp's helpers; kept
// local to avoid coupling the two modules through a private header) ---

// cppcheck-suppress passedByValue  // string_view is the canonical by-value idiom
const cdr::Value * find_field(const cdr::Object & obj, std::string_view name) noexcept
{
  for (const auto & entry : obj.fields) {
    if (entry.first == name) {
      return &entry.second;
    }
  }
  return nullptr;
}

const cdr::Object * as_object(const cdr::Value & v) noexcept
{
  return std::get_if<cdr::Object>(&v.v);
}

const cdr::Sequence * as_sequence(const cdr::Value & v) noexcept
{
  return std::get_if<cdr::Sequence>(&v.v);
}

// Read any IDL float width (float32 / float64) as a double. Returns false when
// the value is not a floating-point primitive.
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

// Read a named float field of `obj` into `out`. False when missing or not a
// float primitive.
// cppcheck-suppress passedByValue  // string_view is the canonical by-value idiom
bool read_double_field(const cdr::Object & obj, std::string_view name, double & out) noexcept
{
  const auto * v = find_field(obj, name);
  return v != nullptr && to_double(*v, out);
}

// Read a builtin_interfaces/Time `sec` field, tolerating both int32 (canonical)
// and uint32 (the mcap-ros2-support quirk) encodings. Leaves `out` untouched
// when the field is absent or a different type.
void read_stamp_sec(const cdr::Object & stamp, std::int32_t & out) noexcept
{
  const auto * v = find_field(stamp, "sec");
  if (v == nullptr) {
    return;
  }
  if (const auto * s32 = std::get_if<std::int32_t>(&v->v)) {
    out = *s32;
  } else if (const auto * u32 = std::get_if<std::uint32_t>(&v->v)) {
    out = static_cast<std::int32_t>(*u32);
  }
}

void read_stamp_nanosec(const cdr::Object & stamp, std::uint32_t & out) noexcept
{
  const auto * v = find_field(stamp, "nanosec");
  if (v == nullptr) {
    return;
  }
  if (const auto * u32 = std::get_if<std::uint32_t>(&v->v)) {
    out = *u32;
  } else if (const auto * s32 = std::get_if<std::int32_t>(&v->v)) {
    out = static_cast<std::uint32_t>(*s32);
  }
}

// Lift header.stamp.{sec,nanosec} out of a decoded message root, if present.
void parse_header_stamp(const cdr::Object & root, NavSatSample & sample) noexcept
{
  const auto * header_v = find_field(root, "header");
  if (header_v == nullptr) {
    return;
  }
  const auto * header = as_object(*header_v);
  if (header == nullptr) {
    return;
  }
  const auto * stamp_v = find_field(*header, "stamp");
  if (stamp_v == nullptr) {
    return;
  }
  const auto * stamp = as_object(*stamp_v);
  if (stamp == nullptr) {
    return;
  }
  read_stamp_sec(*stamp, sample.stamp_sec);
  read_stamp_nanosec(*stamp, sample.stamp_nanosec);
}

// Lift NavSatFix.position_covariance (float64[9], row-major 3x3) out of a
// decoded message root, if present. Short arrays leave trailing entries zero.
void parse_position_covariance(const cdr::Object & root, NavSatSample & sample) noexcept
{
  const auto * cov_v = find_field(root, "position_covariance");
  if (cov_v == nullptr) {
    return;
  }
  const auto * cov_seq = as_sequence(*cov_v);
  if (cov_seq == nullptr) {
    return;
  }
  const std::size_t n = std::min<std::size_t>(cov_seq->elements.size(), 9);
  for (std::size_t i = 0; i < n; ++i) {
    double value = 0.0;
    if (to_double(cov_seq->elements[i], value)) {
      sample.position_covariance[i] = value;
    }
  }
}

// --- the whitelist + snake<->ROS-type tables ---

struct SnakeType
{
  const char * snake;
  const char * ros_type;
};

const SnakeType kFromTable[] = {
  {"nav_sat_fix", kNavSatFixType},
};

const SnakeType kToTable[] = {
  {"pose_with_covariance_stamped", kPoseWithCovarianceStampedType},
  {"pose_stamped", kPoseStampedType},
};

// Whitelisted (source, target) ROS-type pairs and whether the target carries
// covariance. Adding a route is a one-line edit here.
struct RouteEntry
{
  const char * from_ros_type;
  const char * to_ros_type;
  bool target_has_covariance;
};

const RouteEntry kRoutes[] = {
  {kNavSatFixType, kPoseWithCovarianceStampedType, true},
  {kNavSatFixType, kPoseStampedType, false},
};

std::vector<std::string> snake_choices(std::span<const SnakeType> table)
{
  std::vector<std::string> out;
  out.reserve(table.size());
  for (const auto & e : table) {
    out.emplace_back(e.snake);
  }
  return out;
}

// --- GeographicLib projection wrapper ---

// Projects WGS84 lat/lon/alt to the configured Cartesian frame. Built once and
// reused per message; the per-call cost is one GeographicLib forward solve.
class Projector
{
public:
  static Projector make_enu(const GeoOrigin & origin)
  {
    Projector p;
    p.crs_ = GeoCrs::kEnu;
    p.enu_ = GeographicLib::LocalCartesian(origin.lat, origin.lon, origin.alt);
    return p;
  }

  static Projector make_utm(const std::optional<GeoOrigin> & origin)
  {
    Projector p;
    p.crs_ = GeoCrs::kUtm;
    if (origin.has_value()) {
      // Pin every sample to the origin's zone so easting/northing stay in one
      // frame, then subtract the origin so coordinates stay small.
      int zone = GeographicLib::UTMUPS::STANDARD;
      bool northp = true;
      double e = 0.0;
      double n = 0.0;
      GeographicLib::UTMUPS::Forward(origin->lat, origin->lon, zone, northp, e, n);
      p.utm_zone_ = zone;
      p.utm_northp_ = northp;
      p.utm_off_e_ = e;
      p.utm_off_n_ = n;
      p.utm_off_alt_ = origin->alt;
      p.utm_force_zone_ = true;
    }
    return p;
  }

  [[nodiscard]] std::array<double, 3> project(double lat, double lon, double alt) const
  {
    if (crs_ == GeoCrs::kEnu) {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      enu_.Forward(lat, lon, alt, x, y, z);
      return {x, y, z};
    }
    int zone = utm_force_zone_ ? utm_zone_ : GeographicLib::UTMUPS::STANDARD;
    bool northp = utm_northp_;
    double e = 0.0;
    double n = 0.0;
    const int setzone = utm_force_zone_ ? utm_zone_ : GeographicLib::UTMUPS::STANDARD;
    GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, e, n, setzone);
    return {e - utm_off_e_, n - utm_off_n_, alt - utm_off_alt_};
  }

private:
  GeoCrs crs_ = GeoCrs::kEnu;
  GeographicLib::LocalCartesian enu_;
  bool utm_force_zone_ = false;
  int utm_zone_ = GeographicLib::UTMUPS::STANDARD;
  bool utm_northp_ = true;
  double utm_off_e_ = 0.0;
  double utm_off_n_ = 0.0;
  double utm_off_alt_ = 0.0;
};

// --- direct CDR serialization ---
//
// The target messages (PoseStamped / PoseWithCovarianceStamped) have fixed CDR
// layouts, so we emit them with the project's CdrWriter instead of going
// through rmw_serialize. This removes the per-sample rmw_serialize cost and
// the associated RMW buffer management. Byte identity against rmw_serialize is
// verified by unit tests.

void write_pose_header(
  cdr::CdrWriter & writer, std::int32_t sec, std::uint32_t nsec, const std::string & frame_id)
{
  writer.write_i32(sec);
  writer.write_u32(nsec);
  writer.write_string(frame_id);
}

void write_pose_body(cdr::CdrWriter & writer, const std::array<double, 3> & xyz)
{
  // geometry_msgs/Point position
  writer.write_f64(xyz[0]);
  writer.write_f64(xyz[1]);
  writer.write_f64(xyz[2]);
  // geometry_msgs/Quaternion orientation (identity)
  writer.write_f64(0.0);
  writer.write_f64(0.0);
  writer.write_f64(0.0);
  writer.write_f64(1.0);
}

std::vector<std::byte> serialize_pose_stamped(
  const std::array<double, 3> & xyz, std::int32_t sec, std::uint32_t nsec,
  const std::string & frame_id)
{
  cdr::CdrWriter writer;
  write_pose_header(writer, sec, nsec, frame_id);
  write_pose_body(writer, xyz);
  return writer.take();
}

std::vector<std::byte> serialize_pose_with_covariance_stamped(
  const std::array<double, 3> & xyz, const std::array<double, 36> & covariance, std::int32_t sec,
  std::uint32_t nsec, const std::string & frame_id)
{
  cdr::CdrWriter writer;
  write_pose_header(writer, sec, nsec, frame_id);
  write_pose_body(writer, xyz);
  for (const double v : covariance) {
    writer.write_f64(v);
  }
  return writer.take();
}

}  // namespace

const std::vector<std::string> & from_snake_choices()
{
  static const std::vector<std::string> kChoices = snake_choices(kFromTable);
  return kChoices;
}

const std::vector<std::string> & to_snake_choices()
{
  static const std::vector<std::string> kChoices = snake_choices(kToTable);
  return kChoices;
}

// cppcheck-suppress passedByValue  // string_view is the canonical by-value idiom
std::optional<std::string> from_snake_to_ros_type(std::string_view snake)
{
  for (const auto & e : kFromTable) {
    if (snake == e.snake) {
      return std::string(e.ros_type);
    }
  }
  return std::nullopt;
}

// cppcheck-suppress passedByValue  // string_view is the canonical by-value idiom
std::optional<std::string> to_snake_to_ros_type(std::string_view snake)
{
  for (const auto & e : kToTable) {
    if (snake == e.snake) {
      return std::string(e.ros_type);
    }
  }
  return std::nullopt;
}

// cppcheck-suppress passedByValue  // string_view is the canonical by-value idiom
std::optional<std::string> ros_type_to_snake(std::string_view ros_type)
{
  for (const auto & e : kFromTable) {
    if (ros_type == e.ros_type) {
      return std::string(e.snake);
    }
  }
  for (const auto & e : kToTable) {
    if (ros_type == e.ros_type) {
      return std::string(e.snake);
    }
  }
  return std::nullopt;
}

// cppcheck-suppress passedByValue  // string_view is the canonical by-value idiom
std::optional<GeoRoute> resolve_route(std::string_view from_ros_type, std::string_view to_ros_type)
{
  for (const auto & r : kRoutes) {
    if (from_ros_type == r.from_ros_type && to_ros_type == r.to_ros_type) {
      return GeoRoute{
        std::string(r.from_ros_type), std::string(r.to_ros_type), r.target_has_covariance};
    }
  }
  return std::nullopt;
}

std::optional<NavSatSample> extract_nav_sat_fix(const cdr_walker::Value & message)
{
  const auto * root = as_object(message);
  if (root == nullptr) {
    return std::nullopt;
  }

  NavSatSample sample;

  // latitude / longitude / altitude are required for any pose projection.
  if (
    !read_double_field(*root, "latitude", sample.latitude) ||
    !read_double_field(*root, "longitude", sample.longitude) ||
    !read_double_field(*root, "altitude", sample.altitude)) {
    return std::nullopt;
  }

  parse_header_stamp(*root, sample);
  parse_position_covariance(*root, sample);

  return sample;
}

// --- GeoPoseConverter ---

struct GeoPoseConverter::Impl
{
  GeoConvertConfig config;
  Projector projector;
  const rosidl_message_type_support_t * typesupport = nullptr;
};

namespace
{

// Map NavSatFix's row-major 3x3 ENU position covariance into the upper-left
// position block of a 6x6 pose covariance (x,y,z,roll,pitch,yaw). Rotation
// terms stay zero.
void fill_pose_covariance(std::array<double, 36> & out, const std::array<double, 9> & in)
{
  out.fill(0.0);
  for (std::size_t i = 0; i < 3; ++i) {
    for (std::size_t j = 0; j < 3; ++j) {
      out[(i * 6) + j] = in[(i * 3) + j];
    }
  }
}

}  // namespace

GeoPoseConverter::GeoPoseConverter(const GeoConvertConfig & config)
: impl_(std::make_unique<Impl>())
{
  impl_->config = config;

  if (config.crs == GeoCrs::kEnu) {
    if (!config.origin.has_value()) {
      throw std::invalid_argument("ENU conversion requires an origin (datum)");
    }
    impl_->projector = Projector::make_enu(*config.origin);
  } else {
    impl_->projector = Projector::make_utm(config.origin);
  }

  if (
    config.target_ros_type != kPoseWithCovarianceStampedType &&
    config.target_ros_type != kPoseStampedType) {
    throw std::invalid_argument("unsupported geo target type: " + config.target_ros_type);
  }

  auto intro = core::load_introspection(config.target_ros_type);
  if (!intro.ok()) {
    throw std::runtime_error(
      "could not load introspection typesupport for " + config.target_ros_type + ": " +
      intro.error);
  }
  impl_->typesupport = intro.typesupport;
}

GeoPoseConverter::~GeoPoseConverter() = default;
GeoPoseConverter::GeoPoseConverter(GeoPoseConverter &&) noexcept = default;
GeoPoseConverter & GeoPoseConverter::operator=(GeoPoseConverter &&) noexcept = default;

std::vector<std::byte> GeoPoseConverter::convert(const NavSatSample & sample) const
{
  const auto xyz = impl_->projector.project(sample.latitude, sample.longitude, sample.altitude);
  const std::string & frame_id = impl_->config.frame_id;

  if (impl_->config.target_ros_type == kPoseWithCovarianceStampedType) {
    std::array<double, 36> cov{};
    fill_pose_covariance(cov, sample.position_covariance);
    return serialize_pose_with_covariance_stamped(
      xyz, cov, sample.stamp_sec, sample.stamp_nanosec, frame_id);
  }

  return serialize_pose_stamped(xyz, sample.stamp_sec, sample.stamp_nanosec, frame_id);
}

}  // namespace bagwiz::core::msg_convert
