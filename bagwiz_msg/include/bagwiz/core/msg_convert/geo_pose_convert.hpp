// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__MSG_CONVERT__GEO_POSE_CONVERT_HPP_
#define BAGWIZ__CORE__MSG_CONVERT__GEO_POSE_CONVERT_HPP_

#include "bagwiz/core/cdr_walker/value.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

// Whitelisted message-type conversions in the "geo" (position) family,
// backing `bagwiz convert msg geo`. The first (and currently only)
// route family is sensor_msgs/msg/NavSatFix -> a geometry_msgs pose type,
// projecting WGS84 latitude/longitude/altitude into a Cartesian frame
// (ENU local tangent plane or UTM) via GeographicLib.
//
// The conversion is intentionally one-directional: NavSatFix carries
// absolute geographic coordinates that project cleanly into Cartesian, but
// the reverse needs the original datum/zone (absent from a pose bag) and
// would have to synthesise NavSatFix status fields. Reverse routes are left
// for a follow-up.
namespace bagwiz::core::msg_convert
{

// Canonical ROS 2 type names for the route endpoints.
inline constexpr const char * kNavSatFixType = "sensor_msgs/msg/NavSatFix";
inline constexpr const char * kPoseWithCovarianceStampedType =
  "geometry_msgs/msg/PoseWithCovarianceStamped";
inline constexpr const char * kPoseStampedType = "geometry_msgs/msg/PoseStamped";

// Target Cartesian coordinate system the pose position is expressed in.
//   kEnu  - local East-North-Up tangent plane around an origin (metres).
//           Requires an origin (datum).
//   kUtm  - Universal Transverse Mercator easting/northing (metres). The
//           zone/hemisphere are derived from the longitude; an optional
//           origin shifts the result so coordinates stay small.
enum class GeoCrs { kEnu, kUtm };

// WGS84 reference point used as the ENU origin (and optional UTM offset).
struct GeoOrigin
{
  double lat = 0.0;  // degrees
  double lon = 0.0;  // degrees
  double alt = 0.0;  // metres
};

// One whitelisted conversion route, resolved from a (from, to) type pair.
struct GeoRoute
{
  std::string from_ros_type;
  std::string to_ros_type;
  // True when the target message has a covariance field the source's
  // position_covariance can populate (PoseWithCovarianceStamped). False for
  // PoseStamped, which has no covariance and therefore drops it.
  bool target_has_covariance = false;
};

// The snake_case choices exposed on the CLI as `--src` / `--dst`. Kept here
// (next to the type mapping) so the command's CLI::IsMember list and the
// resolver never drift apart.
const std::vector<std::string> & from_snake_choices();
const std::vector<std::string> & to_snake_choices();

// Map a snake_case CLI choice to its ROS 2 type name. std::nullopt for an
// unknown choice (the CLI's IsMember check makes that unreachable in normal
// use, but callers validate defensively).
std::optional<std::string> from_snake_to_ros_type(std::string_view snake);
std::optional<std::string> to_snake_to_ros_type(std::string_view snake);

// Inverse of from_snake_to_ros_type / to_snake_to_ros_type: a ROS type name
// back to its snake_case label, for error messages that start from a topic's
// actual type. std::nullopt when the type is not part of any geo route.
std::optional<std::string> ros_type_to_snake(std::string_view ros_type);

// Look up the whitelisted route for a (source, target) ROS type pair.
// std::nullopt when the pair is not permitted. The lookup keys on ROS type
// names (not snake labels) so the `--topic` path — which reads the source
// type from the bag rather than `--src` — shares the same whitelist.
std::optional<GeoRoute> resolve_route(std::string_view from_ros_type, std::string_view to_ros_type);

// The fields lifted out of a decoded NavSatFix message. Only what the pose
// conversion needs: the timestamp (reused on the target), the geographic
// position, and the 3x3 position covariance. The source frame_id is
// intentionally not captured — the target's frame_id is set from config.
struct NavSatSample
{
  std::int32_t stamp_sec = 0;
  std::uint32_t stamp_nanosec = 0;
  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
  // Row-major 3x3 ENU covariance as carried by NavSatFix.position_covariance.
  std::array<double, 9> position_covariance{};
};

// Pull a NavSatSample out of a decoded sensor_msgs/msg/NavSatFix Value tree.
// Returns std::nullopt when the tree does not have the expected shape
// (missing latitude/longitude/altitude). Tolerant of float32-vs-float64 and
// int32-vs-uint32 stamp encodings, matching extract_tf_message's policy.
std::optional<NavSatSample> extract_nav_sat_fix(const cdr_walker::Value & message);

// Resolved configuration for a converter instance. `origin` must be present
// for kEnu; it is optional for kUtm (absent = no offset). `frame_id` is
// written onto every target header. `target_ros_type` selects which pose
// message is built; `target_has_covariance` mirrors the route.
struct GeoConvertConfig
{
  GeoCrs crs = GeoCrs::kEnu;
  std::optional<GeoOrigin> origin;
  std::string frame_id;
  std::string target_ros_type;
  bool target_has_covariance = false;
};

// Projects NavSatSamples into serialized target-type CDR payloads. Holds the
// GeographicLib projector and the target's introspection typesupport, both
// built once at construction so per-message work is just project + serialize.
//
// Stateful and thread-incompatible; construct one per conversion run.
class GeoPoseConverter
{
public:
  // Throws std::invalid_argument when the config is inconsistent (kEnu with
  // no origin, unknown target type) and std::runtime_error when the target
  // type's introspection typesupport cannot be loaded.
  explicit GeoPoseConverter(const GeoConvertConfig & config);
  ~GeoPoseConverter();

  GeoPoseConverter(const GeoPoseConverter &) = delete;
  GeoPoseConverter & operator=(const GeoPoseConverter &) = delete;
  GeoPoseConverter(GeoPoseConverter &&) noexcept;
  GeoPoseConverter & operator=(GeoPoseConverter &&) noexcept;

  // Project `sample` and serialize it as the configured target type. Throws
  // std::runtime_error on a projection or serialization failure.
  [[nodiscard]] std::vector<std::byte> convert(const NavSatSample & sample) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace bagwiz::core::msg_convert

#endif  // BAGWIZ__CORE__MSG_CONVERT__GEO_POSE_CONVERT_HPP_
