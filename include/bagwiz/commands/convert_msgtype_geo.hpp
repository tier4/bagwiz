// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__CONVERT_MSGTYPE_GEO_HPP_
#define BAGWIZ__COMMANDS__CONVERT_MSGTYPE_GEO_HPP_

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace bagwiz::commands
{

// Parsed arguments for `bagwiz convert msgtype geo`. The command rewrites a
// rosbag, converting the message type of selected topics from a geographic
// source (sensor_msgs/msg/NavSatFix) into a geometry_msgs pose type, projecting
// WGS84 lat/lon/alt into the chosen Cartesian frame (ENU or UTM). Every other
// topic is copied verbatim.
//
// Topic selection:
//   - `topics` non-empty: convert exactly those topics. `dst` is required;
//     `src` is ignored. All named topics must share one message type.
//   - `topics` empty: `src` and `dst` are required; every topic whose type
//     matches `src` is converted.
struct ConvertMsgtypeGeoArgs
{
  std::filesystem::path input_path;
  std::string src;                      // snake_case source choice; empty allowed when topics given
  std::string dst;                      // snake_case target choice; required
  std::vector<std::string> topics;      // explicit topic selection; empty = by-type
  std::string crs = "enu";              // "enu" | "utm"; defaults to enu
  std::optional<std::string> origin;    // "lat,lon,alt" WGS84 datum / offset
  std::optional<std::string> frame_id;  // overrides the default world-frame name
  std::optional<std::filesystem::path> output_path;  // empty = in-place rewrite
  bool overwrite = false;
};

// Run the conversion. Returns the process exit code: 0 on success, 1 on any
// error (bag open failure, empty/ambiguous selection, unsupported route,
// invalid origin, decode/serialize failure, or I/O error). Kept as a free
// function in its own translation unit so the ConvertCommand dispatcher in
// convert.cpp stays small; declared here so convert.cpp can call it.
int run_convert_msgtype_geo(const ConvertMsgtypeGeoArgs & args);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__CONVERT_MSGTYPE_GEO_HPP_
