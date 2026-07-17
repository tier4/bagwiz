// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/point_cloud_io.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <istream>
#include <optional>
#include <ostream>
#include <span>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bagwiz::core::slam
{
namespace
{
void write_f32(std::ostream & os, float value)
{
  std::array<char, sizeof(float)> bytes{};
  std::memcpy(bytes.data(), &value, sizeof(float));
  os.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
}

// Pack {r, g, b} into PCL's packed-float rgb convention: the float32 whose bit
// pattern is the uint32 0x00RRGGBB.
float pack_rgb(const std::array<std::uint8_t, 3> & rgb)
{
  const std::uint32_t packed = (static_cast<std::uint32_t>(rgb[0]) << 16) |
                               (static_cast<std::uint32_t>(rgb[1]) << 8) |
                               static_cast<std::uint32_t>(rgb[2]);
  float value = 0.0F;
  std::memcpy(&value, &packed, sizeof(value));
  return value;
}
}  // namespace

void write_pcd(
  std::ostream & os, std::span<const std::array<float, 3>> points,
  std::span<const float> intensities, std::span<const std::array<std::uint8_t, 3>> colors)
{
  const bool with_intensity = !intensities.empty() && intensities.size() == points.size();
  const bool with_rgb = !colors.empty() && colors.size() == points.size();
  const char * const intensity_field = with_intensity ? " intensity" : "";
  const char * const rgb_field = with_rgb ? " rgb" : "";
  const char * const intensity_size = with_intensity ? " 4" : "";
  const char * const rgb_size = with_rgb ? " 4" : "";
  const char * const intensity_type = with_intensity ? " F" : "";
  const char * const rgb_type = with_rgb ? " F" : "";
  const char * const intensity_count = with_intensity ? " 1" : "";
  const char * const rgb_count = with_rgb ? " 1" : "";

  // Standard PCD v0.7 binary header. WIDTH/POINTS carry the count, HEIGHT 1
  // marks an unorganized cloud, and the identity VIEWPOINT keeps points in the
  // world frame. The body that follows is tightly packed little-endian float32
  // (rgb carries the 0x00RRGGBB bits — see pack_rgb above).
  os << "# .PCD v0.7 - Point Cloud Data file format\n"
     << "VERSION 0.7\n"
     << "FIELDS x y z" << intensity_field << rgb_field << '\n'
     << "SIZE 4 4 4" << intensity_size << rgb_size << '\n'
     << "TYPE F F F" << intensity_type << rgb_type << '\n'
     << "COUNT 1 1 1" << intensity_count << rgb_count << '\n'
     << "WIDTH " << points.size() << '\n'
     << "HEIGHT 1\n"
     << "VIEWPOINT 0 0 0 1 0 0 0\n"
     << "POINTS " << points.size() << '\n'
     << "DATA binary\n";

  for (std::size_t i = 0; i < points.size(); ++i) {
    write_f32(os, points[i][0]);
    write_f32(os, points[i][1]);
    write_f32(os, points[i][2]);
    if (with_intensity) {
      write_f32(os, intensities[i]);
    }
    if (with_rgb) {
      write_f32(os, pack_rgb(colors[i]));
    }
  }
}

PcdReadResult read_pcd(std::istream & is)
{
  // Parse the PCD v0.7 header line by line until "DATA binary".
  std::unordered_map<std::string, std::string> header;
  std::string line;
  while (std::getline(is, line)) {
    if (line.empty()) {
      continue;
    }
    if (line[0] == '#') {
      continue;
    }
    const auto space = line.find(' ');
    const std::string key = (space == std::string::npos) ? line : line.substr(0, space);
    const std::string value = (space == std::string::npos) ? std::string{} : line.substr(space + 1);
    header[key] = value;
    if (key == "DATA") {
      break;
    }
  }

  auto get = [&](const char * key) -> std::optional<std::string> {
    const auto it = header.find(key);
    if (it == header.end()) {
      return std::nullopt;
    }
    return it->second;
  };

  const auto data = get("DATA");
  if (!data || *data != "binary") {
    return {false, {}, "PCD must use DATA binary"};
  }
  const auto version = get("VERSION");
  if (!version || *version != "0.7") {
    return {false, {}, "PCD must be version 0.7"};
  }
  const auto fields_str = get("FIELDS");
  if (!fields_str) {
    return {false, {}, "PCD header missing FIELDS"};
  }
  const auto size_str = get("SIZE");
  const auto type_str = get("TYPE");
  const auto count_str = get("COUNT");
  if (!size_str || !type_str || !count_str) {
    return {false, {}, "PCD header missing SIZE/TYPE/COUNT"};
  }

  std::istringstream fields_ss(*fields_str);
  std::istringstream size_ss(*size_str);
  std::istringstream type_ss(*type_str);
  std::istringstream count_ss(*count_str);
  std::vector<std::string> fields;
  std::vector<std::size_t> sizes;
  std::vector<char> types;
  std::vector<std::size_t> counts;
  std::string token;
  while (fields_ss >> token) {
    fields.push_back(token);
    std::size_t sz = 0;
    if (!(size_ss >> sz)) {
      return {false, {}, "PCD SIZE list too short"};
    }
    sizes.push_back(sz);
    char ty = 0;
    if (!(type_ss >> ty)) {
      return {false, {}, "PCD TYPE list too short"};
    }
    types.push_back(ty);
    std::size_t ct = 0;
    if (!(count_ss >> ct)) {
      return {false, {}, "PCD COUNT list too short"};
    }
    counts.push_back(ct);
  }

  if (fields.size() < 3 || fields.size() > 5) {
    return {false, {}, "PCD FIELDS must be x y z [intensity] [rgb]"};
  }
  for (std::size_t i = 0; i < fields.size(); ++i) {
    if (sizes[i] != 4 || types[i] != 'F' || counts[i] != 1) {
      return {false, {}, "PCD fields must be float32 (F, size 4, count 1)"};
    }
  }
  if (fields[0] != "x" || fields[1] != "y" || fields[2] != "z") {
    return {false, {}, "PCD FIELDS must start with x y z"};
  }
  // Optional fields after x y z: `intensity` and/or `rgb`, in that order (the
  // layout write_pcd produces).
  bool with_intensity = false;
  bool with_rgb = false;
  std::size_t next = 3;
  if (next < fields.size() && fields[next] == "intensity") {
    with_intensity = true;
    ++next;
  }
  if (next < fields.size() && fields[next] == "rgb") {
    with_rgb = true;
    ++next;
  }
  if (next != fields.size()) {
    return {false, {}, "PCD FIELDS must be x y z [intensity] [rgb]"};
  }

  const auto width_str = get("WIDTH");
  const auto height_str = get("HEIGHT");
  const auto points_str = get("POINTS");
  if (!width_str || !height_str || !points_str) {
    return {false, {}, "PCD header missing WIDTH/HEIGHT/POINTS"};
  }
  std::size_t width = 0;
  std::size_t height = 0;
  std::size_t num_points = 0;
  {
    std::istringstream ss(*width_str);
    if (!(ss >> width)) {
      return {false, {}, "PCD WIDTH is not an integer"};
    }
  }
  {
    std::istringstream ss(*height_str);
    if (!(ss >> height)) {
      return {false, {}, "PCD HEIGHT is not an integer"};
    }
  }
  {
    std::istringstream ss(*points_str);
    if (!(ss >> num_points)) {
      return {false, {}, "PCD POINTS is not an integer"};
    }
  }
  if (width * height != num_points) {
    return {false, {}, "PCD WIDTH * HEIGHT does not match POINTS"};
  }

  PcdCloud cloud;
  cloud.points.reserve(num_points);
  if (with_intensity) {
    cloud.intensities.reserve(num_points);
  }
  if (with_rgb) {
    cloud.colors.reserve(num_points);
  }
  constexpr std::size_t kFloatBytes = 4;
  const std::size_t point_bytes = fields.size() * kFloatBytes;
  const std::size_t rgb_offset = with_intensity ? 4 : 3;
  std::vector<char> buffer(point_bytes);
  for (std::size_t i = 0; i < num_points; ++i) {
    if (!is.read(buffer.data(), static_cast<std::streamsize>(point_bytes))) {
      return {false, {}, "PCD body truncated"};
    }
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    std::memcpy(&x, buffer.data() + 0 * kFloatBytes, kFloatBytes);
    std::memcpy(&y, buffer.data() + 1 * kFloatBytes, kFloatBytes);
    std::memcpy(&z, buffer.data() + 2 * kFloatBytes, kFloatBytes);
    cloud.points.push_back({x, y, z});
    if (with_intensity) {
      float intensity = 0.0F;
      std::memcpy(&intensity, buffer.data() + 3 * kFloatBytes, kFloatBytes);
      cloud.intensities.push_back(intensity);
    }
    if (with_rgb) {
      // Unpack the packed-float rgb convention (see pack_rgb above).
      std::uint32_t packed = 0;
      std::memcpy(&packed, buffer.data() + rgb_offset * kFloatBytes, kFloatBytes);
      cloud.colors.push_back(
        {static_cast<std::uint8_t>((packed >> 16) & 0xFFU),
         static_cast<std::uint8_t>((packed >> 8) & 0xFFU),
         static_cast<std::uint8_t>(packed & 0xFFU)});
    }
  }

  return {true, std::move(cloud), {}};
}

}  // namespace bagwiz::core::slam
