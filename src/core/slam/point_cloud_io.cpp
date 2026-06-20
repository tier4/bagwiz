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
#include <cstring>
#include <ostream>
#include <span>

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
}  // namespace

void write_pcd(
  std::ostream & os, std::span<const std::array<float, 3>> points,
  std::span<const float> intensities)
{
  const bool with_intensity = !intensities.empty() && intensities.size() == points.size();
  const char * const intensity_field = with_intensity ? " intensity" : "";
  const char * const intensity_size = with_intensity ? " 4" : "";
  const char * const intensity_type = with_intensity ? " F" : "";
  const char * const intensity_count = with_intensity ? " 1" : "";

  // Standard PCD v0.7 binary header. WIDTH/POINTS carry the count, HEIGHT 1
  // marks an unorganized cloud, and the identity VIEWPOINT keeps points in the
  // world frame. The body that follows is tightly packed little-endian float32.
  os << "# .PCD v0.7 - Point Cloud Data file format\n"
     << "VERSION 0.7\n"
     << "FIELDS x y z" << intensity_field << '\n'
     << "SIZE 4 4 4" << intensity_size << '\n'
     << "TYPE F F F" << intensity_type << '\n'
     << "COUNT 1 1 1" << intensity_count << '\n'
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
  }
}

}  // namespace bagwiz::core::slam
