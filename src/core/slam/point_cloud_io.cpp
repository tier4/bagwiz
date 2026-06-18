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

void write_ply(
  std::ostream & os, std::span<const std::array<float, 3>> points,
  std::span<const float> intensities)
{
  const bool with_intensity = !intensities.empty() && intensities.size() == points.size();

  os << "ply\n"
     << "format binary_little_endian 1.0\n"
     << "element vertex " << points.size() << '\n'
     << "property float x\n"
     << "property float y\n"
     << "property float z\n";
  if (with_intensity) {
    os << "property float intensity\n";
  }
  os << "end_header\n";

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
