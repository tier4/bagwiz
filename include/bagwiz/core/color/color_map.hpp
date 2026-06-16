// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__COLOR__COLOR_MAP_HPP_
#define BAGWIZ__CORE__COLOR__COLOR_MAP_HPP_

#include <array>
#include <cstdint>

namespace bagwiz::core::color
{

enum class ColorMapName { kJet, kTurbo, kViridis, kGrayscale, kRainbow };

struct Rgb
{
  std::uint8_t r = 0;
  std::uint8_t g = 0;
  std::uint8_t b = 0;
};

struct ColorMap
{
  std::array<Rgb, 256> table{};
};

/// Build a 256-entry lookup table for the requested colormap.
ColorMap make_color_map(ColorMapName name);

/// Return the colormap entry at `index`.
Rgb apply(const ColorMap & map, std::uint8_t index);

/// Linearly map `value` from [`min`, `max`] to [0, 255], clamping to the range.
/// Returns 0 when `max <= min`.
std::uint8_t normalize(float value, float min, float max);

}  // namespace bagwiz::core::color

#endif  // BAGWIZ__CORE__COLOR__COLOR_MAP_HPP_
