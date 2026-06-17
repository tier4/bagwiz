// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__POINTCLOUD__COLOR_MAPPER_HPP_
#define BAGWIZ__CORE__POINTCLOUD__COLOR_MAPPER_HPP_

#include "bagwiz/core/pointcloud/color_scheme.hpp"

#include <array>
#include <cstdint>

namespace bagwiz::core::pointcloud
{

using BgrColor = std::array<std::uint8_t, 3>;

class ColorMapper
{
public:
  explicit ColorMapper(ColorScheme scheme);

  [[nodiscard]] BgrColor map(double value, double min, double max) const noexcept;

private:
  std::array<BgrColor, 256> lut_;
};

}  // namespace bagwiz::core::pointcloud

#endif  // BAGWIZ__CORE__POINTCLOUD__COLOR_MAPPER_HPP_
