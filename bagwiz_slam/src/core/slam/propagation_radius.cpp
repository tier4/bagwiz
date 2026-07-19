// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/propagation_radius.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace bagwiz::core::slam
{

std::optional<double> propagation_radius_from_spacings(std::span<const float> spacings)
{
  std::vector<float> scratch(spacings.begin(), spacings.end());
  if (scratch.empty()) {
    return std::nullopt;
  }
  const auto mid = scratch.begin() + static_cast<std::ptrdiff_t>(scratch.size() / 2);
  std::nth_element(scratch.begin(), mid, scratch.end());
  const double median = static_cast<double>(*mid);
  if (!std::isfinite(median) || median <= 0.0) {
    return std::nullopt;
  }
  return std::clamp(4.0 * median, 0.05, 5.0);
}

}  // namespace bagwiz::core::slam
