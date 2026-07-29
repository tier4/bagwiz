// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/image/srgb.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace bagwiz::core::image
{

namespace
{

// Continuous IEC 61966-2-1 decode of a normalized sRGB value in [0, 1].
double srgb_to_linear(double srgb)
{
  return srgb <= 0.04045 ? srgb / 12.92 : std::pow((srgb + 0.055) / 1.055, 2.4);
}

std::array<double, 256> build_decode_table()
{
  std::array<double, 256> table{};
  for (std::size_t i = 0; i < table.size(); ++i) {
    table[i] = srgb_to_linear(static_cast<double>(i) / 255.0);
  }
  return table;
}

// thresholds[i] is the linear-light value that encodes to exactly i + 0.5 in
// 8-bit sRGB. The continuous encode is monotone, so round(encode(x) * 255)
// == i exactly when x lands between thresholds[i - 1] and thresholds[i];
// counting the thresholds at or below x therefore reproduces the analytic
// round without evaluating pow() per call.
std::array<double, 255> build_encode_thresholds()
{
  std::array<double, 255> thresholds{};
  for (std::size_t i = 0; i < thresholds.size(); ++i) {
    thresholds[i] = srgb_to_linear((static_cast<double>(i) + 0.5) / 255.0);
  }
  return thresholds;
}

}  // namespace

double srgb_u8_to_linear(std::uint8_t value)
{
  static const std::array<double, 256> table = build_decode_table();
  return table[value];
}

std::uint8_t linear_to_srgb_u8(double linear)
{
  // The negated comparison also routes NaN to 0, keeping the result
  // deterministic for any input.
  if (!(linear > 0.0)) {
    return 0;
  }
  static const std::array<double, 255> thresholds = build_encode_thresholds();
  const auto it = std::upper_bound(thresholds.begin(), thresholds.end(), linear);
  return static_cast<std::uint8_t>(it - thresholds.begin());
}

}  // namespace bagwiz::core::image
