// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__IMAGE__SRGB_HPP_
#define BAGWIZ__CORE__IMAGE__SRGB_HPP_

#include <cstdint>

// sRGB <-> linear-light conversion for 8-bit channel values (IEC 61966-2-1).
// Camera images arrive gamma-encoded; averaging, exposure ratios, and any
// other radiometric arithmetic are only meaningful on the linear-light values
// underneath, so consumers decode with srgb_u8_to_linear, compute in linear
// light, and re-encode with linear_to_srgb_u8. Both directions are exact
// table lookups at runtime (no pow() in hot loops) and round-trip losslessly:
// linear_to_srgb_u8(srgb_u8_to_linear(v)) == v for every 8-bit value.
namespace bagwiz::core::image
{

// Linear-light value of one 8-bit sRGB channel, in [0, 1]. Exact IEC
// 61966-2-1 decode (the 12.92 linear toe below 0.04045, the 2.4-exponent
// power segment above), served from a 256-entry table.
[[nodiscard]] double srgb_u8_to_linear(std::uint8_t value);

// Nearest 8-bit sRGB encoding of a linear-light value: equivalent to
// round(srgb_encode(linear) * 255) with the continuous IEC 61966-2-1 encode,
// implemented as a threshold search over 255 precomputed half-step decode
// values. Inputs at or below 0 (including NaN) return 0; inputs at or above
// 1 return 255.
[[nodiscard]] std::uint8_t linear_to_srgb_u8(double linear);

}  // namespace bagwiz::core::image

#endif  // BAGWIZ__CORE__IMAGE__SRGB_HPP_
