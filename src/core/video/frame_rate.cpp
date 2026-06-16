// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/video/frame_rate.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>

namespace bagwiz::core::video
{

FrameRate derive_frame_rate(std::int64_t first_ns, std::int64_t last_ns, std::uint64_t count)
{
  if (count < 2 || last_ns <= first_ns) {
    return FrameRate{kDefaultFps, 1};
  }

  // Subtract in unsigned arithmetic: the guard above ensures last_ns > first_ns,
  // so the result is a correct positive value and signed-overflow UB is avoided
  // for extreme timestamps.
  const double span_s =
    static_cast<double>(
      static_cast<std::uint64_t>(last_ns) - static_cast<std::uint64_t>(first_ns)) /
    1e9;
  const double fps = static_cast<double>(count - 1) / span_s;

  // Clamp the rate to the supported range *before* rounding: std::lround has
  // undefined behavior when its argument exceeds LONG_MAX, which a corrupt bag
  // (huge count over a sub-nanosecond span) could otherwise trigger.
  const double fps_clamped =
    std::clamp(fps, static_cast<double>(kMinFps), static_cast<double>(kMaxFps));

  // Represent as milli-fps so non-integer rates (e.g. 9.8 fps) survive as an
  // exact rational, then reduce. The integer clamp is a redundant safety net.
  constexpr int kMilli = 1000;
  int num = static_cast<int>(std::lround(fps_clamped * kMilli));
  int den = kMilli;
  num = std::clamp(num, kMinFps * kMilli, kMaxFps * kMilli);

  const int g = std::gcd(num, den);
  if (g > 0) {
    num /= g;
    den /= g;
  }
  return FrameRate{num, den};
}

}  // namespace bagwiz::core::video
