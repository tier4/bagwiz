// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__VIDEO__FRAME_RATE_HPP_
#define BAGWIZ__CORE__VIDEO__FRAME_RATE_HPP_

#include <cstdint>

namespace bagwiz::core::video
{

// A video frame rate as an exact rational (num / den frames per second) — the
// form libav's AVStream / AVCodecContext time bases consume.
struct FrameRate
{
  int num = 0;
  int den = 1;
};

// Defaults reused by callers and tests.
inline constexpr int kDefaultFps = 10;
inline constexpr int kMinFps = 1;
inline constexpr int kMaxFps = 240;

// Derive a constant frame rate from a topic's first and last message
// timestamps and message count, used when the user does not pass --fps.
//
//   fps = (count - 1) / ((last_ns - first_ns) / 1e9)
//
// i.e. the average spacing of the recorded messages, so a constant-rate encode
// spans the same wall-clock duration as the recording. The result is rounded to
// milli-fps, reduced, and clamped to [kMinFps, kMaxFps].
//
// Degenerate inputs fall back to kDefaultFps: fewer than two messages (no
// interval to measure) or a non-positive span (all messages share a timestamp).
FrameRate derive_frame_rate(std::int64_t first_ns, std::int64_t last_ns, std::uint64_t count);

}  // namespace bagwiz::core::video

#endif  // BAGWIZ__CORE__VIDEO__FRAME_RATE_HPP_
