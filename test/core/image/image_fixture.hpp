// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef CORE__IMAGE__IMAGE_FIXTURE_HPP_
#define CORE__IMAGE__IMAGE_FIXTURE_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace bagwiz::test
{

// Encode a solid `w`x`h` image of color (r, g, b) into a still-image bitstream
// using libav. `format` selects the codec: "jpeg" -> MJPEG (lossy), "png" ->
// PNG (lossless). Returns the encoded bytes, or an empty vector if the codec is
// unavailable in this FFmpeg build or encoding fails. Used to synthesize the
// CompressedImage payloads the decoder and `generate video` tests consume.
std::vector<std::byte> encode_still_image(
  const std::string & format, std::uint32_t w, std::uint32_t h, std::uint8_t r, std::uint8_t g,
  std::uint8_t b);

}  // namespace bagwiz::test

#endif  // CORE__IMAGE__IMAGE_FIXTURE_HPP_
