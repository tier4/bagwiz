// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__IMAGE__COMPRESSED_IMAGE_HPP_
#define BAGWIZ__CORE__IMAGE__COMPRESSED_IMAGE_HPP_

#include <cstddef>
#include <optional>
#include <span>
#include <string>

namespace bagwiz::core::image
{

// A borrowed view of one sensor_msgs/msg/CompressedImage. `data` points into the
// caller-owned CDR payload passed to extract_compressed_image() and is valid
// only as long as that payload outlives the view — no bytes are copied. `format`
// is the message's format string (e.g. "jpeg", "png", or the image_transport
// "rgb8; jpeg compressed bgr8" convention); `data` holds the compressed bytes
// (the JPEG / PNG / ... bitstream).
struct CompressedImageView
{
  std::string format;
  std::span<const std::byte> data;  // compressed bitstream, borrowed from payload
};

// Outcome of extract_compressed_image(). On success `image` holds the view and
// `error` is empty; on failure `image` is empty and `error` carries a
// human-readable reason suitable for logging. Never throws.
struct CompressedImageResult
{
  std::optional<CompressedImageView> image;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return image.has_value() && error.empty(); }
};

// Parse a CDR-serialized sensor_msgs/msg/CompressedImage payload and return a
// borrowed view of its format string and compressed data. The payload must
// outlive the returned view (zero-copy). A truncated or malformed payload yields
// an error result rather than throwing.
[[nodiscard]] CompressedImageResult extract_compressed_image(std::span<const std::byte> payload);

}  // namespace bagwiz::core::image

#endif  // BAGWIZ__CORE__IMAGE__COMPRESSED_IMAGE_HPP_
