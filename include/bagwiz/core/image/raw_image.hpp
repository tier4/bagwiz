// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__IMAGE__RAW_IMAGE_HPP_
#define BAGWIZ__CORE__IMAGE__RAW_IMAGE_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>

namespace bagwiz::core::image
{

// A borrowed view of one decoded sensor_msgs/msg/Image. `data` points into the
// caller-owned CDR payload passed to extract_raw_image() and is valid only as
// long as that payload outlives the view — no pixel bytes are copied. `step` is
// the row stride in bytes as recorded in the message and may exceed
// width * channels when rows carry trailing padding.
struct RawImageView
{
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t step = 0;
  std::string encoding;             // ROS encoding string, e.g. "bgr8" / "rgb8"
  std::span<const std::byte> data;  // step * height bytes, borrowed from payload
  // header.stamp as sec * 1e9 + nanosec. 0 when the publisher left it unset.
  std::int64_t header_stamp_ns = 0;
};

// Outcome of extract_raw_image(). On success `image` holds the view and `error`
// is empty; on failure `image` is empty and `error` carries a human-readable
// reason suitable for logging. Never throws.
struct RawImageResult
{
  std::optional<RawImageView> image;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return image.has_value() && error.empty(); }
};

// Parse a CDR-serialized sensor_msgs/msg/Image payload and return a borrowed
// view of its dimensions, encoding, and pixel data. The payload must outlive
// the returned view (zero-copy). A truncated or malformed payload yields an
// error result rather than throwing.
[[nodiscard]] RawImageResult extract_raw_image(std::span<const std::byte> payload);

}  // namespace bagwiz::core::image

#endif  // BAGWIZ__CORE__IMAGE__RAW_IMAGE_HPP_
