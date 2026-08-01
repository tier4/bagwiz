// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__IMAGE__PACKED_RASTER_HPP_
#define BAGWIZ__CORE__IMAGE__PACKED_RASTER_HPP_

#include "bagwiz/core/image/image_decoder.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

// One seam that turns any supported image-topic message into a single canonical
// raster, shared by `generate video` and `walk`'s image preview so both accept
// exactly the same inputs. The raster is owned, tightly packed (no row padding),
// and always 8-bit BGR24 — rgb8 source frames are swapped to BGR here so every
// consumer can assume one channel order. libav stays behind the decoder pimpl;
// this header exposes only std types.
namespace bagwiz::core::image
{

// A fully-decoded, owned image in canonical packed 8-bit BGR24 (3 channels,
// interleaved, row stride == width * 3, no trailing padding). `bgr` holds
// exactly width * 3 * height bytes.
struct PackedRaster
{
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::vector<std::byte> bgr;  // packed BGR24, width * 3 * height bytes
  std::string encoding;        // source ROS/codec string (e.g. "rgb8", "jpeg")
  // Source message header.stamp as sec * 1e9 + nanosec. 0 when the publisher
  // left it unset. Carried through so consumers can time-align against other
  // topics by capture time rather than bag record time.
  std::int64_t header_stamp_ns = 0;

  [[nodiscard]] bool empty() const noexcept { return width == 0 || height == 0 || bgr.empty(); }
};

// Outcome of to_packed_raster(). On success `raster` holds the image and `error`
// is empty; on failure `raster` is empty and `error` carries a human-readable
// reason suitable for logging. Never throws.
struct PackedRasterResult
{
  std::optional<PackedRaster> raster;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return raster.has_value() && error.empty(); }
};

// Decode one image-topic message `payload` into a canonical packed BGR24 raster.
// `type` is the topic's message type string: "sensor_msgs/msg/Image" (raw,
// bgr8/rgb8) or "sensor_msgs/msg/CompressedImage" (JPEG/PNG, decoded via libav).
// Any other type, an unsupported raw encoding, or a malformed/corrupt payload
// yields an error result rather than throwing.
[[nodiscard]] PackedRasterResult to_packed_raster(
  std::string_view type, std::span<const std::byte> payload);

// Same as to_packed_raster(type, payload) but compressed payloads decode
// through `decoder`, reusing its FFmpeg contexts across calls. For per-frame
// streams (one decoder per camera/thread); pixels are identical to the
// decoder-less overload.
[[nodiscard]] PackedRasterResult to_packed_raster(
  std::string_view type, std::span<const std::byte> payload, ImageDecoder & decoder);

// True when `type` is a message type to_packed_raster() can decode (raw
// "sensor_msgs/msg/Image" or "sensor_msgs/msg/CompressedImage"). Callers use it
// to gate image-only UI (e.g. walk's preview hint) without attempting a decode.
[[nodiscard]] bool is_supported_image_type(std::string_view type) noexcept;

// The message's capture stamp (header.stamp as sec * 1e9 + nanosec) WITHOUT
// decoding its pixels, parsed from the CDR header through the zero-copy
// views. Falls back to `record_stamp_ns` (the bag record time) when the type
// is unsupported, the payload is malformed, or the publisher left
// header.stamp unset. For callers that time-align an image against other
// topics before committing to an expensive decode.
[[nodiscard]] std::int64_t image_capture_stamp_ns(
  std::string_view type, std::span<const std::byte> payload, std::int64_t record_stamp_ns);

}  // namespace bagwiz::core::image

#endif  // BAGWIZ__CORE__IMAGE__PACKED_RASTER_HPP_
