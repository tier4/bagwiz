// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/image/packed_raster.hpp"

#include "bagwiz/core/image/compressed_image.hpp"
#include "bagwiz/core/image/image_decoder.hpp"
#include "bagwiz/core/image/raw_image.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <utility>

namespace bagwiz::core::image
{

namespace
{
constexpr std::string_view kImageType = "sensor_msgs/msg/Image";
constexpr std::string_view kCompressedImageType = "sensor_msgs/msg/CompressedImage";

// Raw sensor_msgs/msg/Image -> canonical packed BGR24. bgr8 rows are copied
// verbatim (dropping any stride padding); rgb8 rows are swapped to BGR.
PackedRasterResult from_raw(std::span<const std::byte> payload)
{
  PackedRasterResult result;
  const auto raw = extract_raw_image(payload);
  if (!raw.ok()) {
    result.error = raw.error;
    return result;
  }
  const auto & view = *raw.image;

  const bool is_bgr = (view.encoding == "bgr8");
  const bool is_rgb = (view.encoding == "rgb8");
  if (!is_bgr && !is_rgb) {
    result.error = "image encoding '" + view.encoding + "' is not supported; only bgr8 and rgb8.";
    return result;
  }
  if (view.width == 0 || view.height == 0) {
    result.error = "image has zero width or height";
    return result;
  }

  const std::size_t row_bytes = static_cast<std::size_t>(view.width) * 3U;
  if (view.step < row_bytes) {
    result.error = "image row stride (step) is smaller than width*3";
    return result;
  }
  const std::size_t height = view.height;
  if (view.data.size() < static_cast<std::size_t>(view.step) * height) {
    result.error = "image pixel buffer is smaller than step*height";
    return result;
  }

  PackedRaster out;
  out.width = view.width;
  out.height = view.height;
  out.encoding = view.encoding;
  out.bgr.resize(row_bytes * height);

  for (std::uint32_t y = 0; y < view.height; ++y) {
    const std::size_t src = static_cast<std::size_t>(y) * view.step;
    const std::size_t dst = static_cast<std::size_t>(y) * row_bytes;
    if (is_bgr) {
      std::copy_n(
        view.data.begin() + static_cast<std::ptrdiff_t>(src), row_bytes,
        out.bgr.begin() + static_cast<std::ptrdiff_t>(dst));
    } else {  // rgb8 -> bgr8
      for (std::uint32_t x = 0; x < view.width; ++x) {
        const std::size_t s = src + static_cast<std::size_t>(x) * 3U;
        const std::size_t d = dst + static_cast<std::size_t>(x) * 3U;
        out.bgr[d + 0] = view.data[s + 2];  // B <- R
        out.bgr[d + 1] = view.data[s + 1];  // G
        out.bgr[d + 2] = view.data[s + 0];  // R <- B
      }
    }
  }

  result.raster = std::move(out);
  return result;
}

// sensor_msgs/msg/CompressedImage -> canonical packed BGR24 via the libav-backed
// decoder, which already emits packed BGR24.
PackedRasterResult from_compressed(std::span<const std::byte> payload)
{
  PackedRasterResult result;
  const auto compressed = extract_compressed_image(payload);
  if (!compressed.ok()) {
    result.error = compressed.error;
    return result;
  }
  auto decoded = decode_compressed_image(compressed.image->data, compressed.image->format);
  if (!decoded.ok()) {
    result.error = decoded.error;
    return result;
  }

  PackedRaster out;
  out.width = decoded.image->width;
  out.height = decoded.image->height;
  out.bgr = std::move(decoded.image->bgr);
  out.encoding = compressed.image->format;
  result.raster = std::move(out);
  return result;
}
}  // namespace

PackedRasterResult to_packed_raster(std::string_view type, std::span<const std::byte> payload)
{
  if (type == kImageType) {
    return from_raw(payload);
  }
  if (type == kCompressedImageType) {
    return from_compressed(payload);
  }
  PackedRasterResult result;
  result.error = "unsupported image message type '" + std::string(type) +
                 "'; expected sensor_msgs/msg/Image or sensor_msgs/msg/CompressedImage";
  return result;
}

// cppcheck-suppress passedByValue  // string_view is the canonical by-value idiom
bool is_supported_image_type(std::string_view type) noexcept
{
  return type == kImageType || type == kCompressedImageType;
}

}  // namespace bagwiz::core::image
