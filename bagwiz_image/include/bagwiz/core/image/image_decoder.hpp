// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__IMAGE__IMAGE_DECODER_HPP_
#define BAGWIZ__CORE__IMAGE__IMAGE_DECODER_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

// Decode a compressed still image (JPEG / PNG, as carried by
// sensor_msgs/msg/CompressedImage) into a packed 8-bit BGR raster. libav
// (FFmpeg) does the decode; like video_encoder.hpp this header exposes only std
// types so FFmpeg never enters bagwiz_image's export set.
namespace bagwiz::core::image
{

// A fully-decoded image owning its pixels. The raster is packed 8-bit BGR
// (3 channels, interleaved, row stride == width * 3), matching the "bgr8" layout
// VideoEncoder::write_frame consumes. `bgr` holds exactly width * 3 * height
// bytes.
struct DecodedImage
{
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::vector<std::byte> bgr;  // packed BGR24, width * 3 * height bytes
};

// Outcome of decode_compressed_image(). On success `image` holds the raster and
// `error` is empty; on failure `image` is empty and `error` explains why. Never
// throws.
struct DecodeResult
{
  std::optional<DecodedImage> image;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return image.has_value() && error.empty(); }
};

// Decode `data` (a JPEG or PNG bitstream) into a packed BGR24 raster. The codec
// is detected from the bitstream's magic bytes; `format` is the message's format
// string, used only to enrich the error message when detection fails. Returns an
// error result (never throws) for empty input, an unrecognized/unsupported
// codec, a missing decoder in this FFmpeg build, or a corrupt bitstream.
[[nodiscard]] DecodeResult decode_compressed_image(
  std::span<const std::byte> data, std::string_view format = {});

// A reusable decoder that keeps its FFmpeg codec context, packet/frame
// buffers, swscale context, and output buffer alive across frames, so a
// per-frame stream (e.g. one camera's images) pays the setup cost once
// instead of per decode. decode() is exactly decode_compressed_image() in
// behavior — same accepted inputs, same pixels, same errors — and the
// contexts are re-created transparently when the codec, geometry, or pixel
// format changes mid-stream. Not thread-safe; use one instance per thread.
class ImageDecoder
{
public:
  ImageDecoder();
  ~ImageDecoder();
  ImageDecoder(ImageDecoder &&) noexcept;
  ImageDecoder & operator=(ImageDecoder &&) noexcept;
  ImageDecoder(const ImageDecoder &) = delete;
  ImageDecoder & operator=(const ImageDecoder &) = delete;

  [[nodiscard]] DecodeResult decode(std::span<const std::byte> data, std::string_view format = {});

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace bagwiz::core::image

#endif  // BAGWIZ__CORE__IMAGE__IMAGE_DECODER_HPP_
