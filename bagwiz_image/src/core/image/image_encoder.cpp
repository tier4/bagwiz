// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/image/image_encoder.hpp"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/error.h>
#include <libavutil/frame.h>
#include <libavutil/pixfmt.h>
}

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace bagwiz::core::image
{
namespace
{

std::string av_err(int errnum)
{
  std::array<char, AV_ERROR_MAX_STRING_SIZE> buf{};
  av_strerror(errnum, buf.data(), buf.size());
  return std::string(buf.data());
}

// RAII bundle of libav encode handles (codec, frame, packet), freed on destruction
// to ensure early returns never leak.
struct EncodeContext
{
  AVCodecContext * codec = nullptr;
  AVFrame * frame = nullptr;
  AVPacket * pkt = nullptr;

  EncodeContext() = default;
  EncodeContext(const EncodeContext &) = delete;
  EncodeContext & operator=(const EncodeContext &) = delete;
  EncodeContext(EncodeContext &&) = delete;
  EncodeContext & operator=(EncodeContext &&) = delete;

  ~EncodeContext()
  {
    if (pkt != nullptr) {
      av_packet_free(&pkt);
    }
    if (frame != nullptr) {
      av_frame_free(&frame);
    }
    if (codec != nullptr) {
      avcodec_free_context(&codec);
    }
  }
};

}  // namespace

EncodePngResult encode_png(const PackedRaster & raster)
{
  EncodePngResult result;

  if (raster.empty()) {
    result.error = "cannot encode an empty raster";
    return result;
  }
  // Bound the dimensions before multiplying them out, so the width * 3 * height
  // size check below cannot overflow size_t (libav's API is int-typed anyway).
  const auto int_max = static_cast<std::uint32_t>(std::numeric_limits<int>::max());
  if (raster.width > int_max || raster.height > int_max) {
    result.error = "raster dimensions exceed the supported maximum";
    return result;
  }
  const std::size_t expected = static_cast<std::size_t>(raster.width) * 3U * raster.height;
  if (raster.bgr.size() != expected) {
    result.error = "raster pixel buffer size does not match width * 3 * height";
    return result;
  }

  const AVCodec * encoder = avcodec_find_encoder(AV_CODEC_ID_PNG);
  if (encoder == nullptr) {
    result.error = "PNG encoder not available in this FFmpeg build";
    return result;
  }

  EncodeContext ctx;
  ctx.codec = avcodec_alloc_context3(encoder);
  if (ctx.codec == nullptr) {
    result.error = "could not allocate encoder context";
    return result;
  }
  ctx.codec->width = static_cast<int>(raster.width);
  ctx.codec->height = static_cast<int>(raster.height);
  ctx.codec->pix_fmt = AV_PIX_FMT_RGB24;
  // PNG is a single intra frame; the time base is unused for a still image but
  // libav requires a non-zero value.
  ctx.codec->time_base = AVRational{1, 1};
  if (int ret = avcodec_open2(ctx.codec, encoder, nullptr); ret < 0) {
    result.error = "could not open encoder: " + av_err(ret);
    return result;
  }

  ctx.frame = av_frame_alloc();
  if (ctx.frame == nullptr) {
    result.error = "could not allocate frame";
    return result;
  }
  ctx.frame->format = AV_PIX_FMT_RGB24;
  ctx.frame->width = static_cast<int>(raster.width);
  ctx.frame->height = static_cast<int>(raster.height);
  // av_frame_get_buffer aligns each row, so frame->linesize[0] may exceed
  // width * 3 — fill row by row through that stride rather than as one block.
  if (int ret = av_frame_get_buffer(ctx.frame, 0); ret < 0) {
    result.error = "could not allocate frame buffer: " + av_err(ret);
    return result;
  }
  if (int ret = av_frame_make_writable(ctx.frame); ret < 0) {
    result.error = "frame buffer is not writable: " + av_err(ret);
    return result;
  }

  // Copy the packed BGR24 source into the RGB24 frame, swapping B and R per
  // pixel. A manual swap keeps the source side free of swscale's SIMD over-read
  // on tightly-packed rows (the decoder pads its swscale destination for the
  // same reason); a pure channel reorder needs no colorspace math.
  const int width = static_cast<int>(raster.width);
  const int height = static_cast<int>(raster.height);
  const std::size_t src_stride = static_cast<std::size_t>(width) * 3U;
  const auto * src =
    reinterpret_cast<const std::uint8_t *>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
      raster.bgr.data());
  for (int y = 0; y < height; ++y) {
    const std::uint8_t * src_row = src + static_cast<std::size_t>(y) * src_stride;
    std::uint8_t * dst_row =
      ctx.frame->data[0] + static_cast<std::ptrdiff_t>(y) * ctx.frame->linesize[0];
    for (int x = 0; x < width; ++x) {
      const std::uint8_t * sp = src_row + static_cast<std::size_t>(x) * 3U;
      std::uint8_t * dp = dst_row + static_cast<std::size_t>(x) * 3U;
      dp[0] = sp[2];  // R <- src red
      dp[1] = sp[1];  // G <- src green
      dp[2] = sp[0];  // B <- src blue
    }
  }
  ctx.frame->pts = 0;

  ctx.pkt = av_packet_alloc();
  if (ctx.pkt == nullptr) {
    result.error = "could not allocate packet";
    return result;
  }

  if (int ret = avcodec_send_frame(ctx.codec, ctx.frame); ret < 0) {
    result.error = "encoder send_frame failed: " + av_err(ret);
    return result;
  }
  // Signal end-of-stream so the single still frame is flushed out as a packet.
  // A successful flush returns 0; AVERROR_EOF would only mean the encoder was
  // already drained (impossible after a single send), so accept it defensively
  // and surface any other negative code as a real error.
  if (int ret = avcodec_send_frame(ctx.codec, nullptr); ret < 0 && ret != AVERROR_EOF) {
    result.error = "encoder flush failed: " + av_err(ret);
    return result;
  }

  std::vector<std::byte> png;
  while (true) {
    const int ret = avcodec_receive_packet(ctx.codec, ctx.pkt);
    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
      break;
    }
    if (ret < 0) {
      result.error = "encoder receive_packet failed: " + av_err(ret);
      return result;
    }
    const auto * data =
      reinterpret_cast<const std::byte *>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        ctx.pkt->data);
    png.insert(png.end(), data, data + static_cast<std::size_t>(ctx.pkt->size));
    av_packet_unref(ctx.pkt);
  }

  if (png.empty()) {
    result.error = "encoder produced no output";
    return result;
  }

  result.png = std::move(png);
  return result;
}

}  // namespace bagwiz::core::image
