// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/image/image_decoder.hpp"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/error.h>
#include <libavutil/imgutils.h>
#include <libavutil/mem.h>
#include <libavutil/pixfmt.h>
#include <libswscale/swscale.h>
}

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <span>
#include <string>
#include <string_view>
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

// Pick a still-image decoder from the bitstream's leading magic bytes. JPEG and
// PNG are the formats sensor_msgs/msg/CompressedImage carries in practice (the
// image_transport `compressed` plugin emits one of the two). Returns
// AV_CODEC_ID_NONE when neither signature matches.
AVCodecID codec_from_magic(std::span<const std::byte> data)
{
  const auto * p =
    reinterpret_cast<const unsigned char *>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
      data.data());
  if (data.size() >= 3 && p[0] == 0xFF && p[1] == 0xD8 && p[2] == 0xFF) {
    return AV_CODEC_ID_MJPEG;
  }
  static constexpr std::array<unsigned char, 8> kPngSig{0x89, 0x50, 0x4E, 0x47,
                                                        0x0D, 0x0A, 0x1A, 0x0A};
  if (data.size() >= kPngSig.size() && std::memcmp(p, kPngSig.data(), kPngSig.size()) == 0) {
    return AV_CODEC_ID_PNG;
  }
  return AV_CODEC_ID_NONE;
}

// JPEG decoders report the deprecated full-range YUVJ* pixel formats, which make
// sws_getContext spew a "deprecated pixel format" warning on every call (i.e.
// once per frame). Map them to their plain YUV* equivalents and report that the
// source is full-range so the caller can tell swscale via
// sws_setColorspaceDetails — preserving the JPEG color range without the noise.
AVPixelFormat normalize_pixel_format(AVPixelFormat fmt, bool & full_range)
{
  switch (fmt) {
    case AV_PIX_FMT_YUVJ420P:
      full_range = true;
      return AV_PIX_FMT_YUV420P;
    case AV_PIX_FMT_YUVJ422P:
      full_range = true;
      return AV_PIX_FMT_YUV422P;
    case AV_PIX_FMT_YUVJ444P:
      full_range = true;
      return AV_PIX_FMT_YUV444P;
    case AV_PIX_FMT_YUVJ440P:
      full_range = true;
      return AV_PIX_FMT_YUV440P;
    default:
      full_range = false;
      return fmt;
  }
}

// RAII for the libav decode handles, so every early return frees them.
struct DecodeContext
{
  AVCodecContext * codec = nullptr;
  AVPacket * pkt = nullptr;
  AVFrame * frame = nullptr;
  SwsContext * sws = nullptr;
  std::uint8_t * dst_buf = nullptr;  // av_image_alloc'd swscale destination

  DecodeContext() = default;
  DecodeContext(const DecodeContext &) = delete;
  DecodeContext & operator=(const DecodeContext &) = delete;
  DecodeContext(DecodeContext &&) = delete;
  DecodeContext & operator=(DecodeContext &&) = delete;

  ~DecodeContext()
  {
    if (dst_buf != nullptr) {
      av_freep(&dst_buf);
    }
    if (sws != nullptr) {
      sws_freeContext(sws);
    }
    if (frame != nullptr) {
      av_frame_free(&frame);
    }
    if (pkt != nullptr) {
      av_packet_free(&pkt);  // frees the av_malloc'd buffer handed to av_packet_from_data
    }
    if (codec != nullptr) {
      avcodec_free_context(&codec);
    }
  }
};

}  // namespace

DecodeResult decode_compressed_image(std::span<const std::byte> data, std::string_view format)
{
  DecodeResult result;

  if (data.empty()) {
    result.error = "empty compressed image data";
    return result;
  }
  if (data.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    result.error = "compressed image exceeds the supported maximum size";
    return result;
  }

  const AVCodecID id = codec_from_magic(data);
  if (id == AV_CODEC_ID_NONE) {
    result.error = "unrecognized compressed image format (expected JPEG or PNG)" +
                   (format.empty() ? std::string{} : "; format='" + std::string(format) + "'");
    return result;
  }

  const AVCodec * decoder = avcodec_find_decoder(id);
  if (decoder == nullptr) {
    result.error =
      std::string("decoder not available in this FFmpeg build: ") + avcodec_get_name(id);
    return result;
  }

  DecodeContext ctx;
  ctx.codec = avcodec_alloc_context3(decoder);
  if (ctx.codec == nullptr) {
    result.error = "could not allocate decoder context";
    return result;
  }
  if (int ret = avcodec_open2(ctx.codec, decoder, nullptr); ret < 0) {
    result.error = "could not open decoder: " + av_err(ret);
    return result;
  }

  // avcodec_send_packet reads up to AV_INPUT_BUFFER_PADDING_SIZE bytes past the
  // end, so the bitstream must live in an av_malloc'd, zero-padded buffer.
  // av_packet_from_data takes ownership; av_packet_free releases it.
  ctx.pkt = av_packet_alloc();
  if (ctx.pkt == nullptr) {
    result.error = "could not allocate packet";
    return result;
  }
  const int data_size = static_cast<int>(data.size());
  auto * buf = static_cast<std::uint8_t *>(
    av_malloc(static_cast<std::size_t>(data_size) + AV_INPUT_BUFFER_PADDING_SIZE));
  if (buf == nullptr) {
    result.error = "could not allocate decode input buffer";
    return result;
  }
  std::memcpy(buf, data.data(), data.size());
  std::memset(buf + data_size, 0, AV_INPUT_BUFFER_PADDING_SIZE);
  if (int ret = av_packet_from_data(ctx.pkt, buf, data_size); ret < 0) {
    av_free(buf);
    result.error = "could not wrap decode input: " + av_err(ret);
    return result;
  }

  ctx.frame = av_frame_alloc();
  if (ctx.frame == nullptr) {
    result.error = "could not allocate frame";
    return result;
  }

  if (int ret = avcodec_send_packet(ctx.codec, ctx.pkt); ret < 0) {
    result.error = "decoder send_packet failed: " + av_err(ret);
    return result;
  }
  int ret = avcodec_receive_frame(ctx.codec, ctx.frame);
  if (ret == AVERROR(EAGAIN)) {
    // A single still frame may need an explicit flush before it surfaces.
    if (int flush = avcodec_send_packet(ctx.codec, nullptr); flush < 0 && flush != AVERROR_EOF) {
      result.error = "decoder flush failed: " + av_err(flush);
      return result;
    }
    ret = avcodec_receive_frame(ctx.codec, ctx.frame);
  }
  if (ret < 0) {
    result.error = "decoder receive_frame failed: " + av_err(ret);
    return result;
  }

  if (ctx.frame->width <= 0 || ctx.frame->height <= 0) {
    result.error = "decoded image has invalid dimensions";
    return result;
  }
  const int width = ctx.frame->width;
  const int height = ctx.frame->height;
  bool full_range = false;
  const AVPixelFormat src_fmt =
    normalize_pixel_format(static_cast<AVPixelFormat>(ctx.frame->format), full_range);

  ctx.sws = sws_getContext(
    width, height, src_fmt, width, height, AV_PIX_FMT_BGR24, SWS_BILINEAR, nullptr, nullptr,
    nullptr);
  if (ctx.sws == nullptr) {
    result.error = "failed to create swscale context for the decoded image";
    return result;
  }
  if (full_range) {
    // Tell swscale the source uses the JPEG (full 0-255) range so a remapped
    // YUVJ* frame keeps its original luma/chroma scaling. brightness/contrast/
    // saturation use swscale's 16.16 fixed-point identity (0 / 1.0 / 1.0).
    const int * coeffs = sws_getCoefficients(SWS_CS_DEFAULT);
    sws_setColorspaceDetails(
      ctx.sws, coeffs, /*srcRange=*/1, coeffs, /*dstRange=*/1, /*brightness=*/0,
      /*contrast=*/1 << 16, /*saturation=*/1 << 16);
  }

  // sws_scale's SIMD paths can write past the end of a tightly-packed row, so
  // give it an aligned, padded destination via av_image_alloc rather than
  // writing straight into a width*3-strided buffer (which overflows for small
  // widths). The packed rows are copied out afterward.
  std::array<std::uint8_t *, 4> dst_data{};
  std::array<int, 4> dst_linesize{};
  const int alloc =
    av_image_alloc(dst_data.data(), dst_linesize.data(), width, height, AV_PIX_FMT_BGR24, 16);
  if (alloc < 0) {
    result.error = "could not allocate decode output buffer: " + av_err(alloc);
    return result;
  }
  ctx.dst_buf = dst_data[0];  // owned by DecodeContext; freed on any return below

  const int scaled = sws_scale(
    ctx.sws, ctx.frame->data, ctx.frame->linesize, 0, height, dst_data.data(), dst_linesize.data());
  if (scaled != height) {
    result.error = "swscale produced an unexpected number of rows";
    return result;
  }

  DecodedImage image;
  image.width = static_cast<std::uint32_t>(width);
  image.height = static_cast<std::uint32_t>(height);
  const int row_bytes = width * 3;
  image.bgr.assign(
    static_cast<std::size_t>(row_bytes) * static_cast<std::size_t>(height), std::byte{0});
  // Copy the packed rows out of the (possibly over-strided) swscale buffer. Read
  // through ctx.dst_buf — the same pointer as dst_data[0] — so the owning member
  // is used in-function, not only in the destructor.
  for (int y = 0; y < height; ++y) {
    std::memcpy(
      image.bgr.data() + static_cast<std::ptrdiff_t>(y) * row_bytes,
      ctx.dst_buf + static_cast<std::ptrdiff_t>(y) * dst_linesize[0],
      static_cast<std::size_t>(row_bytes));
  }

  result.image = std::move(image);
  return result;
}

}  // namespace bagwiz::core::image
