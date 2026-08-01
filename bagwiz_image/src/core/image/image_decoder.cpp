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

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
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

}  // namespace

// Holds the libav decode handles across decode() calls. Same members and frees
// as the one-shot DecodeContext this replaces, plus the keys needed to tell
// whether a context can be reused as-is or must be rebuilt for the new frame.
struct ImageDecoder::Impl
{
  AVCodecContext * codec = nullptr;
  AVCodecID open_id = AV_CODEC_ID_NONE;  // codec the context was opened for
  AVPacket * pkt = nullptr;              // allocated once, unref'd per frame
  AVFrame * frame = nullptr;             // allocated once, unref'd per frame
  SwsContext * sws = nullptr;
  int sws_w = 0, sws_h = 0;  // geometry/format the sws context was built for
  AVPixelFormat sws_fmt = AV_PIX_FMT_NONE;
  bool sws_full_range = false;
  std::uint8_t * dst_buf = nullptr;  // av_image_alloc'd, reused while (w,h) unchanged
  std::array<int, 4> dst_linesize{};
  int dst_w = 0, dst_h = 0;

  // Codec select/open, packet wrap, send/receive/flush — shared by decode()
  // and decode_to_yuv(). On success `frame` holds a valid, positively-sized
  // decoded frame; on failure `error` explains why and `frame` must not be
  // read. Flushes the codec unconditionally before returning either way (see
  // the DecodeCleanup comment at the call site), so the returned frame's
  // validity relies on it holding its own ref rather than on codec state.
  bool receive_frame(std::span<const std::byte> data, std::string_view format, std::string & error);

  ~Impl()
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

ImageDecoder::ImageDecoder() : impl_(std::make_unique<Impl>())
{
}
ImageDecoder::~ImageDecoder() = default;
ImageDecoder::ImageDecoder(ImageDecoder &&) noexcept = default;
ImageDecoder & ImageDecoder::operator=(ImageDecoder &&) noexcept = default;

bool ImageDecoder::Impl::receive_frame(
  std::span<const std::byte> data, std::string_view format, std::string & error)
{
  if (data.empty()) {
    error = "empty compressed image data";
    return false;
  }
  if (data.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    error = "compressed image exceeds the supported maximum size";
    return false;
  }

  const AVCodecID id = codec_from_magic(data);
  if (id == AV_CODEC_ID_NONE) {
    error = "unrecognized compressed image format (expected JPEG or PNG)" +
            (format.empty() ? std::string{} : "; format='" + std::string(format) + "'");
    return false;
  }

  const AVCodec * decoder = avcodec_find_decoder(id);
  if (decoder == nullptr) {
    error = std::string("decoder not available in this FFmpeg build: ") + avcodec_get_name(id);
    return false;
  }

  // Reopen only when the codec changes; a same-codec stream (the common case)
  // reuses the already-open context.
  if (open_id != id) {
    if (codec != nullptr) {
      avcodec_free_context(&codec);
    }
    open_id = AV_CODEC_ID_NONE;  // not valid again until avcodec_open2 succeeds below
    codec = avcodec_alloc_context3(decoder);
    if (codec == nullptr) {
      error = "could not allocate decoder context";
      return false;
    }
    if (int ret = avcodec_open2(codec, decoder, nullptr); ret < 0) {
      avcodec_free_context(&codec);
      error = "could not open decoder: " + av_err(ret);
      return false;
    }
    open_id = id;
  }

  if (pkt == nullptr) {
    pkt = av_packet_alloc();
    if (pkt == nullptr) {
      error = "could not allocate packet";
      return false;
    }
  }
  if (frame == nullptr) {
    frame = av_frame_alloc();
    if (frame == nullptr) {
      error = "could not allocate frame";
      return false;
    }
  }

  // Drop the previous call's decoded frame now, not after this call finishes —
  // a zero-copy view of it (DecodedYuvView) must stay valid until the next
  // decode()/decode_to_yuv() call starts, not just until this one ends.
  av_frame_unref(frame);

  // avcodec_send_packet reads up to AV_INPUT_BUFFER_PADDING_SIZE bytes past the
  // end, so the bitstream must live in an av_malloc'd, zero-padded buffer.
  // av_packet_from_data takes ownership; av_packet_unref below releases it.
  const int data_size = static_cast<int>(data.size());
  auto * buf = static_cast<std::uint8_t *>(
    av_malloc(static_cast<std::size_t>(data_size) + AV_INPUT_BUFFER_PADDING_SIZE));
  if (buf == nullptr) {
    error = "could not allocate decode input buffer";
    return false;
  }
  std::memcpy(buf, data.data(), data.size());
  std::memset(buf + data_size, 0, AV_INPUT_BUFFER_PADDING_SIZE);
  if (int ret = av_packet_from_data(pkt, buf, data_size); ret < 0) {
    av_free(buf);
    error = "could not wrap decode input: " + av_err(ret);
    return false;
  }

  // From here on pkt owns `buf` and the codec is about to receive it, so
  // every remaining return path — success or error — must release the
  // packet's buffer and flush the codec. Without the unconditional flush, the
  // EAGAIN path below (which sends a NULL packet to force a still frame out)
  // would leave this persistent context stuck in draining mode and reject the
  // next frame's packet. The flush is safe for a zero-copy view of `frame`:
  // the frame holds its own ref, so its data survives the flush and stays
  // valid until the next receive_frame() call's av_frame_unref above.
  struct DecodeCleanup
  {
    AVPacket * pkt;
    AVCodecContext * codec;
    ~DecodeCleanup()
    {
      av_packet_unref(pkt);
      avcodec_flush_buffers(codec);
    }
  } cleanup{pkt, codec};

  if (int ret = avcodec_send_packet(codec, pkt); ret < 0) {
    error = "decoder send_packet failed: " + av_err(ret);
    return false;
  }
  int ret = avcodec_receive_frame(codec, frame);
  if (ret == AVERROR(EAGAIN)) {
    // A single still frame may need an explicit flush before it surfaces.
    if (int flush = avcodec_send_packet(codec, nullptr); flush < 0 && flush != AVERROR_EOF) {
      error = "decoder flush failed: " + av_err(flush);
      return false;
    }
    ret = avcodec_receive_frame(codec, frame);
  }
  if (ret < 0) {
    error = "decoder receive_frame failed: " + av_err(ret);
    return false;
  }

  if (frame->width <= 0 || frame->height <= 0) {
    error = "decoded image has invalid dimensions";
    return false;
  }
  return true;
}

DecodeResult ImageDecoder::decode(std::span<const std::byte> data, std::string_view format)
{
  Impl & ctx = *impl_;
  DecodeResult result;

  if (!ctx.receive_frame(data, format, result.error)) {
    return result;
  }

  const int width = ctx.frame->width;
  const int height = ctx.frame->height;
  bool full_range = false;
  const AVPixelFormat src_fmt =
    normalize_pixel_format(static_cast<AVPixelFormat>(ctx.frame->format), full_range);

  if (
    ctx.sws == nullptr || ctx.sws_w != width || ctx.sws_h != height || ctx.sws_fmt != src_fmt ||
    ctx.sws_full_range != full_range) {
    if (ctx.sws != nullptr) {
      sws_freeContext(ctx.sws);
      ctx.sws = nullptr;
    }
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
    ctx.sws_w = width;
    ctx.sws_h = height;
    ctx.sws_fmt = src_fmt;
    ctx.sws_full_range = full_range;
  }

  // sws_scale's SIMD paths can write past the end of a tightly-packed row, so
  // give it an aligned, padded destination via av_image_alloc rather than
  // writing straight into a width*3-strided buffer (which overflows for small
  // widths). The packed rows are copied out afterward. Reused across calls
  // while (width, height) stay the same.
  std::array<std::uint8_t *, 4> dst_data{};
  if (ctx.dst_buf == nullptr || ctx.dst_w != width || ctx.dst_h != height) {
    if (ctx.dst_buf != nullptr) {
      av_freep(&ctx.dst_buf);
    }
    const int alloc =
      av_image_alloc(dst_data.data(), ctx.dst_linesize.data(), width, height, AV_PIX_FMT_BGR24, 16);
    if (alloc < 0) {
      result.error = "could not allocate decode output buffer: " + av_err(alloc);
      return result;
    }
    ctx.dst_buf = dst_data[0];  // owned by Impl; freed on the next resize or on destruction
    ctx.dst_w = width;
    ctx.dst_h = height;
  } else {
    dst_data[0] = ctx.dst_buf;
  }

  const int scaled = sws_scale(
    ctx.sws, ctx.frame->data, ctx.frame->linesize, 0, height, dst_data.data(),
    ctx.dst_linesize.data());
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
  // Copy the packed rows out of the (possibly over-strided) swscale buffer.
  for (int y = 0; y < height; ++y) {
    std::memcpy(
      image.bgr.data() + static_cast<std::ptrdiff_t>(y) * row_bytes,
      ctx.dst_buf + static_cast<std::ptrdiff_t>(y) * ctx.dst_linesize[0],
      static_cast<std::size_t>(row_bytes));
  }

  result.image = std::move(image);
  return result;
}

DecodeYuvResult ImageDecoder::decode_to_yuv(
  std::span<const std::byte> data, std::string_view format)
{
  Impl & ctx = *impl_;
  DecodeYuvResult result;

  if (!ctx.receive_frame(data, format, result.error)) {
    return result;
  }

  bool full_range = false;
  const AVPixelFormat src_fmt =
    normalize_pixel_format(static_cast<AVPixelFormat>(ctx.frame->format), full_range);

  DecodedYuvView view;
  switch (src_fmt) {
    case AV_PIX_FMT_YUV420P:
      view.chroma = YuvChroma::k420;
      break;
    case AV_PIX_FMT_YUV422P:
      view.chroma = YuvChroma::k422;
      break;
    case AV_PIX_FMT_YUV444P:
      view.chroma = YuvChroma::k444;
      break;
    case AV_PIX_FMT_GRAY8:
      view.chroma = YuvChroma::kGray;
      break;
    default:
      result.error = "decoded pixel format is not planar YUV";
      return result;
  }

  view.width = static_cast<std::uint32_t>(ctx.frame->width);
  view.height = static_cast<std::uint32_t>(ctx.frame->height);
  view.full_range = full_range;
  view.y = ctx.frame->data[0];
  view.y_stride = ctx.frame->linesize[0];
  if (view.chroma != YuvChroma::kGray) {
    view.u = ctx.frame->data[1];
    view.u_stride = ctx.frame->linesize[1];
    view.v = ctx.frame->data[2];
    view.v_stride = ctx.frame->linesize[2];
  }

  result.view = view;
  return result;
}

std::array<std::uint8_t, 3> sample_rgb(
  const DecodedYuvView & view, std::uint32_t x, std::uint32_t y)
{
  const auto clamp8 = [](double v) {
    return static_cast<std::uint8_t>(std::lround(std::clamp(v, 0.0, 255.0)));
  };
  const int luma = view.y[static_cast<std::size_t>(y) * view.y_stride + x];
  double yv = view.full_range ? luma : (luma - 16) * (255.0 / 219.0);
  double cb = 0.0, cr = 0.0;
  if (view.chroma != YuvChroma::kGray) {
    const int hs = view.chroma == YuvChroma::k444 ? 0 : 1;  // horizontal shift
    const int vs = view.chroma == YuvChroma::k420 ? 1 : 0;  // vertical shift
    const std::size_t ci = static_cast<std::size_t>(y >> vs) * view.u_stride + (x >> hs);
    const std::size_t cj = static_cast<std::size_t>(y >> vs) * view.v_stride + (x >> hs);
    const double scale = view.full_range ? 1.0 : 255.0 / 224.0;
    cb = (view.u[ci] - 128) * scale;
    cr = (view.v[cj] - 128) * scale;
  }
  return {
    clamp8(yv + 1.402 * cr), clamp8(yv - 0.344136 * cb - 0.714136 * cr), clamp8(yv + 1.772 * cb)};
}

DecodeResult decode_compressed_image(std::span<const std::byte> data, std::string_view format)
{
  ImageDecoder decoder;
  return decoder.decode(data, format);
}

}  // namespace bagwiz::core::image
