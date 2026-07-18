// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "core/image/image_fixture.hpp"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/frame.h>
#include <libavutil/pixfmt.h>
#include <libswscale/swscale.h>
}

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace bagwiz::test
{
namespace
{

// Minimal RAII bundle so each early return frees the libav handles.
struct EncodeHandles
{
  AVCodecContext * ctx = nullptr;
  AVFrame * rgb = nullptr;
  AVFrame * enc = nullptr;
  SwsContext * sws = nullptr;
  AVPacket * pkt = nullptr;

  ~EncodeHandles()
  {
    if (sws != nullptr) {
      sws_freeContext(sws);
    }
    if (rgb != nullptr) {
      av_frame_free(&rgb);
    }
    if (enc != nullptr) {
      av_frame_free(&enc);
    }
    if (pkt != nullptr) {
      av_packet_free(&pkt);
    }
    if (ctx != nullptr) {
      avcodec_free_context(&ctx);
    }
  }
};

}  // namespace

std::vector<std::byte> encode_still_image(
  const std::string & format, std::uint32_t w, std::uint32_t h, std::uint8_t r, std::uint8_t g,
  std::uint8_t b)
{
  const bool png = (format == "png");
  const AVCodecID id = png ? AV_CODEC_ID_PNG : AV_CODEC_ID_MJPEG;
  const AVPixelFormat enc_fmt = png ? AV_PIX_FMT_RGB24 : AV_PIX_FMT_YUVJ420P;

  const AVCodec * encoder = avcodec_find_encoder(id);
  if (encoder == nullptr) {
    return {};
  }

  EncodeHandles hs;
  hs.ctx = avcodec_alloc_context3(encoder);
  if (hs.ctx == nullptr) {
    return {};
  }
  hs.ctx->width = static_cast<int>(w);
  hs.ctx->height = static_cast<int>(h);
  hs.ctx->pix_fmt = enc_fmt;
  hs.ctx->time_base = AVRational{1, 1};
  if (avcodec_open2(hs.ctx, encoder, nullptr) < 0) {
    return {};
  }

  // Source RGB24 frame filled with the solid color.
  hs.rgb = av_frame_alloc();
  hs.rgb->format = AV_PIX_FMT_RGB24;
  hs.rgb->width = static_cast<int>(w);
  hs.rgb->height = static_cast<int>(h);
  if (av_frame_get_buffer(hs.rgb, 0) < 0) {
    return {};
  }
  for (std::uint32_t y = 0; y < h; ++y) {
    std::uint8_t * row = hs.rgb->data[0] + static_cast<std::ptrdiff_t>(y) * hs.rgb->linesize[0];
    for (std::uint32_t x = 0; x < w; ++x) {
      row[x * 3 + 0] = r;
      row[x * 3 + 1] = g;
      row[x * 3 + 2] = b;
    }
  }

  // Convert to the encoder's pixel format.
  hs.enc = av_frame_alloc();
  hs.enc->format = enc_fmt;
  hs.enc->width = static_cast<int>(w);
  hs.enc->height = static_cast<int>(h);
  if (av_frame_get_buffer(hs.enc, 0) < 0) {
    return {};
  }
  hs.sws = sws_getContext(
    static_cast<int>(w), static_cast<int>(h), AV_PIX_FMT_RGB24, static_cast<int>(w),
    static_cast<int>(h), enc_fmt, SWS_BILINEAR, nullptr, nullptr, nullptr);
  if (hs.sws == nullptr) {
    return {};
  }
  sws_scale(
    hs.sws, hs.rgb->data, hs.rgb->linesize, 0, static_cast<int>(h), hs.enc->data, hs.enc->linesize);
  hs.enc->pts = 0;

  if (avcodec_send_frame(hs.ctx, hs.enc) < 0) {
    return {};
  }
  (void)avcodec_send_frame(hs.ctx, nullptr);  // flush

  hs.pkt = av_packet_alloc();
  if (hs.pkt == nullptr) {
    return {};
  }
  std::vector<std::byte> out;
  while (avcodec_receive_packet(hs.ctx, hs.pkt) == 0) {
    for (int i = 0; i < hs.pkt->size; ++i) {
      out.push_back(static_cast<std::byte>(hs.pkt->data[i]));
    }
    av_packet_unref(hs.pkt);
  }
  return out;
}

}  // namespace bagwiz::test
