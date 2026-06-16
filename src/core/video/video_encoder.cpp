// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/video/video_encoder.hpp"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/error.h>
#include <libavutil/opt.h>
#include <libavutil/pixfmt.h>
#include <libavutil/rational.h>
#include <libswscale/swscale.h>
}

#include <array>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <utility>

namespace bagwiz::core::video
{
namespace
{

std::string av_err(int errnum)
{
  std::array<char, AV_ERROR_MAX_STRING_SIZE> buf{};
  av_strerror(errnum, buf.data(), buf.size());
  return std::string(buf.data());
}

// Container + codec selection derived from the output extension.
struct CodecChoice
{
  AVCodecID id = AV_CODEC_ID_NONE;
  const char * encoder_name = nullptr;  // preferred encoder name; null -> find by id
  AVPixelFormat pix_fmt = AV_PIX_FMT_YUV420P;
  bool jpeg_range = false;    // tag the stream full-range (MJPEG)
  bool requires_even = true;  // 4:2:0 chroma needs even dimensions
};

std::optional<CodecChoice> codec_for_extension(
  const std::filesystem::path & output, std::string & error)
{
  std::string ext = output.extension().string();
  for (auto & c : ext) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }

  if (ext == ".mp4" || ext == ".mkv" || ext == ".mov") {
    return CodecChoice{AV_CODEC_ID_H264, "libx264", AV_PIX_FMT_YUV420P, false, true};
  }
  if (ext == ".avi") {
    return CodecChoice{AV_CODEC_ID_MJPEG, nullptr, AV_PIX_FMT_YUVJ420P, true, true};
  }
  error = "unsupported output extension '" + ext + "'; use .mp4/.mkv (H.264) or .avi (MJPEG)";
  return std::nullopt;
}

}  // namespace

struct VideoEncoder::Impl
{
  AVFormatContext * fmt = nullptr;
  AVCodecContext * codec = nullptr;
  AVStream * stream = nullptr;
  SwsContext * sws = nullptr;
  AVFrame * frame = nullptr;
  AVPacket * pkt = nullptr;
  int width = 0;
  int height = 0;
  AVPixelFormat pix_fmt = AV_PIX_FMT_YUV420P;
  SourcePixelFormat sws_src_fmt = SourcePixelFormat::kBgr8;
  bool sws_ready = false;
  bool finished = false;
  std::int64_t next_pts = 0;

  Impl() = default;
  Impl(const Impl &) = delete;
  Impl & operator=(const Impl &) = delete;
  Impl(Impl &&) = delete;
  Impl & operator=(Impl &&) = delete;

  ~Impl()
  {
    if (sws != nullptr) {
      sws_freeContext(sws);
    }
    if (frame != nullptr) {
      av_frame_free(&frame);
    }
    if (pkt != nullptr) {
      av_packet_free(&pkt);
    }
    if (codec != nullptr) {
      avcodec_free_context(&codec);
    }
    if (fmt != nullptr) {
      if (fmt->pb != nullptr && (fmt->oformat->flags & AVFMT_NOFILE) == 0) {
        avio_closep(&fmt->pb);
      }
      avformat_free_context(fmt);
    }
  }

  // Send one frame (or nullptr to flush) and write every packet the encoder
  // returns. Empty string on success.
  std::string drain(AVFrame * f)
  {
    int ret = avcodec_send_frame(codec, f);
    if (ret < 0) {
      av_packet_unref(pkt);  // drop any residual data from a prior partial drain
      return "encoder send_frame failed: " + av_err(ret);
    }
    while (true) {
      ret = avcodec_receive_packet(codec, pkt);
      if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
        break;
      }
      if (ret < 0) {
        return "encoder receive_packet failed: " + av_err(ret);
      }
      av_packet_rescale_ts(pkt, codec->time_base, stream->time_base);
      pkt->stream_index = stream->index;
      ret = av_interleaved_write_frame(fmt, pkt);
      av_packet_unref(pkt);
      if (ret < 0) {
        return "muxer write_frame failed: " + av_err(ret);
      }
    }
    return {};
  }
};

VideoEncoder::VideoEncoder(std::unique_ptr<Impl> impl) : impl_(std::move(impl))
{
}
VideoEncoder::~VideoEncoder() = default;
VideoEncoder::VideoEncoder(VideoEncoder &&) noexcept = default;
VideoEncoder & VideoEncoder::operator=(VideoEncoder &&) noexcept = default;

std::string VideoEncoder::write_frame(
  std::span<const std::byte> pixels, std::size_t stride, SourcePixelFormat format)
{
  Impl & im = *impl_;

  // sws_scale takes int strides; a stride past INT_MAX would narrow to a
  // negative value and corrupt the conversion.
  if (stride > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return "row stride " + std::to_string(stride) + " exceeds the supported maximum";
  }
  const auto need = stride * static_cast<std::size_t>(im.height);
  if (pixels.size() < need) {
    return "frame buffer too small: have " + std::to_string(pixels.size()) + " bytes, need " +
           std::to_string(need);
  }

  const AVPixelFormat src =
    (format == SourcePixelFormat::kBgr8) ? AV_PIX_FMT_BGR24 : AV_PIX_FMT_RGB24;
  if (!im.sws_ready || im.sws_src_fmt != format) {
    if (im.sws != nullptr) {
      sws_freeContext(im.sws);
      im.sws = nullptr;
    }
    im.sws = sws_getContext(
      im.width, im.height, src, im.width, im.height, im.pix_fmt, SWS_BILINEAR, nullptr, nullptr,
      nullptr);
    if (im.sws == nullptr) {
      // Leave sws_ready false so a later call retries instead of running
      // sws_scale on a null context (which would be undefined behavior).
      im.sws_ready = false;
      return "failed to create swscale conversion context";
    }
    im.sws_src_fmt = format;
    im.sws_ready = true;
  }

  int ret = av_frame_make_writable(im.frame);
  if (ret < 0) {
    return "frame_make_writable failed: " + av_err(ret);
  }

  const auto * src_ptr =
    reinterpret_cast<const std::uint8_t *>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
      pixels.data());
  const std::array<const std::uint8_t *, 4> src_data{src_ptr, nullptr, nullptr, nullptr};
  const std::array<int, 4> src_stride{static_cast<int>(stride), 0, 0, 0};
  sws_scale(
    im.sws, src_data.data(), src_stride.data(), 0, im.height, im.frame->data, im.frame->linesize);

  im.frame->pts = im.next_pts++;
  return im.drain(im.frame);
}

std::string VideoEncoder::finish()
{
  Impl & im = *impl_;
  if (im.finished) {
    return {};
  }
  im.finished = true;
  if (auto e = im.drain(nullptr); !e.empty()) {  // flush the encoder's delay queue
    return e;
  }
  const int ret = av_write_trailer(im.fmt);
  if (ret < 0) {
    return "write_trailer failed: " + av_err(ret);
  }
  return {};
}

OpenVideoEncoderResult open_video_encoder(
  const std::filesystem::path & output, std::uint32_t width, std::uint32_t height, int fps_num,
  int fps_den)
{
  OpenVideoEncoderResult result;

  std::string ext_error;
  const auto choice = codec_for_extension(output, ext_error);
  if (!choice.has_value()) {
    result.error = ext_error;
    return result;
  }
  if (width == 0 || height == 0) {
    result.error = "image has zero width or height";
    return result;
  }
  if (
    width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
    height > static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
    result.error = "image dimensions exceed the supported maximum";
    return result;
  }
  if (choice->requires_even && ((width % 2U) != 0U || (height % 2U) != 0U)) {
    result.error = "this codec needs even dimensions, but the image is " + std::to_string(width) +
                   "x" + std::to_string(height);
    return result;
  }
  if (fps_num <= 0 || fps_den <= 0) {
    result.error = "invalid frame rate";
    return result;
  }

  auto im = std::make_unique<VideoEncoder::Impl>();
  im->width = static_cast<int>(width);
  im->height = static_cast<int>(height);
  im->pix_fmt = choice->pix_fmt;

  const std::string path_str = output.string();

  const AVCodec * encoder = (choice->encoder_name != nullptr)
                              ? avcodec_find_encoder_by_name(choice->encoder_name)
                              : avcodec_find_encoder(choice->id);
  if (encoder == nullptr) {
    result.error =
      std::string("encoder not available in this FFmpeg build: ") +
      (choice->encoder_name != nullptr ? choice->encoder_name : avcodec_get_name(choice->id));
    if (choice->id == AV_CODEC_ID_H264) {
      result.error += " (try an .avi output, which uses the built-in MJPEG encoder)";
    }
    return result;
  }

  int ret = avformat_alloc_output_context2(&im->fmt, nullptr, nullptr, path_str.c_str());
  if (ret < 0 || im->fmt == nullptr) {
    result.error = "could not allocate output context: " + av_err(ret);
    return result;
  }

  im->stream = avformat_new_stream(im->fmt, nullptr);
  if (im->stream == nullptr) {
    result.error = "could not create output stream";
    return result;
  }

  im->codec = avcodec_alloc_context3(encoder);
  if (im->codec == nullptr) {
    result.error = "could not allocate codec context";
    return result;
  }
  im->codec->width = im->width;
  im->codec->height = im->height;
  im->codec->pix_fmt = choice->pix_fmt;
  im->codec->time_base = AVRational{fps_den, fps_num};  // seconds per frame
  im->codec->framerate = AVRational{fps_num, fps_den};
  im->codec->gop_size = 12;
  if (choice->jpeg_range) {
    im->codec->color_range = AVCOL_RANGE_JPEG;
  }
  im->stream->time_base = im->codec->time_base;
  if ((im->fmt->oformat->flags & AVFMT_GLOBALHEADER) != 0) {
    im->codec->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
  }

  AVDictionary * opts = nullptr;
  if (choice->id == AV_CODEC_ID_H264) {
    av_dict_set(&opts, "preset", "medium", 0);
    av_dict_set(&opts, "crf", "23", 0);
  }
  ret = avcodec_open2(im->codec, encoder, &opts);
  av_dict_free(&opts);
  if (ret < 0) {
    result.error = "could not open encoder: " + av_err(ret);
    return result;
  }

  ret = avcodec_parameters_from_context(im->stream->codecpar, im->codec);
  if (ret < 0) {
    result.error = "could not copy codec parameters: " + av_err(ret);
    return result;
  }

  if ((im->fmt->oformat->flags & AVFMT_NOFILE) == 0) {
    ret = avio_open(&im->fmt->pb, path_str.c_str(), AVIO_FLAG_WRITE);
    if (ret < 0) {
      result.error = "could not open '" + path_str + "' for writing: " + av_err(ret);
      return result;
    }
  }

  ret = avformat_write_header(im->fmt, nullptr);
  if (ret < 0) {
    result.error = "could not write file header: " + av_err(ret);
    return result;
  }

  im->frame = av_frame_alloc();
  if (im->frame == nullptr) {
    result.error = "could not allocate frame";
    return result;
  }
  im->frame->format = choice->pix_fmt;
  im->frame->width = im->width;
  im->frame->height = im->height;
  ret = av_frame_get_buffer(im->frame, 0);
  if (ret < 0) {
    result.error = "could not allocate frame buffer: " + av_err(ret);
    return result;
  }

  im->pkt = av_packet_alloc();
  if (im->pkt == nullptr) {
    result.error = "could not allocate packet";
    return result;
  }

  result.encoder = std::make_unique<VideoEncoder>(std::move(im));
  return result;
}

VideoProbe probe_video(const std::filesystem::path & path)
{
  VideoProbe probe;
  const std::string path_str = path.string();

  AVFormatContext * fmt = nullptr;
  int ret = avformat_open_input(&fmt, path_str.c_str(), nullptr, nullptr);
  if (ret < 0) {
    probe.error = "could not open '" + path_str + "': " + av_err(ret);
    return probe;
  }
  ret = avformat_find_stream_info(fmt, nullptr);
  if (ret < 0) {
    probe.error = "could not read stream info: " + av_err(ret);
    avformat_close_input(&fmt);
    return probe;
  }
  const int vs = av_find_best_stream(fmt, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
  if (vs < 0) {
    probe.error = "no video stream found";
    avformat_close_input(&fmt);
    return probe;
  }
  AVStream * st = fmt->streams[vs];
  probe.width = static_cast<std::uint32_t>(st->codecpar->width);
  probe.height = static_cast<std::uint32_t>(st->codecpar->height);
  probe.codec = avcodec_get_name(st->codecpar->codec_id);

  std::int64_t count = 0;
  AVPacket * pkt = av_packet_alloc();
  if (pkt != nullptr) {
    while (av_read_frame(fmt, pkt) >= 0) {
      if (pkt->stream_index == vs) {
        ++count;
      }
      av_packet_unref(pkt);
    }
    av_packet_free(&pkt);
  }
  probe.frame_count = count;

  if (st->duration != AV_NOPTS_VALUE && st->duration > 0) {
    probe.duration_s = static_cast<double>(st->duration) * av_q2d(st->time_base);
  } else if (fmt->duration != AV_NOPTS_VALUE && fmt->duration > 0) {
    probe.duration_s = static_cast<double>(fmt->duration) / AV_TIME_BASE;
  }

  avformat_close_input(&fmt);
  return probe;
}

}  // namespace bagwiz::core::video
