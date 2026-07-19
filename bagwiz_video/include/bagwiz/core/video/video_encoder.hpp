// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__VIDEO__VIDEO_ENCODER_HPP_
#define BAGWIZ__CORE__VIDEO__VIDEO_ENCODER_HPP_

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <span>
#include <string>

// Thin RAII wrapper around a libav (FFmpeg) encode + mux pipeline. All
// <libav*> types are confined to the .cpp via a pimpl, so neither this header
// nor bagwiz_video's public/export surface pulls in FFmpeg headers — mirroring
// how geo_pose_convert hides GeographicLib.
namespace bagwiz::core::video
{

// Pixel layout of the source frames passed to VideoEncoder::write_frame. Both
// are 8-bit, 3-channel, interleaved — matching ROS "bgr8" / "rgb8".
enum class SourcePixelFormat { kBgr8, kRgb8 };

// One instance writes exactly one video file. Frames are pushed one at a time
// and encoded on the fly (no whole-stream buffering); finish() flushes the
// encoder's delay queue and finalizes the container.
class VideoEncoder
{
public:
  struct Impl;  // defined in the .cpp; owns the libav state

  // Takes ownership of an already-opened pipeline. Not externally
  // constructible in practice: Impl is incomplete to outside callers, so only
  // open_video_encoder() (where Impl is complete) can build one.
  explicit VideoEncoder(std::unique_ptr<Impl> impl);
  ~VideoEncoder();

  VideoEncoder(VideoEncoder &&) noexcept;
  VideoEncoder & operator=(VideoEncoder &&) noexcept;
  VideoEncoder(const VideoEncoder &) = delete;
  VideoEncoder & operator=(const VideoEncoder &) = delete;

  // Encode and mux one frame. `pixels` must hold `height` rows of `stride`
  // bytes (stride may exceed width*3 for row-padded sources). Returns an empty
  // string on success or a human-readable error; after an error the encoder
  // must not be reused and the caller should discard the partial output.
  [[nodiscard]] std::string write_frame(
    std::span<const std::byte> pixels, std::size_t stride, SourcePixelFormat format);

  // Flush the encoder and write the container trailer. Call once after the last
  // frame to produce a valid file. Returns "" on success or an error message.
  // Idempotent.
  [[nodiscard]] std::string finish();

private:
  std::unique_ptr<Impl> impl_;
};

// Outcome of open_video_encoder(). On success `encoder` is non-null and `error`
// is empty; on failure `encoder` is null and `error` explains why.
struct OpenVideoEncoderResult
{
  std::unique_ptr<VideoEncoder> encoder;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return encoder != nullptr && error.empty(); }
};

// Open an encoder writing to `output`. Container/codec are inferred from the
// extension: `.mp4` / `.mkv` / `.mov` -> H.264 (libx264, YUV420P);
// `.avi` -> MJPEG. `fps_num`/`fps_den` set the constant frame rate. Failures
// (unsupported extension, missing codec in this FFmpeg build, unwritable path,
// odd dimensions for a codec that needs even ones) return a null encoder with a
// descriptive error.
OpenVideoEncoderResult open_video_encoder(
  const std::filesystem::path & output, std::uint32_t width, std::uint32_t height, int fps_num,
  int fps_den);

// Basic stream facts read back from an encoded file. Lets tests (and a future
// `--verify`) confirm an output is a real, playable video without shelling out
// to ffprobe.
struct VideoProbe
{
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::int64_t frame_count = 0;  // video packets counted from the container
  double duration_s = 0.0;
  std::string codec;
  bool has_b_frames = false;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return error.empty(); }
};

VideoProbe probe_video(const std::filesystem::path & path);

}  // namespace bagwiz::core::video

#endif  // BAGWIZ__CORE__VIDEO__VIDEO_ENCODER_HPP_
