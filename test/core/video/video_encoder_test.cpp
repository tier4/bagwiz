// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/video/video_encoder.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <span>
#include <string>
#include <system_error>
#include <vector>

namespace
{
using bagwiz::core::video::open_video_encoder;
using bagwiz::core::video::probe_video;
using bagwiz::core::video::SourcePixelFormat;

class VideoEncoderTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ =
      std::filesystem::temp_directory_path() /
      ("bagwiz_video_encoder_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) +
       "_" +
       std::to_string(
         reinterpret_cast<std::uintptr_t>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
           this)));
    std::filesystem::create_directories(tmp_dir_);
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }

  std::filesystem::path tmp_dir_;
};

std::vector<std::byte> solid_bgr(std::uint32_t w, std::uint32_t h, std::uint8_t b)
{
  std::vector<std::byte> px(static_cast<std::size_t>(w) * h * 3);
  for (std::size_t i = 0; i + 2 < px.size(); i += 3) {
    px[i] = std::byte{b};         // B
    px[i + 1] = std::byte{0x40};  // G
    px[i + 2] = std::byte{0x80};  // R
  }
  return px;
}

// MJPEG/.avi is used deliberately: the MJPEG encoder is built into every FFmpeg,
// so this test does not depend on libx264 being present in the build.
TEST_F(VideoEncoderTest, EncodesMjpegAviAndProbesBack)
{
  constexpr std::uint32_t kW = 16;
  constexpr std::uint32_t kH = 16;
  constexpr int kFrames = 8;
  const auto out = tmp_dir_ / "clip.avi";

  auto opened = open_video_encoder(out, kW, kH, 10, 1);
  ASSERT_TRUE(opened.ok()) << opened.error;

  for (int i = 0; i < kFrames; ++i) {
    const auto px = solid_bgr(kW, kH, static_cast<std::uint8_t>(i * 30));
    const auto err = opened.encoder->write_frame(
      {px.data(), px.size()}, static_cast<std::size_t>(kW) * 3, SourcePixelFormat::kBgr8);
    ASSERT_TRUE(err.empty()) << "frame " << i << ": " << err;
  }
  const auto fin = opened.encoder->finish();
  ASSERT_TRUE(fin.empty()) << fin;
  opened.encoder.reset();  // close the file before reading it back

  ASSERT_TRUE(std::filesystem::exists(out));
  const auto probe = probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.width, kW);
  EXPECT_EQ(probe.height, kH);
  EXPECT_EQ(probe.frame_count, kFrames);
}

// H.264/.mp4 is the headline output format. libx264 ships in the conda-forge
// gpl ffmpeg these environments use, but skip gracefully if a build lacks it so
// the suite stays portable.
TEST_F(VideoEncoderTest, EncodesH264Mp4WhenLibx264Available)
{
  constexpr std::uint32_t kW = 32;
  constexpr std::uint32_t kH = 16;
  constexpr int kFrames = 6;
  const auto out = tmp_dir_ / "clip.mp4";

  auto opened = open_video_encoder(out, kW, kH, 15, 1);
  if (!opened.ok()) {
    GTEST_SKIP() << "H.264 encoder unavailable: " << opened.error;
  }
  for (int i = 0; i < kFrames; ++i) {
    const auto px = solid_bgr(kW, kH, static_cast<std::uint8_t>(i * 40));
    const auto err = opened.encoder->write_frame(
      {px.data(), px.size()}, static_cast<std::size_t>(kW) * 3, SourcePixelFormat::kBgr8);
    ASSERT_TRUE(err.empty()) << "frame " << i << ": " << err;
  }
  ASSERT_TRUE(opened.encoder->finish().empty());
  opened.encoder.reset();

  const auto probe = probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.width, kW);
  EXPECT_EQ(probe.height, kH);
  EXPECT_EQ(probe.frame_count, kFrames);
  EXPECT_EQ(probe.codec, "h264");
}

TEST_F(VideoEncoderTest, RejectsUnsupportedExtension)
{
  const auto opened = open_video_encoder(tmp_dir_ / "clip.webm", 16, 16, 10, 1);
  EXPECT_FALSE(opened.ok());
  EXPECT_EQ(opened.encoder, nullptr);
}

TEST_F(VideoEncoderTest, RejectsOddDimensions)
{
  const auto opened = open_video_encoder(tmp_dir_ / "clip.avi", 15, 16, 10, 1);  // odd width
  EXPECT_FALSE(opened.ok());
}

}  // namespace
