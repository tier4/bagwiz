// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/visual_frontend.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <utility>
#include <vector>

namespace bagwiz::core::slam
{

struct VisualFrontend::Impl
{
  explicit Impl(VisualFrontendConfig config_in) : config(std::move(config_in)) {}

  [[nodiscard]] std::vector<VisualObservation> track(
    std::int64_t stamp_ns, std::span<const std::byte> bgr, std::uint32_t width,
    std::uint32_t height);

  VisualFrontendConfig config;

  // Previous frame, downscaled to config.tracking_width, kept for the KLT
  // optical-flow step a later task adds.
  cv::Mat prev_gray;

  // Downscale factor applied to reach config.tracking_width:
  // scale = width / tracking_width. Recomputed every call in case the
  // caller's frame size changes.
  double scale = 1.0;

  // One live feature track, in tracking-scale pixel coordinates. Detection
  // and KLT propagation land in a later task; for now this table never
  // gains an entry, so track() always returns no observations.
  struct Track
  {
    std::uint64_t id = 0;
    float x = 0.0F;
    float y = 0.0F;
  };
  std::vector<Track> tracks;
  std::uint64_t next_track_id = 0;
};

std::vector<VisualObservation> VisualFrontend::Impl::track(
  std::int64_t /*stamp_ns*/, std::span<const std::byte> bgr, std::uint32_t width,
  std::uint32_t height)
{
  if (bgr.size() != static_cast<std::size_t>(width) * height * 3) {
    return {};
  }

  // OpenCV's external-data cv::Mat constructor takes a non-const void* even
  // for read-only input; cvtColor does not mutate the source, so const_cast
  // is safe.
  const cv::Mat packed(
    static_cast<int>(height), static_cast<int>(width), CV_8UC3,
    const_cast<std::byte *>(bgr.data()));
  cv::Mat gray;
  cv::cvtColor(packed, gray, cv::COLOR_BGR2GRAY);

  const int tracking_width = std::max(1, config.tracking_width);
  scale = static_cast<double>(width) / static_cast<double>(tracking_width);
  const int tracking_height =
    std::max(1, static_cast<int>(std::lround(static_cast<double>(height) / scale)));
  cv::Mat resized;
  cv::resize(gray, resized, cv::Size(tracking_width, tracking_height), 0, 0, cv::INTER_AREA);

  std::vector<VisualObservation> observations;
  if (prev_gray.empty() || tracks.empty()) {
    // No previous frame to flow from, or no live tracks to propagate:
    // detection/KLT tracking are added in a later task, so every frame
    // still yields nothing.
    prev_gray = std::move(resized);
    return observations;
  }

  prev_gray = std::move(resized);
  return observations;
}

VisualFrontend::VisualFrontend(VisualFrontendConfig config)
: impl_(std::make_unique<Impl>(std::move(config)))
{
}

VisualFrontend::~VisualFrontend() = default;

VisualFrontend::VisualFrontend(VisualFrontend &&) noexcept = default;
VisualFrontend & VisualFrontend::operator=(VisualFrontend &&) noexcept = default;

std::vector<VisualObservation> VisualFrontend::track(
  std::int64_t stamp_ns, std::span<const std::byte> bgr, std::uint32_t width, std::uint32_t height)
{
  return impl_->track(stamp_ns, bgr, width, height);
}

}  // namespace bagwiz::core::slam
