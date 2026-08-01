// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/visual_frontend.hpp"

#include "bagwiz/core/image/camera_distortion.hpp"

#include <opencv2/core.hpp>

#include <chrono>
// goodFeaturesToTrack lives in imgproc on OpenCV 4 but moved to the features
// module on OpenCV 5; features2d.hpp is the one name both ship (a compat shim
// forwarding to features.hpp on 5). The conda humble/jazzy envs resolve
// OpenCV 4.13 while *-cuda resolves 5.0, so this TU must compile against both.
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

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
namespace
{

using Clock = std::chrono::steady_clock;

std::int64_t ns_since(Clock::time_point start)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - start).count();
}

}  // namespace

struct VisualFrontend::Impl
{
  explicit Impl(VisualFrontendConfig config_in) : config(std::move(config_in)) {}

  [[nodiscard]] std::vector<VisualObservation> track(
    std::int64_t stamp_ns, std::span<const std::byte> bgr, std::uint32_t width,
    std::uint32_t height);

  VisualFrontendConfig config;

  // Distortion model selected from config.camera at construction, shared by
  // every emitted observation's undistort_normalized call. Declared after
  // config so its initializer (which reads config.camera) runs after config
  // is constructed: member init order follows declaration order regardless
  // of the constructor's mem-initializer list.
  const image::DistortionModel model =
    image::select_distortion_model(config.camera.distortion_model);

  // Intrinsics actually used to unproject tracked pixels, rescaled from
  // config.camera to the frame size track() receives (mirrors MapColorizer's
  // handling of a decoded/republished stream that differs from the
  // CameraInfo's declared resolution: same core::image::scale_camera_info
  // call). Distortion coefficients are resolution-invariant for the
  // supported models, so only fx/fy/cx/cy need rescaling. Recomputed only
  // when the frame size changes from the last call; streams don't switch
  // resolution mid-run.
  image::CameraInfo effective_camera = config.camera;
  std::uint32_t effective_width = 0;
  std::uint32_t effective_height = 0;

  // Previous frame, downscaled to config.tracking_width, kept for the KLT
  // optical-flow step.
  cv::Mat prev_gray;

  // Downscale factor applied to reach config.tracking_width:
  // scale = width / tracking_width. Recomputed every call in case the
  // caller's frame size changes.
  double scale = 1.0;

  // One live feature track, in tracking-scale pixel coordinates.
  struct Track
  {
    std::uint64_t id = 0;
    float x = 0.0F;
    float y = 0.0F;
  };
  std::vector<Track> tracks;
  std::uint64_t next_track_id = 0;

  VisualFrontendStats stats;
};

std::vector<VisualObservation> VisualFrontend::Impl::track(
  std::int64_t stamp_ns, std::span<const std::byte> bgr, std::uint32_t width, std::uint32_t height)
{
  if (width == 0 || height == 0) {
    return {};
  }
  if (bgr.size() != static_cast<std::size_t>(width) * height * 3) {
    return {};
  }

  ++stats.frames;

  if (effective_width != width || effective_height != height) {
    effective_width = width;
    effective_height = height;
    effective_camera = config.camera;
    if (
      config.camera.width != 0 && config.camera.height != 0 &&
      (config.camera.width != width || config.camera.height != height)) {
      effective_camera = image::scale_camera_info(
        config.camera, static_cast<double>(width) / config.camera.width,
        static_cast<double>(height) / config.camera.height);
    }
  }

  // OpenCV's external-data cv::Mat constructor takes a non-const void* even
  // for read-only input; cvtColor does not mutate the source, so const_cast
  // is safe.
  const cv::Mat packed(
    static_cast<int>(height), static_cast<int>(width), CV_8UC3,
    const_cast<std::byte *>(bgr.data()));
  cv::Mat gray;
  auto t = Clock::now();
  cv::cvtColor(packed, gray, cv::COLOR_BGR2GRAY);
  stats.gray_ns += ns_since(t);

  const int tracking_width = std::max(1, config.tracking_width);
  scale = static_cast<double>(width) / static_cast<double>(tracking_width);
  const int tracking_height =
    std::max(1, static_cast<int>(std::lround(static_cast<double>(height) / scale)));
  cv::Mat resized;
  t = Clock::now();
  cv::resize(gray, resized, cv::Size(tracking_width, tracking_height), 0, 0, cv::INTER_AREA);
  stats.resize_ns += ns_since(t);

  // The very first frame this instance ever sees has no prior frame to flow
  // from, so it only seeds the detector below and reports no observations.
  const bool have_prev_frame = !prev_gray.empty();

  if (have_prev_frame && !tracks.empty()) {
    std::vector<cv::Point2f> prev_pts;
    prev_pts.reserve(tracks.size());
    for (const auto & trk : tracks) {
      prev_pts.emplace_back(trk.x, trk.y);
    }

    std::vector<cv::Point2f> next_pts;
    std::vector<uchar> status;
    std::vector<float> err;
    t = Clock::now();
    cv::calcOpticalFlowPyrLK(
      prev_gray, resized, prev_pts, next_pts, status, err, cv::Size(21, 21), 3);
    stats.klt_forward_ns += ns_since(t);

    // Forward-backward check: flow next_pts back into prev_gray and drop any
    // track whose round trip misses its origin by more than the configured
    // tolerance, along with anything OpenCV itself marked lost or that left
    // the tracking-scale frame.
    std::vector<cv::Point2f> back_pts;
    std::vector<uchar> back_status;
    std::vector<float> back_err;
    t = Clock::now();
    cv::calcOpticalFlowPyrLK(
      resized, prev_gray, next_pts, back_pts, back_status, back_err, cv::Size(21, 21), 3);
    stats.klt_backward_ns += ns_since(t);

    std::vector<Track> surviving;
    surviving.reserve(tracks.size());
    for (std::size_t i = 0; i < tracks.size(); ++i) {
      if (status[i] == 0 || back_status[i] == 0) {
        continue;
      }
      if (cv::norm(prev_pts[i] - back_pts[i]) > config.forward_backward_max) {
        continue;
      }
      const cv::Point2f & p = next_pts[i];
      if (
        p.x < 0.0F || p.y < 0.0F || p.x >= static_cast<float>(resized.cols) ||
        p.y >= static_cast<float>(resized.rows)) {
        continue;
      }
      surviving.push_back(Track{tracks[i].id, p.x, p.y});
    }
    tracks = std::move(surviving);
  }
  // Otherwise there is nothing to flow: either this is the first frame ever
  // (prev_gray empty) or every prior track was already lost (tracks empty),
  // and in both cases `tracks` is already the empty table detection tops up
  // below.

  // Top off with fresh corners when there is room, masking out a
  // min_feature_distance disc around every surviving track so detection
  // does not just rediscover the same corners.
  if (static_cast<int>(tracks.size()) < config.max_features) {
    t = Clock::now();
    cv::Mat mask(resized.size(), CV_8UC1, cv::Scalar(255));
    const int mask_radius = static_cast<int>(std::lround(config.min_feature_distance));
    for (const auto & trk : tracks) {
      cv::circle(
        mask, cv::Point(static_cast<int>(std::lround(trk.x)), static_cast<int>(std::lround(trk.y))),
        mask_radius, cv::Scalar(0), cv::FILLED);
    }

    std::vector<cv::Point2f> corners;
    const int wanted = config.max_features - static_cast<int>(tracks.size());
    cv::goodFeaturesToTrack(
      resized, corners, wanted, /*qualityLevel=*/0.01, config.min_feature_distance, mask);
    for (const auto & c : corners) {
      tracks.push_back(Track{next_track_id++, c.x, c.y});
    }
    stats.detect_ns += ns_since(t);
    ++stats.detect_calls;
  }

  std::vector<VisualObservation> observations;
  if (have_prev_frame) {
    t = Clock::now();
    const double fx = effective_camera.k[0];
    const double cx = effective_camera.k[2];
    const double fy = effective_camera.k[4];
    const double cy = effective_camera.k[5];
    observations.reserve(tracks.size());
    for (const auto & trk : tracks) {
      const double u = static_cast<double>(trk.x) * scale;
      const double v = static_cast<double>(trk.y) * scale;
      const image::NormalizedPoint p =
        image::undistort_normalized((u - cx) / fx, (v - cy) / fy, model, config.camera.d);
      // Nearest-pixel color at the track position, from the delivered
      // full-resolution frame. Track positions are inside the tracking-scale
      // frame (KLT survivors are bounds-checked, detected corners are
      // in-frame by construction), but scaling back up can land at or past
      // width/height, so clamp.
      const auto ix =
        std::clamp<std::int64_t>(std::llround(u), 0, static_cast<std::int64_t>(width) - 1);
      const auto iy =
        std::clamp<std::int64_t>(std::llround(v), 0, static_cast<std::int64_t>(height) - 1);
      const std::byte * px =
        bgr.data() + (static_cast<std::size_t>(iy) * width + static_cast<std::size_t>(ix)) * 3;
      VisualObservation obs;
      obs.camera_id = config.camera_id;
      obs.track_id = trk.id;
      obs.stamp_ns = stamp_ns;
      obs.x = p.x;
      obs.y = p.y;
      obs.rgb = {
        std::to_integer<std::uint8_t>(px[2]), std::to_integer<std::uint8_t>(px[1]),
        std::to_integer<std::uint8_t>(px[0])};
      observations.push_back(obs);
    }
    stats.emit_ns += ns_since(t);
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

const VisualFrontendStats & VisualFrontend::stats() const noexcept
{
  return impl_->stats;
}

}  // namespace bagwiz::core::slam
