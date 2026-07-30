// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/colorize_keyframe.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace bagwiz::core::slam
{

namespace
{

// Sample stride of the sharpness sweep, in both directions: every 3rd
// interior pixel is scored. Frame RANKING is what the blur gate needs, not
// the exact full-image mean, and a 1-in-9 subsample ranks 4K frames the same
// way at a ninth of the cost (the sweep runs once per candidate image, on
// the decode thread).
constexpr std::uint32_t kSharpnessSampleStride = 3;

// BT.601 luma of the packed BGR24 pixel at (u, v), matching
// image::sobel_gradient_magnitude's grayscale conversion.
inline double luma_at(
  std::span<const std::byte> bgr, std::uint32_t width, std::uint32_t u, std::uint32_t v)
{
  const std::size_t base = (static_cast<std::size_t>(v) * width + u) * 3U;
  return 0.114 * static_cast<double>(std::to_integer<std::uint8_t>(bgr[base])) +
         0.587 * static_cast<double>(std::to_integer<std::uint8_t>(bgr[base + 1])) +
         0.299 * static_cast<double>(std::to_integer<std::uint8_t>(bgr[base + 2]));
}

}  // namespace

double image_sharpness_score(
  std::span<const std::byte> bgr, std::uint32_t width, std::uint32_t height)
{
  if (width < 3 || height < 3) {
    return 0.0;
  }
  if (bgr.size() != static_cast<std::size_t>(width) * 3U * height) {
    return 0.0;
  }
  double sum = 0.0;
  std::size_t count = 0;
  for (std::uint32_t v = 1; v + 1 < height; v += kSharpnessSampleStride) {
    for (std::uint32_t u = 1; u + 1 < width; u += kSharpnessSampleStride) {
      const double l00 = luma_at(bgr, width, u - 1, v - 1);
      const double l10 = luma_at(bgr, width, u, v - 1);
      const double l20 = luma_at(bgr, width, u + 1, v - 1);
      const double l01 = luma_at(bgr, width, u - 1, v);
      const double l21 = luma_at(bgr, width, u + 1, v);
      const double l02 = luma_at(bgr, width, u - 1, v + 1);
      const double l12 = luma_at(bgr, width, u, v + 1);
      const double l22 = luma_at(bgr, width, u + 1, v + 1);
      const double gx = (l20 + 2.0 * l21 + l22) - (l00 + 2.0 * l01 + l02);
      const double gy = (l02 + 2.0 * l12 + l22) - (l00 + 2.0 * l10 + l20);
      sum += std::abs(gx) + std::abs(gy);
      ++count;
    }
  }
  return count > 0 ? sum / static_cast<double>(count) : 0.0;
}

ColorizeKeyframePicker::ColorizeKeyframePicker(
  ColorizeKeyframeConfig config, std::span<const core::TrajectoryPose> trajectory)
: config_(config), trajectory_(trajectory)
{
}

std::optional<core::TrajectoryPose> ColorizeKeyframePicker::gate_pose(std::int64_t stamp_ns) const
{
  // Mirror MapColorizer::resolve_colorize_view's span rejection exactly: a
  // stamp the colorizer will reject (and count as skipped) must bypass the
  // gate, so a gated run's images_skipped accounting matches a run without
  // the gate.
  if (trajectory_.empty()) {
    return std::nullopt;
  }
  if (stamp_ns < trajectory_.front().timestamp_ns || stamp_ns > trajectory_.back().timestamp_ns) {
    return std::nullopt;
  }
  return core::lookup_pose(stamp_ns, trajectory_);
}

bool ColorizeKeyframePicker::gate_open(const core::TrajectoryPose & pose) const
{
  if (!anchor_) {
    return true;
  }
  if (config_.min_dist > 0.0) {
    const double dx = pose.tx - anchor_->tx;
    const double dy = pose.ty - anchor_->ty;
    const double dz = pose.tz - anchor_->tz;
    if (dx * dx + dy * dy + dz * dz >= config_.min_dist * config_.min_dist) {
      return true;
    }
  }
  if (config_.min_rot_deg > 0.0) {
    // Rotation between the two body orientations: 2 * acos(|q1 . q2|), the
    // |dot| folding q and -q (the same rotation) together.
    const double dot = std::abs(
      pose.qx * anchor_->qx + pose.qy * anchor_->qy + pose.qz * anchor_->qz +
      pose.qw * anchor_->qw);
    const double angle_deg = 2.0 * std::acos(std::clamp(dot, 0.0, 1.0)) * 180.0 / std::numbers::pi;
    if (angle_deg >= config_.min_rot_deg) {
      return true;
    }
  }
  return false;
}

bool ColorizeKeyframePicker::accept(std::int64_t stamp_ns)
{
  // Mode contract: mixing the entry points would drop frames silently — an
  // accept() bucket never buffers a candidate, so a subsequent offer() inside
  // that bucket discards its frame with nothing to dispatch later. Fail fast
  // instead of relying on the caller's gating alone.
  if (config_.blur) {
    throw std::logic_error(
      "ColorizeKeyframePicker::accept() called on a blur-configured picker; use offer()/flush()");
  }
  if (config_.min_dist <= 0.0) {
    return true;
  }
  const auto pose = gate_pose(stamp_ns);
  if (!pose) {
    return true;
  }
  if (!gate_open(*pose)) {
    ++skipped_;
    return false;
  }
  anchor_ = *pose;
  ++kept_;
  return true;
}

std::optional<ColorizeKeyframePicker::Frame> ColorizeKeyframePicker::offer(Frame frame)
{
  if (!config_.blur) {
    throw std::logic_error(
      "ColorizeKeyframePicker::offer() called on a non-blur picker; use accept()");
  }
  if (config_.min_dist <= 0.0) {
    return frame;
  }
  const auto pose = gate_pose(frame.stamp_ns);
  if (!pose) {
    return frame;
  }
  const double score = image_sharpness_score(frame.bgr, frame.width, frame.height);
  if (!gate_open(*pose)) {
    // One more bucket member that will not be dispatched: either this frame,
    // or the buffered candidate it displaces. Ties keep the earlier frame.
    ++skipped_;
    if (candidate_ && score > candidate_->score) {
      candidate_->frame = std::move(frame);
      candidate_->score = score;
    }
    return std::nullopt;
  }
  // This frame opens a new bucket: dispatch the previous bucket's sharpest
  // and buffer this frame as the new bucket's first candidate.
  std::optional<Frame> dispatched;
  if (candidate_) {
    dispatched = std::move(candidate_->frame);
    ++kept_;
  }
  anchor_ = *pose;
  candidate_ = Candidate{std::move(frame), score};
  return dispatched;
}

std::optional<ColorizeKeyframePicker::Frame> ColorizeKeyframePicker::flush()
{
  if (!config_.blur) {
    throw std::logic_error(
      "ColorizeKeyframePicker::flush() called on a non-blur picker; use accept()");
  }
  if (!candidate_) {
    return std::nullopt;
  }
  ++kept_;
  auto out = std::move(candidate_->frame);
  candidate_.reset();
  return out;
}

}  // namespace bagwiz::core::slam
