// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__COLORIZE_KEYFRAME_HPP_
#define BAGWIZ__CORE__SLAM__COLORIZE_KEYFRAME_HPP_

#include "bagwiz/core/tf/trajectory.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

// Keyframe selection for the SLAM map colorizer (`map slam --color
// --color-min-dist`). Consecutive vehicle-camera frames are near-duplicates —
// at 10 km/h a 10 Hz camera moves 28 cm per frame, and a platform stopped at
// a light contributes hundreds of frames of identical information — while
// colorize cost is linear in the frame count. The picker thins each camera's
// stream BEFORE the colorizer sees it: a frame passes the gate only when the
// interpolated body pose moved or rotated enough since the frame that opened
// the current bucket, and with the blur refinement each bucket dispatches its
// SHARPEST member instead of its first (a motion-blurred frame is worse than
// a redundant one). Decisions depend only on the frame sequence of one
// camera, never on thread count, so a gated run is deterministic like a run
// without the gate. GLIM-free plain data throughout, like point_cloud_io.
namespace bagwiz::core::slam
{

// Rotation half of the pose gate: a frame whose interpolated body pose
// rotated at least this many degrees since the current bucket anchor opens a
// new bucket even before covering --color-min-dist, so a platform turning in
// place keeps contributing new viewpoints. Not surfaced on the CLI; the
// distance knob is the driving one on vehicle data.
inline constexpr double kKeyframeMinRotationDeg = 10.0;

struct ColorizeKeyframeConfig
{
  // Distance half of the pose gate [m]; <= 0 disables the gate entirely
  // (every frame passes, the CLI default).
  double min_dist = 0.0;

  // Rotation half of the pose gate [deg]; <= 0 disables the rotation test.
  double min_rot_deg = kKeyframeMinRotationDeg;

  // Blur refinement: dispatch each bucket's sharpest frame (by
  // image_sharpness_score) instead of its first. Requires decoding every
  // in-gate frame to score it, so the caller must use the offer()/flush()
  // path instead of accept().
  bool blur = false;
};

// Whole-image sharpness score: the mean Sobel gradient magnitude (BT.601
// luma, |gx| + |gy|, the same convention as image::sobel_gradient_magnitude)
// over a strided subsample of the interior pixels — the whole-image
// counterpart of observation_sharpness_weight's per-pixel gradient, cheap
// enough to run once per candidate frame on 4K imagery. Higher = sharper; a
// uniform image scores 0. Images smaller than 3x3 (no interior pixels) and
// rasters that are not exactly width * 3 * height bytes score 0.
// Single-threaded and deterministic.
[[nodiscard]] double image_sharpness_score(
  std::span<const std::byte> bgr, std::uint32_t width, std::uint32_t height);

// Per-camera keyframe picker. Exactly one of the two entry points is valid
// per instance, chosen by config.blur — calling the other throws
// std::logic_error (mixing them would silently drop frames: an accept()
// bucket never buffers a candidate for a later offer() to dispatch):
//
//   * blur off — accept(stamp) BEFORE decoding: the gate needs only the
//     interpolated pose, so a skipped frame never costs a decode.
//   * blur on — offer(frame) with the decoded raster (scoring needs the
//     pixels), then one flush() at end of stream for the final bucket.
//
// Frames whose stamp the colorizer would reject anyway (outside the
// trajectory span, or an empty trajectory) pass straight through without
// touching the gate state or the counters, so MapColorizer's images_skipped
// accounting is identical to a run without the gate.
//
// Not thread-safe: each camera's picker is driven from one thread in frame
// arrival order (the reader loop, or that camera's worker).
class ColorizeKeyframePicker
{
public:
  // One decoded camera frame flowing through the blur path: the resolved
  // capture stamp, the packed BGR24 raster, and the paired scan's occluder
  // points (see ScanImagePairer), carried so a frame dispatched later still
  // reaches MapColorizer::add_image with its own scan.
  struct Frame
  {
    std::int64_t stamp_ns = 0;
    std::vector<std::byte> bgr;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::vector<std::array<float, 3>> dynamic_points;
  };

  // `trajectory` is the optimized body trajectory the colorizer interpolates
  // camera poses from, sorted ascending by timestamp; the span is referenced,
  // not copied, and must outlive the picker.
  ColorizeKeyframePicker(
    ColorizeKeyframeConfig config, std::span<const core::TrajectoryPose> trajectory);

  ColorizeKeyframePicker(const ColorizeKeyframePicker &) = delete;
  ColorizeKeyframePicker & operator=(const ColorizeKeyframePicker &) = delete;

  // Pose-only gate for the blur-off path, called before the frame is
  // decoded. True = dispatch the frame to the colorizer now (it opens a new
  // bucket, or it bypasses the gate: disabled gate / out-of-span stamp /
  // empty trajectory); false = drop it (within --color-min-dist and the
  // rotation gate of the current bucket anchor).
  [[nodiscard]] bool accept(std::int64_t stamp_ns);

  // Blur-path entry: offer the next decoded frame. Returns the frame to
  // dispatch now, if any — the previous bucket's sharpest when this frame
  // opened a new bucket, or the frame itself when it bypasses the gate —
  // and std::nullopt while the current bucket is still collecting. At most
  // one frame is returned per call.
  [[nodiscard]] std::optional<Frame> offer(Frame frame);

  // End of stream for the blur path: returns the final bucket's sharpest
  // frame, or std::nullopt when no bucket is open. Idempotent.
  [[nodiscard]] std::optional<Frame> flush();

  // Frames dispatched as keyframes / dropped by the gate. Bypassing frames
  // (disabled gate, out-of-span stamp) are counted in neither.
  [[nodiscard]] std::size_t kept() const { return kept_; }
  [[nodiscard]] std::size_t skipped() const { return skipped_; }

private:
  // The interpolated body pose for a stamp the gate may act on:
  // std::nullopt when the trajectory is empty or the stamp falls outside
  // its span (the colorizer rejects those frames itself; the gate must not
  // consume them).
  [[nodiscard]] std::optional<core::TrajectoryPose> gate_pose(std::int64_t stamp_ns) const;

  // True when `pose` opens a new bucket: no bucket yet, or at least
  // min_dist meters / min_rot_deg degrees from the current bucket anchor.
  [[nodiscard]] bool gate_open(const core::TrajectoryPose & pose) const;

  struct Candidate
  {
    Frame frame;
    double score = 0.0;
  };

  ColorizeKeyframeConfig config_;
  std::span<const core::TrajectoryPose> trajectory_;
  // Pose of the frame that OPENED the current bucket (not of its sharpest
  // member): buckets tile the trajectory by the gate spacing regardless of
  // which member wins.
  std::optional<core::TrajectoryPose> anchor_;
  // The current bucket's sharpest frame so far (blur path only). Ties keep
  // the earlier frame.
  std::optional<Candidate> candidate_;
  std::size_t kept_ = 0;
  std::size_t skipped_ = 0;
};

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__COLORIZE_KEYFRAME_HPP_
