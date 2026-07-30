// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__VISUAL_FRONTEND_HPP_
#define BAGWIZ__CORE__SLAM__VISUAL_FRONTEND_HPP_

#include "bagwiz/core/image/camera_info.hpp"
#include "bagwiz/core/slam/visual_observation.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

// Sparse feature tracking front end for `map slam --cam` visual constraints
// (Phase 2). Detects and KLT-tracks corner features across a stream of
// camera frames for one camera and emits one VisualObservation per live
// track per frame, already undistorted to normalized image coordinates so
// downstream factor construction stays camera-model agnostic. OpenCV is an
// implementation detail hidden behind the pimpl below; only bagwiz_image's
// CameraInfo and VisualObservation are part of the public interface.
namespace bagwiz::core::slam
{

// Tuning for one VisualFrontend instance (one camera stream).
struct VisualFrontendConfig
{
  int camera_id = 0;                   // stamped into every observation
  image::CameraInfo camera;            // full-resolution intrinsics + distortion
  int max_features = 200;              // target live track count
  int tracking_width = 960;            // frames are downscaled to this width for KLT
  double min_feature_distance = 24.0;  // px at tracking scale, detector spacing
  double forward_backward_max = 1.0;   // px at tracking scale, FB-check gate
};

class VisualFrontend
{
public:
  explicit VisualFrontend(VisualFrontendConfig config);
  ~VisualFrontend();

  VisualFrontend(VisualFrontend &&) noexcept;
  VisualFrontend & operator=(VisualFrontend &&) noexcept;

  VisualFrontend(const VisualFrontend &) = delete;
  VisualFrontend & operator=(const VisualFrontend &) = delete;

  // Track one frame (packed BGR24, stride == width*3). Returns one observation
  // per track alive in this frame (undistorted normalized coords, stamped,
  // camera_id filled). Frames must arrive in stamp order per instance. Not
  // thread-safe; use one instance per camera/thread.
  [[nodiscard]] std::vector<VisualObservation> track(
    std::int64_t stamp_ns, std::span<const std::byte> bgr, std::uint32_t width,
    std::uint32_t height);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__VISUAL_FRONTEND_HPP_
