// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__CAMERA__CAMERA_INFO_HPP_
#define BAGWIZ__CORE__CAMERA__CAMERA_INFO_HPP_

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace bagwiz::core::camera
{

// A decoded sensor_msgs/msg/CameraInfo. Width/height are in pixels, K/R/P are
// the standard intrinsic, rectification, and projection matrices, and D holds
// the distortion coefficients in the order defined by distortion_model.
struct CameraInfo {
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::string frame_id;
  std::string distortion_model;
  std::array<double, 9> K{};
  std::vector<double> D;
  std::array<double, 9> R{};
  std::array<double, 12> P{};
};

// Outcome of extract_camera_info(). On success `image` holds the decoded
// CameraInfo and `error` is empty; on failure `image` is empty and `error`
// carries a human-readable reason suitable for logging. Never throws.
struct CameraInfoResult {
  std::optional<CameraInfo> image;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return image.has_value() && error.empty(); }
};

// Parse a CDR-serialized sensor_msgs/msg/CameraInfo payload and return its
// dimensions, frame_id, distortion model, and camera matrices. A truncated or
// malformed payload, or non-positive focal lengths (K[0] or K[4]), yields an
// error result rather than throwing.
[[nodiscard]] CameraInfoResult extract_camera_info(std::span<const std::byte> payload);

}  // namespace bagwiz::core::camera

#endif  // BAGWIZ__CORE__CAMERA__CAMERA_INFO_HPP_
