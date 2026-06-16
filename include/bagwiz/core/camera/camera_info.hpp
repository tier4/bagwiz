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

struct CameraInfoResult {
  [[nodiscard]] bool ok() const noexcept { return image.has_value(); }
  std::string error;
  std::optional<CameraInfo> image;
};

CameraInfoResult extract_camera_info(std::span<const std::byte> payload);

}  // namespace bagwiz::core::camera

#endif  // BAGWIZ__CORE__CAMERA__CAMERA_INFO_HPP_
