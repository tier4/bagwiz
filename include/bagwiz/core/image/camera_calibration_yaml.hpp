// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__IMAGE__CAMERA_CALIBRATION_YAML_HPP_
#define BAGWIZ__CORE__IMAGE__CAMERA_CALIBRATION_YAML_HPP_

#include <array>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace bagwiz::core::image
{

// The calibration carried by a standard ROS camera_calibration /
// camera_info_manager YAML file, reduced to the sensor_msgs/msg/CameraInfo
// fields it sets. `camera_name` in the file is informational only (it is not a
// CameraInfo field) and is intentionally not stored here.
//
// File-to-field mapping:
//   image_width             -> width
//   image_height            -> height
//   distortion_model        -> distortion_model
//   distortion_coefficients -> d   (data, length rows*cols)
//   camera_matrix           -> k   (3x3, 9 values)
//   rectification_matrix    -> r   (3x3, 9 values)
//   projection_matrix       -> p   (3x4, 12 values)
struct CameraCalibration
{
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::string distortion_model;
  std::vector<double> d;       // distortion coefficients (variable length)
  std::array<double, 9> k{};   // intrinsic camera matrix
  std::array<double, 9> r{};   // rectification matrix
  std::array<double, 12> p{};  // projection / camera matrix
};

// Outcome of parse_camera_calibration_yaml(). On success `calibration` is set
// and `error` is empty; on any problem `calibration` is empty and `error`
// explains why (missing key, wrong matrix size, malformed value, unreadable
// file, ...). Never throws.
struct CameraCalibrationResult
{
  std::optional<CameraCalibration> calibration;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return calibration.has_value() && error.empty(); }
};

// Parse a standard ROS camera_calibration YAML file into a CameraCalibration.
// The format is the one produced by the `camera_calibration` package and
// consumed by `camera_info_manager`: top-level `image_width` / `image_height`
// integers, a `distortion_model` string, and `camera_matrix`,
// `distortion_coefficients`, `rectification_matrix`, `projection_matrix`
// blocks, each a mapping of `rows`, `cols`, and a flat `data` sequence in
// row-major order. The function validates that every required key is present
// and that each matrix's `data` length equals `rows * cols` and matches the
// CameraInfo field size (k/r = 9, p = 12, d non-empty). yaml-cpp parse errors
// are reported through `error` rather than thrown.
[[nodiscard]] CameraCalibrationResult parse_camera_calibration_yaml(
  const std::filesystem::path & yaml_path);

}  // namespace bagwiz::core::image

#endif  // BAGWIZ__CORE__IMAGE__CAMERA_CALIBRATION_YAML_HPP_
