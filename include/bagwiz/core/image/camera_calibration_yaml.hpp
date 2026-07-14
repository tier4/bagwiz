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
// fields it sets, plus `camera_name`.
//
// `camera_name` is not a CameraInfo field, so consumers that only populate a
// CameraInfo (such as `cam-info replace`) ignore it. It is retained anyway so
// emit_camera_calibration_yaml() can write a parsed file back out without
// silently dropping a key the original author set.
//
// File-to-field mapping:
//   image_width             -> width
//   image_height            -> height
//   distortion_model        -> distortion_model
//   distortion_coefficients -> d   (data, length rows*cols)
//   camera_matrix           -> k   (3x3, 9 values)
//   rectification_matrix    -> r   (3x3, 9 values)
//   projection_matrix       -> p   (3x4, 12 values)
//   camera_name             -> camera_name (absent when the file omits it)
struct CameraCalibration
{
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::string distortion_model;
  std::vector<double> d;                   // distortion coefficients (variable length)
  std::array<double, 9> k{};               // intrinsic camera matrix
  std::array<double, 9> r{};               // rectification matrix
  std::array<double, 12> p{};              // projection / camera matrix
  std::optional<std::string> camera_name;  // informational; not a CameraInfo field
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

// Serialize a CameraCalibration back to the camera_calibration YAML text that
// parse_camera_calibration_yaml() accepts, so that parse -> emit -> parse is a
// fixed point. Returns the document; writing it is the caller's job (this stays
// I/O-free so it can be tested without touching the filesystem).
//
// `camera_name` is written only when set. Keys are emitted in the canonical
// order the `camera_calibration` package uses, and each matrix goes out as a
// `rows`/`cols`/flat row-major `data` block.
//
// This is a re-emit, not an edit: the output is normalized YAML, so comments,
// key order, and incidental formatting from a hand-written input file are NOT
// preserved. Only the values survive.
[[nodiscard]] std::string emit_camera_calibration_yaml(const CameraCalibration & calibration);

}  // namespace bagwiz::core::image

#endif  // BAGWIZ__CORE__IMAGE__CAMERA_CALIBRATION_YAML_HPP_
