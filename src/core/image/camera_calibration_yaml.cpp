// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/image/camera_calibration_yaml.hpp"

#include <yaml-cpp/yaml.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace bagwiz::core::image
{

namespace
{

// Read a flat, row-major matrix block of the camera_calibration form:
//
//   <key>:
//     rows: R
//     cols: C
//     data: [ R*C numbers ]
//
// `expected` is the CameraInfo field size the block must yield (9 for
// camera_matrix / rectification_matrix, 12 for projection_matrix; 0 means "any
// non-empty length", used for distortion_coefficients). On success the values
// are appended to `out` and true is returned; otherwise `error` is set and the
// function returns false. Never throws (yaml-cpp conversion failures are caught
// and surfaced through `error`).
bool read_matrix_block(
  const YAML::Node & root, const std::string & key, std::size_t expected, std::vector<double> & out,
  std::string & error)
{
  const YAML::Node block = root[key];
  if (!block) {
    error = "missing required key '" + key + "'";
    return false;
  }
  if (!block.IsMap()) {
    error = "key '" + key + "' must be a mapping with 'rows', 'cols', and 'data'";
    return false;
  }
  const YAML::Node data = block["data"];
  if (!data || !data.IsSequence()) {
    error = "key '" + key + "' must contain a 'data' sequence";
    return false;
  }

  // When rows/cols are present, cross-check them against the data length so a
  // file with a mismatched declared shape is rejected up front.
  if (block["rows"] && block["cols"]) {
    try {
      const auto rows = block["rows"].as<std::size_t>();
      const auto cols = block["cols"].as<std::size_t>();
      if (rows * cols != data.size()) {
        error = "key '" + key + "' declares rows*cols=" + std::to_string(rows * cols) +
                " but 'data' has " + std::to_string(data.size()) + " element(s)";
        return false;
      }
    } catch (const YAML::Exception & e) {
      error = "key '" + key + "' has a non-integer 'rows'/'cols': " + e.what();
      return false;
    }
  }

  if (expected != 0 && data.size() != expected) {
    error = "key '" + key + "' must have exactly " + std::to_string(expected) +
            " 'data' element(s), got " + std::to_string(data.size());
    return false;
  }
  if (expected == 0 && data.size() == 0) {
    error = "key '" + key + "' must have at least one 'data' element";
    return false;
  }

  try {
    for (const auto & v : data) {
      out.push_back(v.as<double>());
    }
  } catch (const YAML::Exception & e) {
    error = "key '" + key + "' has a non-numeric 'data' value: " + e.what();
    return false;
  }
  return true;
}

}  // namespace

CameraCalibrationResult parse_camera_calibration_yaml(const std::filesystem::path & yaml_path)
{
  CameraCalibrationResult result;

  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path.string());
  } catch (const YAML::Exception & e) {
    result.error =
      "failed to parse camera calibration YAML '" + yaml_path.string() + "': " + e.what();
    return result;
  }
  if (!root || !root.IsMap()) {
    result.error =
      "camera calibration YAML '" + yaml_path.string() + "' is empty or not a top-level mapping";
    return result;
  }

  CameraCalibration calib;

  try {
    if (!root["image_width"] || !root["image_height"]) {
      result.error = "missing required key 'image_width' and/or 'image_height'";
      return result;
    }
    calib.width = root["image_width"].as<std::uint32_t>();
    calib.height = root["image_height"].as<std::uint32_t>();
  } catch (const YAML::Exception & e) {
    result.error = std::string("'image_width'/'image_height' must be integers: ") + e.what();
    return result;
  }

  if (!root["distortion_model"]) {
    result.error = "missing required key 'distortion_model'";
    return result;
  }
  try {
    calib.distortion_model = root["distortion_model"].as<std::string>();
  } catch (const YAML::Exception & e) {
    result.error = std::string("'distortion_model' must be a string: ") + e.what();
    return result;
  }

  std::string err;

  if (!read_matrix_block(root, "distortion_coefficients", 0, calib.d, err)) {
    result.error = err;
    return result;
  }

  std::vector<double> k;
  if (!read_matrix_block(root, "camera_matrix", 9, k, err)) {
    result.error = err;
    return result;
  }
  for (std::size_t i = 0; i < 9; ++i) {
    calib.k[i] = k[i];
  }

  std::vector<double> r;
  if (!read_matrix_block(root, "rectification_matrix", 9, r, err)) {
    result.error = err;
    return result;
  }
  for (std::size_t i = 0; i < 9; ++i) {
    calib.r[i] = r[i];
  }

  std::vector<double> p;
  if (!read_matrix_block(root, "projection_matrix", 12, p, err)) {
    result.error = err;
    return result;
  }
  for (std::size_t i = 0; i < 12; ++i) {
    calib.p[i] = p[i];
  }

  result.calibration = std::move(calib);
  return result;
}

}  // namespace bagwiz::core::image
