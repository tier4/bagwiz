// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/image/projection_matrix.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace bagwiz::core::image
{

namespace
{

// Tolerance for "is this matrix the identity". Rectification matrices are
// written out at ~6 decimals, so an exact comparison would reject a legitimate
// identity; anything looser than this is a real rotation.
constexpr double kIdentityTol = 1e-9;

// The distortion models cv::getOptimalNewCameraMatrix's Brown-Conrady maths
// applies to, and therefore the only ones p can be recomputed for.
// `rational_polynomial` is the same model with 8 coefficients instead of 5.
[[nodiscard]] bool is_brown_conrady(const std::string & model)
{
  return model == "plumb_bob" || model == "rational_polynomial";
}

// Models that declare "this camera has no lens distortion". ROS leaves
// distortion_model empty for an already-rectified stream, and "none" appears in
// the wild; either way there is nothing to undistort, so p is [k | 0].
[[nodiscard]] bool declares_no_distortion(const std::string & model)
{
  return model.empty() || model == "none";
}

[[nodiscard]] bool is_fisheye(const std::string & model)
{
  return model == "equidistant" || model == "fisheye";
}

// True when r is a genuine rotation rather than identity. An all-zero r means
// the publisher never set it; UndistortHelper::is_usable_rotation() already
// treats that as identity, so this does too rather than refusing bags that
// undistort fine today.
[[nodiscard]] bool is_non_identity_rotation(const std::array<double, 9> & r)
{
  const bool all_zero = std::all_of(r.begin(), r.end(), [](double v) { return v == 0.0; });
  if (all_zero) {
    return false;
  }
  constexpr std::array<double, 9> identity{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  for (std::size_t i = 0; i < 9; ++i) {
    if (!std::isfinite(r[i]) || std::abs(r[i] - identity[i]) > kIdentityTol) {
      return true;
    }
  }
  return false;
}

// [k | 0]: the projection matrix of a camera with nothing to undistort.
[[nodiscard]] std::array<double, 12> k_with_zero_column(const std::array<double, 9> & k)
{
  return {k[0], k[1], k[2], 0.0, k[3], k[4], k[5], 0.0, k[6], k[7], k[8], 0.0};
}

// A NaN or infinity anywhere makes OpenCV's output meaningless rather than
// merely wrong, so every matrix is screened before it is used.
template <typename Range>
[[nodiscard]] bool all_finite(const Range & values)
{
  return std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); });
}

}  // namespace

ProjectionMatrixResult compute_projection_matrix(
  const ProjectionMatrixInput & input, const double alpha)
{
  ProjectionMatrixResult result;

  if (!std::isfinite(alpha) || alpha < 0.0 || alpha > 1.0) {
    result.error = "alpha must be in [0, 1], got " + std::to_string(alpha);
    return result;
  }

  if (input.width == 0 || input.height == 0) {
    result.error = "image size is " + std::to_string(input.width) + "x" +
                   std::to_string(input.height) +
                   "; a non-zero width and height are required to recompute p";
    return result;
  }

  if (!all_finite(input.k)) {
    result.error = "camera matrix k contains a non-finite value";
    return result;
  }
  if (input.k[0] <= 0.0 || input.k[4] <= 0.0) {
    result.error = "camera matrix k is degenerate: fx=" + std::to_string(input.k[0]) +
                   ", fy=" + std::to_string(input.k[4]) + " (both must be positive)";
    return result;
  }

  // Stereo-rectified: p comes from cv::stereoRectify against the *other*
  // camera, so [newK | 0] would silently break rectification.
  if (is_non_identity_rotation(input.r)) {
    result.error =
      "rectification matrix r is not identity, so this camera is stereo-rectified; its p is "
      "produced by cv::stereoRectify against the paired camera and cannot be recomputed from k "
      "alone";
    return result;
  }

  // A stereo right camera encodes its baseline as p[3] = -fx * baseline (and
  // p[7] for a vertical rig). [newK | 0] would zero it and lose the extrinsic.
  if (input.p[3] != 0.0 || input.p[7] != 0.0) {
    result.error =
      "projection matrix p carries a stereo baseline (p[3]=" + std::to_string(input.p[3]) +
      ", p[7]=" + std::to_string(input.p[7]) +
      "); recomputing from k would zero it and lose the baseline";
    return result;
  }

  // Validate the model before looking at d, so an unsupported model is always an
  // error. Checking it only on the has-distortion path would let a fisheye file
  // whose coefficients happen to be zero through as if it were supported, which
  // silently answers a question this function cannot answer for that camera.
  if (is_fisheye(input.distortion_model)) {
    result.error =
      "distortion_model '" + input.distortion_model +
      "' is a fisheye model; its projection matrix comes from "
      "cv::fisheye::estimateNewCameraMatrixForUndistortRectify (which takes a `balance`, not an "
      "alpha) and is not supported yet";
    return result;
  }

  // An explicit "no distortion" declaration: p is [k | 0] whatever d holds,
  // since the camera reports nothing to undistort.
  if (declares_no_distortion(input.distortion_model)) {
    result.p = k_with_zero_column(input.k);
    return result;
  }

  if (!is_brown_conrady(input.distortion_model)) {
    result.error = "distortion_model '" + input.distortion_model +
                   "' is not supported; p can only be recomputed for 'plumb_bob' or "
                   "'rational_polynomial' (an empty model or 'none' is treated as undistorted)";
    return result;
  }

  // Nothing to undistort: the optimal new camera matrix is k itself. Short-circuit
  // rather than call OpenCV, which rejects an empty distCoeffs.
  const bool has_distortion =
    !input.d.empty() &&
    std::any_of(input.d.begin(), input.d.end(), [](double v) { return v != 0.0; });
  if (!has_distortion) {
    result.p = k_with_zero_column(input.k);
    return result;
  }

  if (!all_finite(input.d)) {
    result.error = "distortion coefficients d contain a non-finite value";
    return result;
  }

  // OpenCV's external-data cv::Mat constructor takes a non-const void*; neither
  // call below mutates its input, so casting away const is safe here.
  const cv::Mat k(3, 3, CV_64F, const_cast<double *>(input.k.data()));
  const cv::Mat d(
    static_cast<int>(input.d.size()), 1, CV_64F, const_cast<double *>(input.d.data()));
  const cv::Size size{static_cast<int>(input.width), static_cast<int>(input.height)};

  cv::Mat new_k;
  try {
    new_k = cv::getOptimalNewCameraMatrix(k, d, size, alpha);
  } catch (const cv::Exception & e) {
    result.error = std::string("cv::getOptimalNewCameraMatrix failed: ") + e.what();
    return result;
  }
  if (new_k.empty() || new_k.rows != 3 || new_k.cols != 3) {
    result.error = "cv::getOptimalNewCameraMatrix returned an unusable matrix";
    return result;
  }

  std::array<double, 9> flat{};
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      flat[static_cast<std::size_t>(row * 3 + col)] = new_k.at<double>(row, col);
    }
  }
  if (!all_finite(flat)) {
    result.error = "cv::getOptimalNewCameraMatrix produced a non-finite value";
    return result;
  }

  result.p = k_with_zero_column(flat);
  return result;
}

std::string projection_backend_version()
{
  return CV_VERSION;
}

}  // namespace bagwiz::core::image
