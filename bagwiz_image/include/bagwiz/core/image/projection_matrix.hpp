// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__IMAGE__PROJECTION_MATRIX_HPP_
#define BAGWIZ__CORE__IMAGE__PROJECTION_MATRIX_HPP_

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace bagwiz::core::image
{

// The calibration a projection matrix is derived from. Field names match the
// sensor_msgs/msg/CameraInfo members (and CameraCalibration's) so callers can
// copy across without a mapping table.
//
// `r` and `p` are the *existing* values: they are not inputs to the maths, they
// are read only so compute_projection_matrix() can refuse the cases where
// recomputing p from k would silently destroy a calibration (see below).
struct ProjectionMatrixInput
{
  std::array<double, 9> k{};   // intrinsic camera matrix
  std::array<double, 9> r{};   // rectification matrix (guardrail only)
  std::array<double, 12> p{};  // existing projection matrix (guardrail only)
  std::vector<double> d;       // distortion coefficients
  std::string distortion_model;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
};

// Outcome of compute_projection_matrix(). On success `p` holds the new 3x4
// row-major projection matrix and `error` is empty; on any problem `p` is empty
// and `error` explains why. Never throws.
struct ProjectionMatrixResult
{
  std::optional<std::array<double, 12>> p;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return p.has_value() && error.empty(); }
};

// Recompute a monocular projection matrix from the intrinsics:
//
//   newK = cv::getOptimalNewCameraMatrix(k, d, (width, height), alpha)
//   p    = [ newK | 0 ]
//
// `alpha` is OpenCV's free-scaling parameter in [0, 1]: 0 crops to only valid
// pixels (the `camera_calibration` package's monocular default, so alpha=0
// recomputes the p that tool would write), 1 retains every source pixel and
// leaves black borders. Values between trade the two off.
//
// Expect a sub-pixel difference from a p that `camera_calibration` wrote
// earlier, rather than a bit-identical match: cv::getOptimalNewCameraMatrix's
// result shifts slightly across OpenCV versions (4.5.4 -> 4.13.0 moves
// fx/fy/cx/cy by up to 0.77px on a 1920x1280 plumb_bob calibration), and bagwiz
// builds against each ROS distro's own OpenCV. The recomputed p is consistent
// with the OpenCV this binary links -- which is the one UndistortHelper then
// feeds it to -- not with whatever version produced the original file.
//
// Supported `distortion_model` values -- p can only be recomputed for the
// Brown-Conrady family cv::getOptimalNewCameraMatrix implements:
//
//   plumb_bob            Brown-Conrady, 5 coefficients. The ROS default.
//   rational_polynomial  The same model with 8 coefficients.
//   "" / none            Declares no lens distortion; p is [k | 0] whatever d holds.
//
// Any other model is an error (never a silent best-effort), including the
// fisheye family below. The model is validated before `d` is examined, so an
// unsupported model is refused even when its coefficients are all zero.
//
// When the model is Brown-Conrady but `d` is empty or all-zero there is nothing
// to undistort, so the result is exactly [k | 0] and OpenCV is not consulted (it
// rejects an empty distCoeffs).
//
// Refused, because recomputing p from k alone would be wrong rather than merely
// imprecise:
//   - a genuine non-identity `r`: the camera is stereo-rectified, so p belongs
//     to cv::stereoRectify and this formula would break rectification. An
//     all-zero (unset) r is treated as identity, matching UndistortHelper.
//   - a non-zero p[3]/p[7]: p carries a stereo baseline (p[3] = -fx*baseline),
//     which [newK | 0] would silently zero out.
//   - a fisheye/equidistant `distortion_model`: needs
//     cv::fisheye::estimateNewCameraMatrixForUndistortRectify and a `balance`
//     parameter, which is different maths, not a different alpha.
//   - a zero width/height, a non-finite/degenerate k, or an out-of-range alpha.
[[nodiscard]] ProjectionMatrixResult compute_projection_matrix(
  const ProjectionMatrixInput & input, double alpha);

// The version of the library backing compute_projection_matrix() (e.g."4.13.0").
//
// Exposed because the result is version-dependent (see above), so callers that
// report a recomputed p want to name the version that produced it. Returning it
// from here keeps OpenCV a private implementation detail of bagwiz_image rather
// than something command code has to include to ask.
[[nodiscard]] std::string projection_backend_version();

}  // namespace bagwiz::core::image

#endif  // BAGWIZ__CORE__IMAGE__PROJECTION_MATRIX_HPP_
