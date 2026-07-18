// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__IMAGE__CAMERA_DISTORTION_HPP_
#define BAGWIZ__CORE__IMAGE__CAMERA_DISTORTION_HPP_

#include <optional>
#include <string>
#include <vector>

// Forward/inverse lens-distortion math shared by every consumer that projects
// 3D points onto a RAW (unrectified) camera image: the point-cloud overlay
// projector and the SLAM map colorizer. Coefficient conventions follow OpenCV
// (and therefore sensor_msgs/msg/CameraInfo). Distinct from
// core/image/undistort.hpp, which remaps whole image rasters.
namespace bagwiz::core::image
{

// Distortion model selected from CameraInfo's distortion_model string.
// plumb_bob, rational_polynomial, and an unspecified model all use the
// radial-tangential (Brown-Conrady / rational) model; equidistant/fisheye use
// OpenCV's equidistant model.
enum class DistortionModel { kNone, kPlumbBob, kEquidistant };

[[nodiscard]] DistortionModel select_distortion_model(const std::string & name);

// A point in normalized image coordinates (X/Z, Y/Z).
struct NormalizedPoint
{
  double x;
  double y;
};

// Apply the forward distortion to a normalized image point (a, b) = (X/Z, Y/Z).
// Plumb-bob coefficients follow OpenCV's order [k1, k2, p1, p2, k3, k4, k5, k6]
// and equidistant [k1, k2, k3, k4]; any entry `d` does not carry is treated as
// zero, so both a 5-element plumb_bob and an 8-element rational model work.
[[nodiscard]] NormalizedPoint distort_normalized(
  double a, double b, DistortionModel model, const std::vector<double> & d);

// Invert distort_normalized with OpenCV's fixed-point iteration. In-domain
// points converge in a handful of steps; folded/divergent points run to an
// internal iteration cap.
[[nodiscard]] NormalizedPoint undistort_normalized(
  double xd, double yd, DistortionModel model, const std::vector<double> & d);

// Distort a normalized ray (a, b) for the raw (unrectified) image, or return
// nullopt when it is a fold-back artifact that must not be used: outside the
// model's valid domain the forward map is non-injective, so points beyond the
// camera FOV fold back into the image. Detected by the forward-then-inverse
// round trip missing the original ray by more than a pixel (fx/fy convert the
// normalized error to pixels).
[[nodiscard]] std::optional<NormalizedPoint> distort_for_raw_image(
  double a, double b, DistortionModel model, const std::vector<double> & d, double fx, double fy);

}  // namespace bagwiz::core::image

#endif  // BAGWIZ__CORE__IMAGE__CAMERA_DISTORTION_HPP_
