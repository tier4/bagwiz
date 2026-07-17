// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/image/camera_distortion.hpp"

#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace bagwiz::core::image
{

namespace
{

// Cap on fixed-point iterations for the inverse (undistortion) used by the
// round-trip validity check below. The loops exit early on convergence, so this
// is only reached by slow/divergent (folded) points; OpenCV uses ~5.
constexpr int kUndistortIterations = 20;

// Fixed-point iteration stops once successive iterates change by less than this
// (normalized image units, i.e. far below a pixel): in-domain points converge in
// a handful of steps, while folded/divergent points run to the cap above.
constexpr double kUndistortEpsilon = 1e-10;

// A distorted point is treated as a fold-back artifact (and dropped) when its
// forward-then-inverse round trip misses the original ray by more than this many
// pixels. In-domain points round-trip to well under a pixel; folded points (from
// outside the model's valid domain) miss by tens of pixels or diverge to NaN.
constexpr double kMaxRoundTripErrorPx = 1.0;

// Apply OpenCV's radial-tangential (plumb_bob / rational_polynomial) distortion
// to a normalized image point (a, b) = (X/Z, Y/Z). Coefficients follow OpenCV's
// order [k1, k2, p1, p2, k3, k4, k5, k6]; any entry the vector does not carry is
// treated as zero, so both a 5-element plumb_bob and an 8-element rational model
// work.
NormalizedPoint distort_plumb_bob(double a, double b, const std::vector<double> & d)
{
  const auto coeff = [&](std::size_t i) { return i < d.size() ? d[i] : 0.0; };
  const double k1 = coeff(0);
  const double k2 = coeff(1);
  const double p1 = coeff(2);
  const double p2 = coeff(3);
  const double k3 = coeff(4);
  const double k4 = coeff(5);
  const double k5 = coeff(6);
  const double k6 = coeff(7);
  const double r2 = a * a + b * b;
  const double r4 = r2 * r2;
  const double r6 = r4 * r2;
  const double radial = (1.0 + k1 * r2 + k2 * r4 + k3 * r6) / (1.0 + k4 * r2 + k5 * r4 + k6 * r6);
  const double x = a * radial + 2.0 * p1 * a * b + p2 * (r2 + 2.0 * a * a);
  const double y = b * radial + p1 * (r2 + 2.0 * b * b) + 2.0 * p2 * a * b;
  return {x, y};
}

// Apply OpenCV's equidistant (fisheye) distortion. Coefficients are
// [k1, k2, k3, k4]; missing entries are treated as zero.
NormalizedPoint distort_equidistant(double a, double b, const std::vector<double> & d)
{
  const double r = std::sqrt(a * a + b * b);
  if (r < 1e-9) {
    return {a, b};
  }
  const auto coeff = [&](std::size_t i) { return i < d.size() ? d[i] : 0.0; };
  const double k1 = coeff(0);
  const double k2 = coeff(1);
  const double k3 = coeff(2);
  const double k4 = coeff(3);
  const double theta = std::atan(r);
  const double t2 = theta * theta;
  const double t4 = t2 * t2;
  const double t6 = t4 * t2;
  const double t8 = t4 * t4;
  const double theta_d = theta * (1.0 + k1 * t2 + k2 * t4 + k3 * t6 + k4 * t8);
  const double scale = theta_d / r;
  return {a * scale, b * scale};
}

// Invert distort_plumb_bob with OpenCV's fixed-point iteration: start from the
// distorted point and repeatedly strip off the radial/tangential terms. The
// coefficient order matches distort_plumb_bob.
NormalizedPoint undistort_plumb_bob(double xd, double yd, const std::vector<double> & d)
{
  const auto coeff = [&](std::size_t i) { return i < d.size() ? d[i] : 0.0; };
  const double k1 = coeff(0);
  const double k2 = coeff(1);
  const double p1 = coeff(2);
  const double p2 = coeff(3);
  const double k3 = coeff(4);
  const double k4 = coeff(5);
  const double k5 = coeff(6);
  const double k6 = coeff(7);
  double x = xd;
  double y = yd;
  for (int i = 0; i < kUndistortIterations; ++i) {
    const double r2 = x * x + y * y;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    const double icd = (1.0 + k4 * r2 + k5 * r4 + k6 * r6) / (1.0 + k1 * r2 + k2 * r4 + k3 * r6);
    const double dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
    const double dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
    const double x_next = (xd - dx) * icd;
    const double y_next = (yd - dy) * icd;
    const bool converged =
      std::abs(x_next - x) < kUndistortEpsilon && std::abs(y_next - y) < kUndistortEpsilon;
    x = x_next;
    y = y_next;
    if (converged) {
      break;
    }
  }
  return {x, y};
}

// Invert distort_equidistant: recover the incidence angle theta from the
// distorted radius theta_d, then scale back to the ideal normalized point.
NormalizedPoint undistort_equidistant(double xd, double yd, const std::vector<double> & d)
{
  const double theta_d = std::sqrt(xd * xd + yd * yd);
  if (theta_d < 1e-9) {
    return {xd, yd};
  }
  const auto coeff = [&](std::size_t i) { return i < d.size() ? d[i] : 0.0; };
  const double k1 = coeff(0);
  const double k2 = coeff(1);
  const double k3 = coeff(2);
  const double k4 = coeff(3);
  double theta = theta_d;
  for (int i = 0; i < kUndistortIterations; ++i) {
    const double t2 = theta * theta;
    const double t4 = t2 * t2;
    const double t6 = t4 * t2;
    const double t8 = t4 * t4;
    const double theta_next = theta_d / (1.0 + k1 * t2 + k2 * t4 + k3 * t6 + k4 * t8);
    const bool converged = std::abs(theta_next - theta) < kUndistortEpsilon;
    theta = theta_next;
    if (converged) {
      break;
    }
  }
  const double scale = std::tan(theta) / theta_d;
  return {xd * scale, yd * scale};
}

// True when the distorted point is a fold-back artifact: the inverse distortion
// does not return to the original ray (a, b).
bool is_foldback(
  double a, double b, const NormalizedPoint & distorted, DistortionModel model,
  const std::vector<double> & d, double fx, double fy)
{
  const auto recovered = undistort_normalized(distorted.x, distorted.y, model, d);
  if (!std::isfinite(recovered.x) || !std::isfinite(recovered.y)) {
    return true;
  }
  const double err_u = (recovered.x - a) * fx;
  const double err_v = (recovered.y - b) * fy;
  return std::hypot(err_u, err_v) > kMaxRoundTripErrorPx;
}

}  // namespace

DistortionModel select_distortion_model(const std::string & name)
{
  if (name == "equidistant" || name == "fisheye") {
    return DistortionModel::kEquidistant;
  }
  // plumb_bob, rational_polynomial, and an unspecified model all use the
  // radial-tangential (Brown-Conrady / rational) model.
  return DistortionModel::kPlumbBob;
}

NormalizedPoint distort_normalized(
  double a, double b, DistortionModel model, const std::vector<double> & d)
{
  switch (model) {
    case DistortionModel::kEquidistant:
      return distort_equidistant(a, b, d);
    case DistortionModel::kPlumbBob:
      return distort_plumb_bob(a, b, d);
    case DistortionModel::kNone:
      break;
  }
  return {a, b};
}

NormalizedPoint undistort_normalized(
  double xd, double yd, DistortionModel model, const std::vector<double> & d)
{
  switch (model) {
    case DistortionModel::kEquidistant:
      return undistort_equidistant(xd, yd, d);
    case DistortionModel::kPlumbBob:
      return undistort_plumb_bob(xd, yd, d);
    case DistortionModel::kNone:
      break;
  }
  return {xd, yd};
}

std::optional<NormalizedPoint> distort_for_raw_image(
  double a, double b, DistortionModel model, const std::vector<double> & d, double fx, double fy)
{
  const auto distorted = distort_normalized(a, b, model, d);
  if (is_foldback(a, b, distorted, model, d, fx, fy)) {
    return std::nullopt;
  }
  return distorted;
}

}  // namespace bagwiz::core::image
