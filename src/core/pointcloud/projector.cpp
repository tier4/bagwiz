// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/projector.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace bagwiz::core::pointcloud
{

namespace
{

float read_field(
  const PointCloud2 & cloud, std::uint32_t point_idx, std::uint32_t offset, PointFieldType type)
{
  const std::byte * base = cloud.data.data() + point_idx * cloud.point_step + offset;
  switch (type) {
    case PointFieldType::kFloat32:
      return *reinterpret_cast<const float *>(base);
    case PointFieldType::kFloat64:
      return static_cast<float>(*reinterpret_cast<const double *>(base));
    case PointFieldType::kInt8:
      return static_cast<float>(*reinterpret_cast<const std::int8_t *>(base));
    case PointFieldType::kUint8:
      return static_cast<float>(*reinterpret_cast<const std::uint8_t *>(base));
    case PointFieldType::kInt16:
      return static_cast<float>(*reinterpret_cast<const std::int16_t *>(base));
    case PointFieldType::kUint16:
      return static_cast<float>(*reinterpret_cast<const std::uint16_t *>(base));
    case PointFieldType::kInt32:
      return static_cast<float>(*reinterpret_cast<const std::int32_t *>(base));
    case PointFieldType::kUint32:
      return static_cast<float>(*reinterpret_cast<const std::uint32_t *>(base));
  }
  return 0.0f;
}

enum class DistortionModel { kNone, kPlumbBob, kEquidistant };

DistortionModel select_distortion_model(const std::string & name)
{
  if (name == "equidistant" || name == "fisheye") {
    return DistortionModel::kEquidistant;
  }
  // plumb_bob, rational_polynomial, and an unspecified model all use the
  // radial-tangential (Brown-Conrady / rational) model below.
  return DistortionModel::kPlumbBob;
}

struct NormalizedPoint
{
  double x;
  double y;
};

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

// True when the distorted point is a fold-back artifact: the inverse distortion
// does not return to the original ray (a, b). Outside the model's valid domain
// the forward map is non-injective, so points beyond the FOV fold back into the
// image; those must not be drawn on the raw overlay.
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

// Distort a normalized ray (a, b) for the raw (unrectified) image, or return
// nullopt when it is a fold-back artifact that must not be drawn.
std::optional<NormalizedPoint> distort_for_raw_image(
  double a, double b, DistortionModel model, const std::vector<double> & d, double fx, double fy)
{
  const auto distorted = distort_normalized(a, b, model, d);
  if (is_foldback(a, b, distorted, model, d, fx, fy)) {
    return std::nullopt;
  }
  return distorted;
}

}  // namespace

ProjectionResult project_pointcloud(
  const PointCloud2 & cloud, const image::CameraInfo & camera_info,
  const std::array<double, 16> & transform, std::uint32_t image_width, std::uint32_t image_height,
  PointCloudProperty property, bool use_rectified)
{
  ProjectionResult result;

  const auto off_x = cloud.field_offset("x");
  const auto off_y = cloud.field_offset("y");
  const auto off_z = cloud.field_offset("z");
  if (!off_x || !off_y || !off_z) {
    result.error = "point cloud is missing required x/y/z fields";
    return result;
  }

  const auto find_field = [&](const std::string & name) -> const PointField * {
    for (const auto & f : cloud.fields) {
      if (f.name == name) {
        return &f;
      }
    }
    return nullptr;
  };

  const PointField * field_x = find_field("x");
  const PointField * field_y = find_field("y");
  const PointField * field_z = find_field("z");

  const bool need_intensity = (property == PointCloudProperty::kIntensity);
  std::optional<std::uint32_t> off_intensity;
  const PointField * field_intensity = nullptr;
  if (need_intensity) {
    off_intensity = cloud.field_offset("intensity");
    if (!off_intensity) {
      result.error = "point cloud has no intensity field";
      return result;
    }
    field_intensity = find_field("intensity");
  }

  const double fx = use_rectified ? camera_info.p[0] : camera_info.k[0];
  const double fy = use_rectified ? camera_info.p[5] : camera_info.k[4];
  const double cx = use_rectified ? camera_info.p[2] : camera_info.k[2];
  const double cy = use_rectified ? camera_info.p[6] : camera_info.k[5];

  // When projecting onto the raw image (use_rectified=false) apply the camera's
  // lens distortion so points land where the distorted image actually shows them.
  // The rectified path uses camera_info.p, which already assumes an undistorted
  // image, so it stays a plain pinhole projection; with no distortion
  // coefficients the raw path also reduces to a plain pinhole.
  const bool apply_distortion = !use_rectified && !camera_info.d.empty();
  const DistortionModel distortion_model = apply_distortion
                                             ? select_distortion_model(camera_info.distortion_model)
                                             : DistortionModel::kNone;

  const std::uint32_t n = cloud.height * cloud.width;
  result.points.reserve(n / 4);  // rough estimate

  for (std::uint32_t i = 0; i < n; ++i) {
    const float px = read_field(cloud, i, *off_x, field_x->datatype);
    const float py = read_field(cloud, i, *off_y, field_y->datatype);
    const float pz = read_field(cloud, i, *off_z, field_z->datatype);

    // Apply 4x4 transform: assume column-major storage matching tf2::StampedTransform.
    const double tx = transform[0] * px + transform[4] * py + transform[8] * pz + transform[12];
    const double ty = transform[1] * px + transform[5] * py + transform[9] * pz + transform[13];
    const double tz = transform[2] * px + transform[6] * py + transform[10] * pz + transform[14];

    if (tz <= 0.0) {
      continue;
    }

    double nx = tx / tz;
    double ny = ty / tz;
    if (apply_distortion) {
      // Skip distortion fold-back artifacts: outside the model's valid domain the
      // forward distortion is non-injective, so far-off-axis points (beyond the
      // camera FOV) fold back into the image and must not be drawn.
      const auto distorted = distort_for_raw_image(nx, ny, distortion_model, camera_info.d, fx, fy);
      if (!distorted) {
        continue;
      }
      nx = distorted->x;
      ny = distorted->y;
    }
    const double u = fx * nx + cx;
    const double v = fy * ny + cy;
    if (u < 0.0 || u >= image_width || v < 0.0 || v >= image_height) {
      continue;
    }

    float value = 0.0f;
    switch (property) {
      case PointCloudProperty::kX:
        value = px;
        break;
      case PointCloudProperty::kY:
        value = py;
        break;
      case PointCloudProperty::kZ:
        value = pz;
        break;
      case PointCloudProperty::kDistance:
        value = std::sqrt(px * px + py * py + pz * pz);
        break;
      case PointCloudProperty::kIntensity:
        value = read_field(cloud, i, *off_intensity, field_intensity->datatype);
        break;
    }

    ProjectedPoint pp;
    pp.u = static_cast<std::int32_t>(u);
    pp.v = static_cast<std::int32_t>(v);
    pp.depth = static_cast<float>(tz);
    pp.value = value;
    result.points.push_back(pp);
  }

  return result;
}

}  // namespace bagwiz::core::pointcloud
