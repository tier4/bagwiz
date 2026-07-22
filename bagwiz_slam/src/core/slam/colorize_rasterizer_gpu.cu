// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/colorize_rasterizer_gpu.hpp"

#include "bagwiz/core/image/camera_distortion.hpp"
#include "bagwiz/core/slam/colorize_weight.hpp"

#include <cuda_runtime_api.h>
#include <thrust/copy.h>
#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/fill.h>
#include <thrust/functional.h>
#include <thrust/host_vector.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <span>
#include <stdexcept>
#include <vector>

namespace bagwiz::core::slam
{

namespace
{

// Bit pattern of +infinity, matching the CPU rasterizer's depth sentinel.
constexpr std::uint32_t kInfinityBits = 0x7F800000U;

// Distortion model ids for device code (mirror image::DistortionModel).
enum class GpuDistortionModel : int
{
  kNone = 0,
  kPlumbBob = 1,
  kEquidistant = 2,
};

// Camera view packaged for device use. All scalar parameters needed by the
// projection kernel are copied out of ColorizeView / CameraInfo so the kernel
// does not touch host std::vector or std::string data.
struct GpuColorizeView
{
  double r[9];
  double t[3];
  double fx;
  double fy;
  double cx;
  double cy;
  double d[8];
  int d_count;
  GpuDistortionModel distortion_model;
  std::uint32_t width;
  std::uint32_t height;
};

// Visible point in device memory. Mirrors VisiblePoint layout but uses plain
// types suitable for __device__ code and cudaMemcpy.
struct GpuVisiblePoint
{
  std::uint32_t index;
  double u;
  double v;
  float depth;
};

GpuColorizeView to_gpu_view(const ColorizeView & view)
{
  GpuColorizeView out{};
  for (int i = 0; i < 9; ++i) {
    out.r[i] = view.r_cam_world[i];
  }
  for (int i = 0; i < 3; ++i) {
    out.t[i] = view.t_cam_world[i];
  }
  out.fx = view.camera.k[0];
  out.fy = view.camera.k[4];
  out.cx = view.camera.k[2];
  out.cy = view.camera.k[5];
  out.d_count = static_cast<int>(std::min<std::size_t>(view.camera.d.size(), 8));
  for (int i = 0; i < out.d_count; ++i) {
    out.d[i] = view.camera.d[i];
  }
  out.distortion_model = static_cast<GpuDistortionModel>(
    static_cast<int>(image::select_distortion_model(view.camera.distortion_model)));
  out.width = view.width;
  out.height = view.height;
  return out;
}

__device__ double device_dist_coeff(const GpuColorizeView & view, int i)
{
  return (i < view.d_count) ? view.d[i] : 0.0;
}

__device__ void device_distort_plumb_bob(
  double a, double b, const GpuColorizeView & view, double & out_x, double & out_y)
{
  const double k1 = device_dist_coeff(view, 0);
  const double k2 = device_dist_coeff(view, 1);
  const double p1 = device_dist_coeff(view, 2);
  const double p2 = device_dist_coeff(view, 3);
  const double k3 = device_dist_coeff(view, 4);
  const double k4 = device_dist_coeff(view, 5);
  const double k5 = device_dist_coeff(view, 6);
  const double k6 = device_dist_coeff(view, 7);
  const double r2 = a * a + b * b;
  const double r4 = r2 * r2;
  const double r6 = r4 * r2;
  const double radial = (1.0 + k1 * r2 + k2 * r4 + k3 * r6) /
                        (1.0 + k4 * r2 + k5 * r4 + k6 * r6);
  out_x = a * radial + 2.0 * p1 * a * b + p2 * (r2 + 2.0 * a * a);
  out_y = b * radial + p1 * (r2 + 2.0 * b * b) + 2.0 * p2 * a * b;
}

__device__ void device_distort_equidistant(
  double a, double b, const GpuColorizeView & view, double & out_x, double & out_y)
{
  const double r = sqrt(a * a + b * b);
  if (r < 1e-9) {
    out_x = a;
    out_y = b;
    return;
  }
  const double k1 = device_dist_coeff(view, 0);
  const double k2 = device_dist_coeff(view, 1);
  const double k3 = device_dist_coeff(view, 2);
  const double k4 = device_dist_coeff(view, 3);
  const double theta = atan(r);
  const double t2 = theta * theta;
  const double t4 = t2 * t2;
  const double t6 = t4 * t2;
  const double t8 = t4 * t4;
  const double theta_d = theta * (1.0 + k1 * t2 + k2 * t4 + k3 * t6 + k4 * t8);
  const double scale = theta_d / r;
  out_x = a * scale;
  out_y = b * scale;
}

__device__ void device_distort_normalized(
  double a, double b, const GpuColorizeView & view, double & out_x, double & out_y)
{
  switch (view.distortion_model) {
    case GpuDistortionModel::kEquidistant:
      device_distort_equidistant(a, b, view, out_x, out_y);
      return;
    case GpuDistortionModel::kPlumbBob:
      device_distort_plumb_bob(a, b, view, out_x, out_y);
      return;
    case GpuDistortionModel::kNone:
    default:
      break;
  }
  out_x = a;
  out_y = b;
}

__device__ void device_undistort_plumb_bob(
  double xd, double yd, const GpuColorizeView & view, double & out_x, double & out_y)
{
  const double k1 = device_dist_coeff(view, 0);
  const double k2 = device_dist_coeff(view, 1);
  const double p1 = device_dist_coeff(view, 2);
  const double p2 = device_dist_coeff(view, 3);
  const double k3 = device_dist_coeff(view, 4);
  const double k4 = device_dist_coeff(view, 5);
  const double k5 = device_dist_coeff(view, 6);
  const double k6 = device_dist_coeff(view, 7);
  constexpr double epsilon = 1e-10;
  double x = xd;
  double y = yd;
  for (int i = 0; i < 20; ++i) {
    const double r2 = x * x + y * y;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    const double icd = (1.0 + k4 * r2 + k5 * r4 + k6 * r6) /
                       (1.0 + k1 * r2 + k2 * r4 + k3 * r6);
    const double dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
    const double dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
    const double x_next = (xd - dx) * icd;
    const double y_next = (yd - dy) * icd;
    const bool converged = fabs(x_next - x) < epsilon && fabs(y_next - y) < epsilon;
    x = x_next;
    y = y_next;
    if (converged) {
      break;
    }
  }
  out_x = x;
  out_y = y;
}

__device__ void device_undistort_equidistant(
  double xd, double yd, const GpuColorizeView & view, double & out_x, double & out_y)
{
  const double theta_d = sqrt(xd * xd + yd * yd);
  if (theta_d < 1e-9) {
    out_x = xd;
    out_y = yd;
    return;
  }
  const double k1 = device_dist_coeff(view, 0);
  const double k2 = device_dist_coeff(view, 1);
  const double k3 = device_dist_coeff(view, 2);
  const double k4 = device_dist_coeff(view, 3);
  constexpr double epsilon = 1e-10;
  double theta = theta_d;
  for (int i = 0; i < 20; ++i) {
    const double t2 = theta * theta;
    const double t4 = t2 * t2;
    const double t6 = t4 * t2;
    const double t8 = t4 * t4;
    const double theta_next = theta_d / (1.0 + k1 * t2 + k2 * t4 + k3 * t6 + k4 * t8);
    const bool converged = fabs(theta_next - theta) < epsilon;
    theta = theta_next;
    if (converged) {
      break;
    }
  }
  const double scale = tan(theta) / theta_d;
  out_x = xd * scale;
  out_y = yd * scale;
}

__device__ void device_undistort_normalized(
  double xd, double yd, const GpuColorizeView & view, double & out_x, double & out_y)
{
  switch (view.distortion_model) {
    case GpuDistortionModel::kEquidistant:
      device_undistort_equidistant(xd, yd, view, out_x, out_y);
      return;
    case GpuDistortionModel::kPlumbBob:
      device_undistort_plumb_bob(xd, yd, view, out_x, out_y);
      return;
    case GpuDistortionModel::kNone:
    default:
      break;
  }
  out_x = xd;
  out_y = yd;
}

// True when the forward-then-inverse round trip misses by more than one pixel.
__device__ bool device_distortion_round_trip_fails(
  double a, double b, double distorted_x, double distorted_y, const GpuColorizeView & view)
{
  double rx, ry;
  device_undistort_normalized(distorted_x, distorted_y, view, rx, ry);
  if (!isfinite(rx) || !isfinite(ry)) {
    return true;
  }
  const double err_u = (rx - a) * view.fx;
  const double err_v = (ry - b) * view.fy;
  return hypot(err_u, err_v) > 1.0;
}

__device__ void device_splat_depth(
  std::uint32_t * buffer, std::uint32_t width, std::uint32_t height, double u, double v,
  double radius_px, std::uint32_t depth_bits)
{
  const int center_u = static_cast<int>(u);
  const int center_v = static_cast<int>(v);
  atomicMin(
    &buffer[static_cast<std::size_t>(center_v) * width + static_cast<std::uint32_t>(center_u)],
    depth_bits);
  if (radius_px <= 0.0) {
    return;
  }
  const int u_min = max(0, static_cast<int>(ceil(u - radius_px)));
  const int u_max = min(
    static_cast<int>(width) - 1, static_cast<int>(floor(u + radius_px)));
  const int v_min = max(0, static_cast<int>(ceil(v - radius_px)));
  const int v_max = min(
    static_cast<int>(height) - 1, static_cast<int>(floor(v + radius_px)));
  const double radius_sq = radius_px * radius_px;
  for (int py = v_min; py <= v_max; ++py) {
    const double dy = static_cast<double>(py) - v;
    for (int px = u_min; px <= u_max; ++px) {
      const double dx = static_cast<double>(px) - u;
      if (dx * dx + dy * dy > radius_sq) {
        continue;
      }
      atomicMin(
        &buffer[static_cast<std::size_t>(py) * width + static_cast<std::uint32_t>(px)], depth_bits);
    }
  }
}

// Projects one world point into the view. Returns true and writes (u, v, depth,
// radius_px) when the point lands inside the image and passes the distortion
// round-trip check.
__device__ bool device_project_point(
  float3 p, const GpuColorizeView & view, double max_range_sq, double f_avg,
  double & out_u, double & out_v, float & out_depth, double & out_radius)
{
  const double x = view.r[0] * p.x + view.r[1] * p.y + view.r[2] * p.z + view.t[0];
  const double y = view.r[3] * p.x + view.r[4] * p.y + view.r[5] * p.z + view.t[1];
  const double z = view.r[6] * p.x + view.r[7] * p.y + view.r[8] * p.z + view.t[2];
  if (z <= 0.0) {
    return false;
  }
  if (x * x + y * y + z * z > max_range_sq) {
    return false;
  }
  const double nx = x / z;
  const double ny = y / z;
  double dx, dy;
  device_distort_normalized(nx, ny, view, dx, dy);
  const double u = view.fx * dx + view.cx;
  const double v = view.fy * dy + view.cy;
  if (u < 0.0 || u >= static_cast<double>(view.width) || v < 0.0 ||
      v >= static_cast<double>(view.height)) {
    return false;
  }
  if (device_distortion_round_trip_fails(nx, ny, dx, dy, view)) {
    return false;
  }
  out_u = u;
  out_v = v;
  out_depth = static_cast<float>(z);
  out_radius = 0.0;
  return true;
}

__global__ void project_and_splat_kernel(
  const float3 * points, const float * spacings, std::size_t num_points,
  GpuColorizeView view, double max_range_sq, bool splat, double splat_radius_max_px, double f_avg,
  std::uint32_t * depth_buffer, GpuVisiblePoint * candidates, std::uint32_t * candidate_count,
  std::uint32_t candidate_capacity)
{
  const std::size_t i = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (i >= num_points) {
    return;
  }
  double u, v;
  float depth;
  double radius;
  if (!device_project_point(points[i], view, max_range_sq, f_avg, u, v, depth, radius)) {
    return;
  }
  if (splat && spacings != nullptr) {
    radius = f_avg * static_cast<double>(spacings[i]) / (2.0 * static_cast<double>(depth));
    radius = fmin(fmax(radius, 0.0), splat_radius_max_px);
  }
  const std::uint32_t depth_bits = __float_as_uint(depth);
  device_splat_depth(depth_buffer, view.width, view.height, u, v, radius, depth_bits);

  const std::uint32_t slot = atomicAdd(candidate_count, 1U);
  if (slot < candidate_capacity) {
    candidates[slot] = GpuVisiblePoint{
      static_cast<std::uint32_t>(i), u, v, depth};
  }
}

__global__ void project_and_splat_dynamic_kernel(
  const float3 * points, std::size_t num_points, GpuColorizeView view, double max_range_sq,
  double dynamic_splat_spacing, double dynamic_splat_radius_min_px,
  double dynamic_splat_radius_max_px, double f_avg, std::uint32_t * depth_buffer)
{
  const std::size_t i = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (i >= num_points) {
    return;
  }
  double u, v;
  float depth;
  double radius;
  if (!device_project_point(points[i], view, max_range_sq, f_avg, u, v, depth, radius)) {
    return;
  }
  radius = f_avg * dynamic_splat_spacing / static_cast<double>(depth);
  radius = fmin(fmax(radius, dynamic_splat_radius_min_px), dynamic_splat_radius_max_px);
  const std::uint32_t depth_bits = __float_as_uint(depth);
  device_splat_depth(depth_buffer, view.width, view.height, u, v, radius, depth_bits);
}

struct KeepVisible
{
  const std::uint32_t * depth_buffer;
  const std::uint32_t * dynamic_buffer;
  std::uint32_t width;
  std::uint32_t inf_bits;
  bool has_dynamic;
  float rel;
  float abs;
  float drel;
  float dabs;

  __device__ bool operator()(const GpuVisiblePoint & c) const
  {
    const std::size_t pixel =
      static_cast<std::size_t>(static_cast<std::uint32_t>(c.v)) * width +
      static_cast<std::uint32_t>(c.u);
    const std::uint32_t dyn_bits = has_dynamic ? dynamic_buffer[pixel] : inf_bits;
    if (dyn_bits != inf_bits) {
      const float dzbuf = __uint_as_float(dyn_bits);
      return c.depth <= dzbuf * (1.0f + drel) + dabs;
    }
    const float zbuf = __uint_as_float(depth_buffer[pixel]);
    return c.depth <= zbuf * (1.0f + rel) + abs;
  }
};

constexpr int kThreadsPerBlock = 256;

}  // namespace

// Weighted observation in device memory. Mirrors ColorizeObservation layout
// but uses plain types suitable for __device__ code and cudaMemcpy.
struct GpuColorizeObservation
{
  std::uint32_t index;
  double r;
  double g;
  double b;
  double weight;
};

// Parameters for device-side weight computation, copied out of
// ObservationWeightParams so the kernel does not touch host data.
struct GpuWeightParams
{
  double distance_ref;
  double sharpness_g0;
  double border_margin_px;
};

__device__ double device_bilerp(const float * map, std::uint32_t width, std::uint32_t height, double u, double v)
{
  const double cu = fmin(fmax(u, 0.0), static_cast<double>(width - 1));
  const double cv = fmin(fmax(v, 0.0), static_cast<double>(height - 1));
  const std::uint32_t u0 = static_cast<std::uint32_t>(cu);
  const std::uint32_t v0 = static_cast<std::uint32_t>(cv);
  const std::uint32_t u1 = min(u0 + 1, width - 1);
  const std::uint32_t v1 = min(v0 + 1, height - 1);
  const double fu = cu - static_cast<double>(u0);
  const double fv = cv - static_cast<double>(v0);
  const double m00 = map[static_cast<std::size_t>(v0) * width + u0];
  const double m01 = map[static_cast<std::size_t>(v0) * width + u1];
  const double m10 = map[static_cast<std::size_t>(v1) * width + u0];
  const double m11 = map[static_cast<std::size_t>(v1) * width + u1];
  return (1.0 - fu) * (1.0 - fv) * m00 + fu * (1.0 - fv) * m01 + (1.0 - fu) * fv * m10 +
         fu * fv * m11;
}

__device__ void device_bilinear_sample_bgr(
  const std::byte * bgr, std::uint32_t width, std::uint32_t height, double u, double v, double out[3])
{
  const double cu = fmin(fmax(u, 0.0), static_cast<double>(width - 1));
  const double cv = fmin(fmax(v, 0.0), static_cast<double>(height - 1));
  const std::uint32_t u0 = static_cast<std::uint32_t>(cu);
  const std::uint32_t v0 = static_cast<std::uint32_t>(cv);
  const std::uint32_t u1 = min(u0 + 1, width - 1);
  const std::uint32_t v1 = min(v0 + 1, height - 1);
  const double fu = cu - static_cast<double>(u0);
  const double fv = cv - static_cast<double>(v0);
  const std::size_t row_stride = static_cast<std::size_t>(width) * 3;

  auto pixel_at = [&](std::uint32_t col, std::uint32_t row, std::size_t channel) -> double {
    return static_cast<double>(
      bgr[row * row_stride + col * 3 + channel]);
  };

  for (std::size_t c = 0; c < 3; ++c) {
    const double p00 = pixel_at(u0, v0, c);
    const double p10 = pixel_at(u1, v0, c);
    const double p01 = pixel_at(u0, v1, c);
    const double p11 = pixel_at(u1, v1, c);
    out[c] =
      (1.0 - fu) * (1.0 - fv) * p00 + fu * (1.0 - fv) * p10 + (1.0 - fu) * fv * p01 + fu * fv * p11;
  }
}

__device__ double device_distance_weight(float depth, double distance_ref)
{
  const double ref_over_z = distance_ref / static_cast<double>(depth);
  const double w = ref_over_z * ref_over_z;
  return fmin(fmax(w, 0.0), 1.0);
}

__device__ double device_incidence_weight(
  float3 normal, float3 point, const double cam_center[3])
{
  const double nn = static_cast<double>(normal.x) * normal.x +
                    static_cast<double>(normal.y) * normal.y +
                    static_cast<double>(normal.z) * normal.z;
  if (nn > 0.0) {
    const double dx = static_cast<double>(point.x) - cam_center[0];
    const double dy = static_cast<double>(point.y) - cam_center[1];
    const double dz = static_cast<double>(point.z) - cam_center[2];
    const double len = sqrt(dx * dx + dy * dy + dz * dz);
    if (len > 0.0) {
      return fabs(
               static_cast<double>(normal.x) * dx + static_cast<double>(normal.y) * dy +
               static_cast<double>(normal.z) * dz) /
             (sqrt(nn) * len);
    }
  }
  return 1.0;
}

__device__ double device_sharpness_weight(float g, double sharpness_g0)
{
  return sharpness_g0 > 0.0 ? static_cast<double>(g) / (static_cast<double>(g) + sharpness_g0) : 1.0;
}

__device__ double device_border_weight(
  double u, double v, std::uint32_t width, std::uint32_t height, double border_margin_px)
{
  if (border_margin_px <= 0.0) {
    return 1.0;
  }
  const double edge = fmin(fmin(u, v), fmin(static_cast<double>(width - 1) - u, static_cast<double>(height - 1) - v));
  return fmin(fmax(edge / border_margin_px, 0.0), 1.0);
}

__global__ void sobel_gradient_kernel(
  const std::byte * bgr, std::uint32_t width, std::uint32_t height, float * magnitude)
{
  const std::uint32_t col = blockIdx.x * blockDim.x + threadIdx.x;
  const std::uint32_t row = blockIdx.y * blockDim.y + threadIdx.y;
  if (col >= width || row >= height) {
    return;
  }
  const std::size_t pixel = static_cast<std::size_t>(row) * width + col;
  if (width < 3 || height < 3) {
    magnitude[pixel] = 0.0F;
    return;
  }

  auto luma = [&](std::uint32_t c, std::uint32_t r) -> float {
    const std::size_t idx = static_cast<std::size_t>(r) * width + c;
    const double b = static_cast<double>(bgr[idx * 3 + 0]);
    const double g = static_cast<double>(bgr[idx * 3 + 1]);
    const double r_ = static_cast<double>(bgr[idx * 3 + 2]);
    return static_cast<float>(0.114 * b + 0.587 * g + 0.299 * r_);
  };

  const std::uint32_t c0 = max(1U, min(col, width - 2));
  const std::uint32_t r0 = max(1U, min(row, height - 2));
  const float gx =
    (luma(c0 + 1, r0 - 1) + 2.0F * luma(c0 + 1, r0) + luma(c0 + 1, r0 + 1)) -
    (luma(c0 - 1, r0 - 1) + 2.0F * luma(c0 - 1, r0) + luma(c0 - 1, r0 + 1));
  const float gy =
    (luma(c0 - 1, r0 + 1) + 2.0F * luma(c0, r0 + 1) + luma(c0 + 1, r0 + 1)) -
    (luma(c0 - 1, r0 - 1) + 2.0F * luma(c0, r0 - 1) + luma(c0 + 1, r0 - 1));
  magnitude[pixel] = fabsf(gx) + fabsf(gy);
}

__global__ void sample_observations_kernel(
  const GpuVisiblePoint * visible, std::size_t visible_count, const float3 * points,
  const float3 * normals, bool has_normals, const double cam_center[3], const std::byte * bgr,
  std::uint32_t width, std::uint32_t height, const float * gradient, GpuWeightParams params,
  bool use_weights, double weight_min, GpuColorizeObservation * out,
  std::uint32_t * out_count, std::uint32_t out_capacity)
{
  const std::size_t i = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (i >= visible_count) {
    return;
  }
  const GpuVisiblePoint & vp = visible[i];

  double weight = 1.0;
  if (use_weights) {
    const double w_dist = device_distance_weight(vp.depth, params.distance_ref);
    double w_inc = 1.0;
    if (has_normals && vp.index < static_cast<std::uint32_t>(-1)) {
      w_inc = device_incidence_weight(normals[vp.index], points[vp.index], cam_center);
    }
    const double g = device_bilerp(gradient, width, height, vp.u, vp.v);
    const double w_sharp = device_sharpness_weight(static_cast<float>(g), params.sharpness_g0);
    const double w_border = device_border_weight(vp.u, vp.v, width, height, params.border_margin_px);
    weight = w_dist * w_inc * w_sharp * w_border;
    if (weight < weight_min) {
      return;
    }
  }

  double rgb[3];
  device_bilinear_sample_bgr(bgr, width, height, vp.u, vp.v, rgb);

  const std::uint32_t slot = atomicAdd(out_count, 1U);
  if (slot < out_capacity) {
    // BGR -> RGB.
    out[slot] = GpuColorizeObservation{vp.index, rgb[2], rgb[1], rgb[0], weight};
  }
}

class GpuColorizeRasterizer : public ColorizeRasterizer
{
public:
  GpuColorizeRasterizer(
    std::span<const std::array<float, 3>> points, std::span<const float> spacings,
    std::span<const std::array<float, 3>> normals, const ColorizeRasterizerConfig & config,
    const pointcloud::KdTree * /*tree*/)
  : config_(config),
    d_points_(points.size()),
    d_spacings_(points.size()),
    d_normals_(normals.empty() ? 0 : points.size())
  {
    if (!points.empty()) {
      static_assert(sizeof(std::array<float, 3>) == sizeof(float3), "float3 size mismatch");
      std::vector<float3> host_pts(points.size());
      std::memcpy(host_pts.data(), points.data(), points.size() * sizeof(float3));
      thrust::copy(host_pts.begin(), host_pts.end(), d_points_.begin());
    }
    if (spacings.size() == points.size()) {
      thrust::copy(spacings.begin(), spacings.end(), d_spacings_.begin());
    } else {
      d_spacings_.clear();
      d_spacings_.shrink_to_fit();
    }
    if (normals.size() == points.size()) {
      std::vector<float3> host_normals(normals.size());
      std::memcpy(host_normals.data(), normals.data(), normals.size() * sizeof(float3));
      thrust::copy(host_normals.begin(), host_normals.end(), d_normals_.begin());
      has_normals_ = true;
    }
  }

  [[nodiscard]] bool can_sample() const override { return true; }

  void visible_points(
    const ColorizeView & view, std::span<const std::array<float, 3>> dynamic_points,
    std::vector<VisiblePoint> & out) override
  {
    out.clear();
    project_and_filter(view, dynamic_points);
    if (d_visible_.empty()) {
      return;
    }
    thrust::host_vector<GpuVisiblePoint> h_visible(d_visible_);
    out.reserve(h_visible.size());
    for (const auto & c : h_visible) {
      out.push_back(VisiblePoint{c.index, c.u, c.v, c.depth});
    }
  }

  void sample_observations(
    const ColorizeView & view, std::span<const std::byte> bgr,
    std::span<const std::array<float, 3>> dynamic_points,
    const std::array<double, 3> & cam_center, std::span<const std::array<float, 3>> /*normals*/,
    bool use_weights, const ObservationWeightParams & params, double weight_min,
    std::vector<ColorizeObservation> & out) override
  {
    out.clear();
    project_and_filter(view, dynamic_points);
    if (d_visible_.empty() || bgr.size() != static_cast<std::size_t>(view.width) * view.height * 3) {
      return;
    }

    upload_image(bgr, view.width, view.height);
    compute_gradient(view.width, view.height);

    d_observations_.resize(d_visible_.size());
    thrust::device_vector<std::uint32_t> d_out_count(1, 0U);

    GpuWeightParams gpu_params{
      params.distance_ref, params.sharpness_g0, params.border_margin_px};
    const double cam_center_arr[3] = {cam_center[0], cam_center[1], cam_center[2]};
    thrust::device_vector<double> d_cam_center(cam_center_arr, cam_center_arr + 3);

    const std::size_t num_blocks =
      (d_visible_.size() + kThreadsPerBlock - 1) / kThreadsPerBlock;
    sample_observations_kernel<<<static_cast<int>(num_blocks), kThreadsPerBlock>>>(
      d_visible_.data().get(), d_visible_.size(), d_points_.data().get(), d_normals_.data().get(),
      has_normals_, d_cam_center.data().get(), d_bgr_.data().get(), view.width, view.height,
      d_gradient_.data().get(), gpu_params, use_weights, weight_min, d_observations_.data().get(),
      d_out_count.data().get(), static_cast<std::uint32_t>(d_observations_.size()));

    cudaError_t err = cudaDeviceSynchronize();
    if (err != cudaSuccess) {
      cudaGetLastError();
      throw std::runtime_error(std::string("GPU colorize sample failed: ") + cudaGetErrorString(err));
    }

    std::uint32_t out_count = d_out_count[0];
    if (out_count > d_observations_.size()) {
      out_count = static_cast<std::uint32_t>(d_observations_.size());
    }
    thrust::host_vector<GpuColorizeObservation> h_obs(d_observations_.begin(), d_observations_.begin() + out_count);
    out.reserve(h_obs.size());
    for (const auto & o : h_obs) {
      out.push_back(ColorizeObservation{o.index, o.r, o.g, o.b, o.weight});
    }
  }

private:
  void ensure_depth_buffers(std::uint32_t width, std::uint32_t height)
  {
    const std::size_t num_pixels = static_cast<std::size_t>(width) * height;
    if (width != depth_width_ || height != depth_height_) {
      depth_width_ = width;
      depth_height_ = height;
      d_depth_buffer_.assign(num_pixels, kInfinityBits);
      d_dynamic_buffer_.assign(num_pixels, kInfinityBits);
    } else {
      thrust::fill(d_depth_buffer_.begin(), d_depth_buffer_.end(), kInfinityBits);
      thrust::fill(d_dynamic_buffer_.begin(), d_dynamic_buffer_.end(), kInfinityBits);
    }
  }

  void project_and_filter(const ColorizeView & view, std::span<const std::array<float, 3>> dynamic_points)
  {
    if (d_points_.empty() || view.width == 0 || view.height == 0) {
      d_visible_.clear();
      return;
    }
    ensure_depth_buffers(view.width, view.height);

    const GpuColorizeView gpu_view = to_gpu_view(view);
    const double max_range_sq = config_.max_range > 0.0
                                  ? config_.max_range * config_.max_range
                                  : std::numeric_limits<double>::infinity();
    const double f_avg = 0.5 * (gpu_view.fx + gpu_view.fy);

    d_candidates_.resize(d_points_.size());
    thrust::device_vector<std::uint32_t> d_candidate_count(1, 0U);

    const std::size_t num_blocks =
      (d_points_.size() + kThreadsPerBlock - 1) / kThreadsPerBlock;
    const float * d_spacings_ptr = d_spacings_.empty() ? nullptr : d_spacings_.data().get();
    project_and_splat_kernel<<<static_cast<int>(num_blocks), kThreadsPerBlock>>>(
      d_points_.data().get(), d_spacings_ptr, d_points_.size(), gpu_view, max_range_sq,
      config_.splat, config_.splat_radius_max_px, f_avg, d_depth_buffer_.data().get(),
      d_candidates_.data().get(), d_candidate_count.data().get(),
      static_cast<std::uint32_t>(d_candidates_.size()));

    if (!dynamic_points.empty()) {
      std::vector<float3> host_dyn(dynamic_points.size());
      std::memcpy(host_dyn.data(), dynamic_points.data(), dynamic_points.size() * sizeof(float3));
      thrust::device_vector<float3> d_dyn(host_dyn.begin(), host_dyn.end());
      const std::size_t dyn_blocks =
        (d_dyn.size() + kThreadsPerBlock - 1) / kThreadsPerBlock;
      project_and_splat_dynamic_kernel<<<static_cast<int>(dyn_blocks), kThreadsPerBlock>>>(
        d_dyn.data().get(), d_dyn.size(), gpu_view, max_range_sq, config_.dynamic_splat_spacing,
        config_.dynamic_splat_radius_min_px, config_.dynamic_splat_radius_max_px, f_avg,
        d_dynamic_buffer_.data().get());
    }

    std::uint32_t candidate_count = d_candidate_count[0];
    if (candidate_count > d_candidates_.size()) {
      candidate_count = static_cast<std::uint32_t>(d_candidates_.size());
    }

    d_visible_.resize(candidate_count);
    KeepVisible predicate{
      d_depth_buffer_.data().get(),
      d_dynamic_buffer_.data().get(),
      view.width,
      kInfinityBits,
      !dynamic_points.empty(),
      static_cast<float>(config_.depth_rel_tolerance),
      static_cast<float>(config_.depth_abs_tolerance),
      static_cast<float>(config_.dynamic_rel_tolerance),
      static_cast<float>(config_.dynamic_abs_tolerance)};
    auto end_it = thrust::copy_if(
      thrust::device, d_candidates_.begin(), d_candidates_.begin() + candidate_count,
      d_visible_.begin(), predicate);
    d_visible_.resize(static_cast<std::size_t>(end_it - d_visible_.begin()));

    cudaError_t err = cudaDeviceSynchronize();
    if (err != cudaSuccess) {
      cudaGetLastError();
      throw std::runtime_error(std::string("GPU colorize rasterizer failed: ") + cudaGetErrorString(err));
    }
  }

  void upload_image(std::span<const std::byte> bgr, std::uint32_t width, std::uint32_t height)
  {
    const std::size_t num_bytes = static_cast<std::size_t>(width) * height * 3;
    if (d_bgr_.size() != num_bytes) {
      d_bgr_.resize(num_bytes);
    }
    cudaMemcpy(d_bgr_.data().get(), bgr.data(), num_bytes, cudaMemcpyHostToDevice);
  }

  void compute_gradient(std::uint32_t width, std::uint32_t height)
  {
    const std::size_t num_pixels = static_cast<std::size_t>(width) * height;
    if (d_gradient_.size() != num_pixels) {
      d_gradient_.resize(num_pixels);
    }
    constexpr int kGradBlock = 16;
    const dim3 block(kGradBlock, kGradBlock);
    const dim3 grid(
      (width + kGradBlock - 1) / kGradBlock, (height + kGradBlock - 1) / kGradBlock);
    sobel_gradient_kernel<<<grid, block>>>(
      d_bgr_.data().get(), width, height, d_gradient_.data().get());
  }

  ColorizeRasterizerConfig config_;
  thrust::device_vector<float3> d_points_;
  thrust::device_vector<float> d_spacings_;
  thrust::device_vector<float3> d_normals_;
  bool has_normals_ = false;
  thrust::device_vector<std::uint32_t> d_depth_buffer_;
  thrust::device_vector<std::uint32_t> d_dynamic_buffer_;
  thrust::device_vector<GpuVisiblePoint> d_candidates_;
  thrust::device_vector<GpuVisiblePoint> d_visible_;
  thrust::device_vector<std::byte> d_bgr_;
  thrust::device_vector<float> d_gradient_;
  thrust::device_vector<GpuColorizeObservation> d_observations_;
  std::uint32_t depth_width_ = 0;
  std::uint32_t depth_height_ = 0;
};

std::unique_ptr<ColorizeRasterizer> make_gpu_colorize_rasterizer(
  std::span<const std::array<float, 3>> points, std::span<const float> spacings,
  std::span<const std::array<float, 3>> normals, const ColorizeRasterizerConfig & config,
  const pointcloud::KdTree * tree)
{
  int device_count = 0;
  if (cudaGetDeviceCount(&device_count) != cudaSuccess || device_count <= 0) {
    cudaGetLastError();
    return nullptr;
  }
  try {
    return std::make_unique<GpuColorizeRasterizer>(points, spacings, normals, config, tree);
  } catch (...) {
    cudaGetLastError();
    return nullptr;
  }
}

}  // namespace bagwiz::core::slam
