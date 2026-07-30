// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/image/camera_distortion.hpp"
#include "bagwiz/core/slam/colorize_rasterizer_gpu.hpp"

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

// Every device-side computation below runs in FP32. On the consumer GPUs this
// backend targets (GeForce/RTX, and the Ada/Ampere Jetsons) FP64 throughput
// is 1/64 of FP32, so the previous all-double kernels ran at a small fraction
// of the device's capability. FP32 loses nothing that matters here:
//   * the input points are float3 — their storage already quantizes world
//     coordinates at |p| * 2^-23 (~0.25 mm at 2 km from the origin) — and the
//     float camera transform adds error of the same order, far below the
//     depth tolerances (0.2 m absolute) and the 1 px distortion round-trip
//     gate that consume these values;
//   * the depth buffer already held float bits and the visibility test
//     already ran in float.
// The one place double genuinely matters — composing T_cam_world from the
// interpolated trajectory pose and the extrinsic — happens host-side in
// MapColorizer::resolve_colorize_view; only the final composed view is
// narrowed to float here. The CUDA backend is explicitly outside the CPU
// reproducibility guarantee (docs/commands/map.md).

// Bit pattern of +infinity, matching the CPU rasterizer's depth sentinel.
constexpr std::uint32_t kInfinityBits = 0x7F800000U;

// Distortion model ids for device code (mirror image::DistortionModel).
enum class GpuDistortionModel : int {
  kNone = 0,
  kPlumbBob = 1,
  kEquidistant = 2,
};

// Camera view packaged for device use. All scalar parameters needed by the
// projection kernel are copied out of ColorizeView / CameraInfo — narrowed to
// FP32 — so the kernel does not touch host std::vector or std::string data.
struct GpuColorizeView
{
  float r[9];
  float t[3];
  float fx;
  float fy;
  float cx;
  float cy;
  float d[8];
  int d_count;
  GpuDistortionModel distortion_model;
  std::uint32_t width;
  std::uint32_t height;
};

// Visible point in device memory. Mirrors VisiblePoint layout but uses plain
// types suitable for __device__ code and cudaMemcpy; (u, v) are FP32 (a
// subpixel sample position is consumed at ~0.01 px precision downstream, and
// float halves the device->host copy volume).
struct GpuVisiblePoint
{
  std::uint32_t index;
  float u;
  float v;
  float depth;
};

GpuColorizeView to_gpu_view(const ColorizeView & view)
{
  GpuColorizeView out{};
  for (int i = 0; i < 9; ++i) {
    out.r[i] = static_cast<float>(view.r_cam_world[i]);
  }
  for (int i = 0; i < 3; ++i) {
    out.t[i] = static_cast<float>(view.t_cam_world[i]);
  }
  out.fx = static_cast<float>(view.camera.k[0]);
  out.fy = static_cast<float>(view.camera.k[4]);
  out.cx = static_cast<float>(view.camera.k[2]);
  out.cy = static_cast<float>(view.camera.k[5]);
  out.d_count = static_cast<int>(std::min<std::size_t>(view.camera.d.size(), 8));
  for (int i = 0; i < out.d_count; ++i) {
    out.d[i] = static_cast<float>(view.camera.d[i]);
  }
  out.distortion_model = static_cast<GpuDistortionModel>(
    static_cast<int>(image::select_distortion_model(view.camera.distortion_model)));
  out.width = view.width;
  out.height = view.height;
  return out;
}

__device__ float device_dist_coeff(const GpuColorizeView & view, int i)
{
  return (i < view.d_count) ? view.d[i] : 0.0F;
}

__device__ void device_distort_plumb_bob(
  float a, float b, const GpuColorizeView & view, float & out_x, float & out_y)
{
  const float k1 = device_dist_coeff(view, 0);
  const float k2 = device_dist_coeff(view, 1);
  const float p1 = device_dist_coeff(view, 2);
  const float p2 = device_dist_coeff(view, 3);
  const float k3 = device_dist_coeff(view, 4);
  const float k4 = device_dist_coeff(view, 5);
  const float k5 = device_dist_coeff(view, 6);
  const float k6 = device_dist_coeff(view, 7);
  const float r2 = a * a + b * b;
  const float r4 = r2 * r2;
  const float r6 = r4 * r2;
  const float radial = (1.0F + k1 * r2 + k2 * r4 + k3 * r6) / (1.0F + k4 * r2 + k5 * r4 + k6 * r6);
  out_x = a * radial + 2.0F * p1 * a * b + p2 * (r2 + 2.0F * a * a);
  out_y = b * radial + p1 * (r2 + 2.0F * b * b) + 2.0F * p2 * a * b;
}

__device__ void device_distort_equidistant(
  float a, float b, const GpuColorizeView & view, float & out_x, float & out_y)
{
  const float r = sqrtf(a * a + b * b);
  if (r < 1e-6F) {
    out_x = a;
    out_y = b;
    return;
  }
  const float k1 = device_dist_coeff(view, 0);
  const float k2 = device_dist_coeff(view, 1);
  const float k3 = device_dist_coeff(view, 2);
  const float k4 = device_dist_coeff(view, 3);
  const float theta = atanf(r);
  const float t2 = theta * theta;
  const float t4 = t2 * t2;
  const float t6 = t4 * t2;
  const float t8 = t4 * t4;
  const float theta_d = theta * (1.0F + k1 * t2 + k2 * t4 + k3 * t6 + k4 * t8);
  const float scale = theta_d / r;
  out_x = a * scale;
  out_y = b * scale;
}

__device__ void device_distort_normalized(
  float a, float b, const GpuColorizeView & view, float & out_x, float & out_y)
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

// Fixed-point iteration epsilon: the double code used 1e-10, unreachable in
// FP32 (one ulp of a normalized coordinate ~0.5 is ~6e-8, and the iteration
// dances around the fixed point by a few ulp). 1e-6 on normalized coordinates
// is under a thousandth of a pixel at any real focal length, and the 1 px
// round-trip gate downstream is the accuracy contract that matters.
constexpr float kUndistortEpsilon = 1e-6F;

__device__ void device_undistort_plumb_bob(
  float xd, float yd, const GpuColorizeView & view, float & out_x, float & out_y)
{
  const float k1 = device_dist_coeff(view, 0);
  const float k2 = device_dist_coeff(view, 1);
  const float p1 = device_dist_coeff(view, 2);
  const float p2 = device_dist_coeff(view, 3);
  const float k3 = device_dist_coeff(view, 4);
  const float k4 = device_dist_coeff(view, 5);
  const float k5 = device_dist_coeff(view, 6);
  const float k6 = device_dist_coeff(view, 7);
  float x = xd;
  float y = yd;
  for (int i = 0; i < 20; ++i) {
    const float r2 = x * x + y * y;
    const float r4 = r2 * r2;
    const float r6 = r4 * r2;
    const float icd = (1.0F + k4 * r2 + k5 * r4 + k6 * r6) / (1.0F + k1 * r2 + k2 * r4 + k3 * r6);
    const float dx = 2.0F * p1 * x * y + p2 * (r2 + 2.0F * x * x);
    const float dy = p1 * (r2 + 2.0F * y * y) + 2.0F * p2 * x * y;
    const float x_next = (xd - dx) * icd;
    const float y_next = (yd - dy) * icd;
    const bool converged =
      fabsf(x_next - x) < kUndistortEpsilon && fabsf(y_next - y) < kUndistortEpsilon;
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
  float xd, float yd, const GpuColorizeView & view, float & out_x, float & out_y)
{
  const float theta_d = sqrtf(xd * xd + yd * yd);
  if (theta_d < 1e-6F) {
    out_x = xd;
    out_y = yd;
    return;
  }
  const float k1 = device_dist_coeff(view, 0);
  const float k2 = device_dist_coeff(view, 1);
  const float k3 = device_dist_coeff(view, 2);
  const float k4 = device_dist_coeff(view, 3);
  float theta = theta_d;
  for (int i = 0; i < 20; ++i) {
    const float t2 = theta * theta;
    const float t4 = t2 * t2;
    const float t6 = t4 * t2;
    const float t8 = t4 * t4;
    const float theta_next = theta_d / (1.0F + k1 * t2 + k2 * t4 + k3 * t6 + k4 * t8);
    const bool converged = fabsf(theta_next - theta) < kUndistortEpsilon;
    theta = theta_next;
    if (converged) {
      break;
    }
  }
  const float scale = tanf(theta) / theta_d;
  out_x = xd * scale;
  out_y = yd * scale;
}

__device__ void device_undistort_normalized(
  float xd, float yd, const GpuColorizeView & view, float & out_x, float & out_y)
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
  float a, float b, float distorted_x, float distorted_y, const GpuColorizeView & view)
{
  float rx, ry;
  device_undistort_normalized(distorted_x, distorted_y, view, rx, ry);
  if (!isfinite(rx) || !isfinite(ry)) {
    return true;
  }
  const float err_u = (rx - a) * view.fx;
  const float err_v = (ry - b) * view.fy;
  return hypotf(err_u, err_v) > 1.0F;
}

__device__ void device_splat_depth(
  std::uint32_t * buffer, std::uint32_t width, std::uint32_t height, float u, float v,
  float radius_px, std::uint32_t depth_bits)
{
  const int center_u = static_cast<int>(u);
  const int center_v = static_cast<int>(v);
  atomicMin(
    &buffer[static_cast<std::size_t>(center_v) * width + static_cast<std::uint32_t>(center_u)],
    depth_bits);
  if (radius_px <= 0.0F) {
    return;
  }
  const int u_min = max(0, static_cast<int>(ceilf(u - radius_px)));
  const int u_max = min(static_cast<int>(width) - 1, static_cast<int>(floorf(u + radius_px)));
  const int v_min = max(0, static_cast<int>(ceilf(v - radius_px)));
  const int v_max = min(static_cast<int>(height) - 1, static_cast<int>(floorf(v + radius_px)));
  const float radius_sq = radius_px * radius_px;
  for (int py = v_min; py <= v_max; ++py) {
    const float dy = static_cast<float>(py) - v;
    for (int px = u_min; px <= u_max; ++px) {
      const float dx = static_cast<float>(px) - u;
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
  float3 p, const GpuColorizeView & view, float max_range_sq, float f_avg, float & out_u,
  float & out_v, float & out_depth, float & out_radius)
{
  const float x = view.r[0] * p.x + view.r[1] * p.y + view.r[2] * p.z + view.t[0];
  const float y = view.r[3] * p.x + view.r[4] * p.y + view.r[5] * p.z + view.t[1];
  const float z = view.r[6] * p.x + view.r[7] * p.y + view.r[8] * p.z + view.t[2];
  if (z <= 0.0F) {
    return false;
  }
  if (x * x + y * y + z * z > max_range_sq) {
    return false;
  }
  const float nx = x / z;
  const float ny = y / z;
  float dx, dy;
  device_distort_normalized(nx, ny, view, dx, dy);
  const float u = view.fx * dx + view.cx;
  const float v = view.fy * dy + view.cy;
  if (
    u < 0.0F || u >= static_cast<float>(view.width) || v < 0.0F ||
    v >= static_cast<float>(view.height)) {
    return false;
  }
  if (device_distortion_round_trip_fails(nx, ny, dx, dy, view)) {
    return false;
  }
  out_u = u;
  out_v = v;
  out_depth = z;
  out_radius = 0.0F;
  return true;
}

__global__ void project_and_splat_kernel(
  const float3 * points, const float * spacings, std::size_t num_points, GpuColorizeView view,
  float max_range_sq, bool splat, float splat_radius_max_px, float f_avg,
  std::uint32_t * depth_buffer, GpuVisiblePoint * candidates, std::uint32_t * candidate_count,
  std::uint32_t candidate_capacity)
{
  const std::size_t i = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (i >= num_points) {
    return;
  }
  float u, v;
  float depth;
  float radius;
  if (!device_project_point(points[i], view, max_range_sq, f_avg, u, v, depth, radius)) {
    return;
  }
  if (splat && spacings != nullptr) {
    radius = f_avg * spacings[i] / (2.0F * depth);
    radius = fminf(fmaxf(radius, 0.0F), splat_radius_max_px);
  }
  const std::uint32_t depth_bits = __float_as_uint(depth);
  device_splat_depth(depth_buffer, view.width, view.height, u, v, radius, depth_bits);

  const std::uint32_t slot = atomicAdd(candidate_count, 1U);
  if (slot < candidate_capacity) {
    candidates[slot] = GpuVisiblePoint{static_cast<std::uint32_t>(i), u, v, depth};
  }
}

__global__ void project_and_splat_dynamic_kernel(
  const float3 * points, std::size_t num_points, GpuColorizeView view, float max_range_sq,
  float dynamic_splat_spacing, float dynamic_splat_radius_min_px, float dynamic_splat_radius_max_px,
  float f_avg, std::uint32_t * depth_buffer)
{
  const std::size_t i = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (i >= num_points) {
    return;
  }
  float u, v;
  float depth;
  float radius;
  if (!device_project_point(points[i], view, max_range_sq, f_avg, u, v, depth, radius)) {
    return;
  }
  radius = f_avg * dynamic_splat_spacing / depth;
  radius = fminf(fmaxf(radius, dynamic_splat_radius_min_px), dynamic_splat_radius_max_px);
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
    const std::size_t pixel = static_cast<std::size_t>(static_cast<std::uint32_t>(c.v)) * width +
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

class GpuColorizeRasterizer : public ColorizeRasterizer
{
public:
  GpuColorizeRasterizer(
    std::span<const std::array<float, 3>> points, std::span<const float> spacings,
    const ColorizeRasterizerConfig & config, const pointcloud::KdTree * /*tree*/)
  : config_(config), d_points_(points.size()), d_spacings_(points.size())
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
      // Empty or mismatched spacings disable the data-driven splat radius; the
      // kernel will splat only the center pixel.
      d_spacings_.clear();
      d_spacings_.shrink_to_fit();
    }
  }

  void visible_points(
    const ColorizeView & view, std::span<const std::array<float, 3>> dynamic_points,
    std::vector<VisiblePoint> & out) override
  {
    out.clear();
    if (d_points_.empty() || view.width == 0 || view.height == 0) {
      return;
    }

    const std::size_t num_pixels = static_cast<std::size_t>(view.width) * view.height;
    if (view.width != depth_width_ || view.height != depth_height_) {
      depth_width_ = view.width;
      depth_height_ = view.height;
      d_depth_buffer_.assign(num_pixels, kInfinityBits);
      d_dynamic_buffer_.assign(num_pixels, kInfinityBits);
    } else {
      thrust::fill(d_depth_buffer_.begin(), d_depth_buffer_.end(), kInfinityBits);
      thrust::fill(d_dynamic_buffer_.begin(), d_dynamic_buffer_.end(), kInfinityBits);
    }

    const GpuColorizeView gpu_view = to_gpu_view(view);
    const float max_range_sq = config_.max_range > 0.0
                                 ? static_cast<float>(config_.max_range * config_.max_range)
                                 : std::numeric_limits<float>::infinity();
    const float f_avg = 0.5F * (gpu_view.fx + gpu_view.fy);

    d_candidates_.resize(d_points_.size());
    thrust::device_vector<std::uint32_t> d_candidate_count(1, 0U);

    const std::size_t num_blocks = (d_points_.size() + kThreadsPerBlock - 1) / kThreadsPerBlock;
    const float * d_spacings_ptr = d_spacings_.empty() ? nullptr : d_spacings_.data().get();
    project_and_splat_kernel<<<static_cast<int>(num_blocks), kThreadsPerBlock>>>(
      d_points_.data().get(), d_spacings_ptr, d_points_.size(), gpu_view, max_range_sq,
      config_.splat, static_cast<float>(config_.splat_radius_max_px), f_avg,
      d_depth_buffer_.data().get(), d_candidates_.data().get(), d_candidate_count.data().get(),
      static_cast<std::uint32_t>(d_candidates_.size()));

    if (!dynamic_points.empty()) {
      std::vector<float3> host_dyn(dynamic_points.size());
      std::memcpy(host_dyn.data(), dynamic_points.data(), dynamic_points.size() * sizeof(float3));
      thrust::device_vector<float3> d_dyn(host_dyn.begin(), host_dyn.end());
      const std::size_t dyn_blocks = (d_dyn.size() + kThreadsPerBlock - 1) / kThreadsPerBlock;
      project_and_splat_dynamic_kernel<<<static_cast<int>(dyn_blocks), kThreadsPerBlock>>>(
        d_dyn.data().get(), d_dyn.size(), gpu_view, max_range_sq,
        static_cast<float>(config_.dynamic_splat_spacing),
        static_cast<float>(config_.dynamic_splat_radius_min_px),
        static_cast<float>(config_.dynamic_splat_radius_max_px), f_avg,
        d_dynamic_buffer_.data().get());
    }

    std::uint32_t candidate_count = d_candidate_count[0];
    if (candidate_count > d_candidates_.size()) {
      // Atomic counter overflowed the preallocated buffer (extremely unlikely
      // unless every point projected in-bounds). Clamp and proceed; the overflow
      // slots are simply lost, matching the CPU rasterizer's behavior for a
      // degenerate all-visible scenario.
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
    const std::size_t visible_count = static_cast<std::size_t>(end_it - d_visible_.begin());

    // Synchronize before reading back; any kernel/launch error surfaces here.
    cudaError_t err = cudaDeviceSynchronize();
    if (err != cudaSuccess) {
      cudaGetLastError();
      throw std::runtime_error(
        std::string("GPU colorize rasterizer failed: ") + cudaGetErrorString(err));
    }

    if (visible_count == 0) {
      return;
    }
    thrust::host_vector<GpuVisiblePoint> h_visible(
      d_visible_.begin(), d_visible_.begin() + visible_count);
    out.reserve(visible_count);
    for (const auto & c : h_visible) {
      out.push_back(
        VisiblePoint{c.index, static_cast<double>(c.u), static_cast<double>(c.v), c.depth});
    }
  }

private:
  ColorizeRasterizerConfig config_;
  thrust::device_vector<float3> d_points_;
  thrust::device_vector<float> d_spacings_;
  thrust::device_vector<std::uint32_t> d_depth_buffer_;
  thrust::device_vector<std::uint32_t> d_dynamic_buffer_;
  thrust::device_vector<GpuVisiblePoint> d_candidates_;
  thrust::device_vector<GpuVisiblePoint> d_visible_;
  std::uint32_t depth_width_ = 0;
  std::uint32_t depth_height_ = 0;
};

std::unique_ptr<ColorizeRasterizer> make_gpu_colorize_rasterizer(
  std::span<const std::array<float, 3>> points, std::span<const float> spacings,
  const ColorizeRasterizerConfig & config, const pointcloud::KdTree * tree)
{
  int device_count = 0;
  if (cudaGetDeviceCount(&device_count) != cudaSuccess || device_count <= 0) {
    cudaGetLastError();
    return nullptr;
  }
  try {
    return std::make_unique<GpuColorizeRasterizer>(points, spacings, config, tree);
  } catch (...) {
    cudaGetLastError();
    return nullptr;
  }
}

}  // namespace bagwiz::core::slam
