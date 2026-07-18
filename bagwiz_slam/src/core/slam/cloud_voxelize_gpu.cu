// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/cloud_voxelize_gpu.hpp"

#include <cuda_runtime_api.h>
#include <thrust/device_vector.h>
#include <thrust/functional.h>
#include <thrust/host_vector.h>
#include <thrust/iterator/zip_iterator.h>
#include <thrust/reduce.h>
#include <thrust/sort.h>
#include <thrust/transform.h>
#include <thrust/tuple.h>

#include <cmath>
#include <cstdint>
#include <cstring>

namespace bagwiz::core::slam
{
namespace
{

// Per-voxel running sums + member count. operator+ folds two partials so a single
// reduce_by_key yields per-voxel sum_xyz / sum_intensity / count, from which the
// centroid and mean intensity are read back on the host.
struct VoxAccum
{
  float x;
  float y;
  float z;
  float intensity;
  std::uint32_t count;
};

struct AccumPlus
{
  __host__ __device__ VoxAccum operator()(const VoxAccum & a, const VoxAccum & b) const
  {
    return VoxAccum{a.x + b.x, a.y + b.y, a.z + b.z, a.intensity + b.intensity, a.count + b.count};
  }
};

// floor(coord / resolution) per axis, packed into a signed 64-bit voxel key
// (21 bits per axis). 21 signed bits => +/-2^20 voxels; at a 0.2 m voxel that is
// +/-200 km, beyond any real map. Coordinates past that are clamped (they would
// only collide at the very edge of an implausibly large map), which is safe
// because the GPU path is outside the reproducibility guarantee.
struct PointToKey
{
  float inv_res;
  __host__ __device__ std::int64_t operator()(const float3 & p) const
  {
    const long long lo = -(1LL << 20);
    const long long hi = (1LL << 20) - 1;
    auto quantize = [&](float v) -> std::uint64_t {
      // Clamp in float space BEFORE the cast: static_cast<long long>(+/-Inf) (or
      // NaN) is UB. !(scaled >= lo) catches -Inf and NaN (NaN compares false).
      const float scaled = floorf(v * inv_res);
      long long c;
      if (!(scaled >= static_cast<float>(lo))) {
        c = lo;
      } else if (scaled > static_cast<float>(hi)) {
        c = hi;
      } else {
        c = static_cast<long long>(scaled);
      }
      return static_cast<std::uint64_t>(c) & ((1ULL << 21) - 1);
    };
    const std::uint64_t kx = quantize(p.x);
    const std::uint64_t ky = quantize(p.y);
    const std::uint64_t kz = quantize(p.z);
    return static_cast<std::int64_t>((kx << 42) | (ky << 21) | kz);
  }
};

// Build a single-point VoxAccum from (xyz, intensity).
struct MakeAccum
{
  __host__ __device__ VoxAccum operator()(const thrust::tuple<float3, float> & t) const
  {
    const float3 p = thrust::get<0>(t);
    return VoxAccum{p.x, p.y, p.z, thrust::get<1>(t), 1u};
  }
};

}  // namespace

bool voxelize_gpu(
  const std::vector<std::array<float, 3>> & world_points, const std::vector<float> & intensities,
  double resolution, std::vector<std::array<float, 3>> & out_points,
  std::vector<float> & out_intensities)
{
  // No device -> let the caller use the CPU VoxelGrid.
  int device_count = 0;
  if (cudaGetDeviceCount(&device_count) != cudaSuccess || device_count <= 0) {
    cudaGetLastError();  // clear the sticky error so a later CUDA call is not misattributed.
    return false;
  }

  const std::size_t n = world_points.size();
  if (n == 0) {
    out_points.clear();
    out_intensities.clear();
    return true;  // an empty cloud is a successful no-op.
  }
  const bool with_intensity = intensities.size() == n;
  const float inv_res = static_cast<float>(1.0 / (resolution > 0.0 ? resolution : 1e-6));

  // thrust throws on allocation/launch failure; any failure returns false so the
  // caller silently falls back to the CPU path (the map is still produced).
  try {
    // std::array<float,3> and float3 are both three packed floats; copy through
    // memcpy (well-defined between equal-size trivially-copyable types) instead of
    // aliasing one as the other (which strict aliasing forbids), then upload.
    static_assert(sizeof(std::array<float, 3>) == sizeof(float3), "array<float,3> != float3 size");
    std::vector<float3> host_pts(n);
    std::memcpy(host_pts.data(), world_points.data(), n * sizeof(float3));
    thrust::device_vector<float3> d_pts(host_pts.begin(), host_pts.end());
    thrust::device_vector<float> d_int(n, 0.0f);
    if (with_intensity) {
      thrust::copy(intensities.begin(), intensities.end(), d_int.begin());
    }

    thrust::device_vector<std::int64_t> d_keys(n);
    thrust::transform(d_pts.begin(), d_pts.end(), d_keys.begin(), PointToKey{inv_res});

    thrust::device_vector<VoxAccum> d_acc(n);
    auto zip_begin = thrust::make_zip_iterator(thrust::make_tuple(d_pts.begin(), d_int.begin()));
    auto zip_end = thrust::make_zip_iterator(thrust::make_tuple(d_pts.end(), d_int.end()));
    thrust::transform(zip_begin, zip_end, d_acc.begin(), MakeAccum{});

    thrust::sort_by_key(d_keys.begin(), d_keys.end(), d_acc.begin());

    thrust::device_vector<std::int64_t> d_out_keys(n);
    thrust::device_vector<VoxAccum> d_out_acc(n);
    auto ends = thrust::reduce_by_key(
      d_keys.begin(), d_keys.end(), d_acc.begin(), d_out_keys.begin(), d_out_acc.begin(),
      thrust::equal_to<std::int64_t>(), AccumPlus{});
    const std::size_t m = static_cast<std::size_t>(ends.second - d_out_acc.begin());

    thrust::host_vector<VoxAccum> h_acc(d_out_acc.begin(), d_out_acc.begin() + m);
    if (cudaDeviceSynchronize() != cudaSuccess) {
      cudaGetLastError();
      return false;
    }

    out_points.resize(m);
    if (with_intensity) {
      out_intensities.resize(m);
    } else {
      out_intensities.clear();
    }
    for (std::size_t j = 0; j < m; ++j) {
      const VoxAccum & a = h_acc[j];
      const float inv_n = a.count > 0 ? 1.0f / static_cast<float>(a.count) : 0.0f;
      out_points[j] = {a.x * inv_n, a.y * inv_n, a.z * inv_n};
      if (with_intensity) {
        out_intensities[j] = a.intensity * inv_n;
      }
    }
    return true;
  } catch (...) {
    cudaGetLastError();  // swallow + clear; the caller falls back to the CPU VoxelGrid.
    return false;
  }
}

}  // namespace bagwiz::core::slam
