// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__CLOUD_VOXELIZE_GPU_HPP_
#define BAGWIZ__CORE__SLAM__CLOUD_VOXELIZE_GPU_HPP_

#include <array>
#include <cstddef>
#include <vector>

// CUDA voxel-grid downsampler for the exported map (Tier-2 optimization). It
// mirrors core::slam::VoxelGrid's semantics — bucket points into cubic voxels of
// side `resolution` [m] and collapse each occupied voxel to the centroid (and
// mean intensity) of its members — but runs the whole reduction on the GPU
// (thrust sort_by_key + reduce_by_key), avoiding the multi-GB single-threaded
// host pass on a large (e.g. 20 GB) bag.
//
// GLIM-free AND Eigen-free: takes plain world-frame xyz that the caller has
// already placed at the globally-optimized poses, so it slots in behind
// CloudMapper::fill_map's existing flat-array build. Declared unconditionally
// (plain C++ signature) but DEFINED only in cloud_voxelize_gpu.cu, compiled
// under BAGWIZ_WITH_SLAM_CUDA — so the only caller (#ifdef'd in cloud_mapper.cpp)
// links it in a CUDA build and uses the CPU VoxelGrid otherwise.
//
// Output order is sorted voxel-key order (NOT VoxelGrid's first-seen order); a
// PCD point cloud is unordered and the GPU path is outside the reproducibility
// guarantee, so this is intentional and harmless.
namespace bagwiz::core::slam
{

// Voxelize `world_points` on the GPU. When `intensities` is non-empty it must be
// parallel to `world_points`; the per-voxel mean intensity is then written to
// `out_intensities` (left empty otherwise). Returns true on success (outputs
// filled); returns FALSE on no CUDA device, allocation failure, or any CUDA/thrust
// error, leaving the outputs untouched so the caller falls back to the CPU
// VoxelGrid. An empty input is a successful no-op (empty outputs).
bool voxelize_gpu(
  const std::vector<std::array<float, 3>> & world_points, const std::vector<float> & intensities,
  double resolution, std::vector<std::array<float, 3>> & out_points,
  std::vector<float> & out_intensities);

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__CLOUD_VOXELIZE_GPU_HPP_
