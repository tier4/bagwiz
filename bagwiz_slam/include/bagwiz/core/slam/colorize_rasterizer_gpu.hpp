// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__COLORIZE_RASTERIZER_GPU_HPP_
#define BAGWIZ__CORE__SLAM__COLORIZE_RASTERIZER_GPU_HPP_

#include "bagwiz/core/slam/colorize_rasterizer.hpp"

#include <array>
#include <cstddef>
#include <memory>
#include <span>

// CUDA-backed ColorizeRasterizer for `bagwiz map slam --cam --backend cuda`.
// The header is CUDA-free; the implementation lives in colorize_rasterizer_gpu.cu
// and is compiled only when BAGWIZ_WITH_SLAM_CUDA is on. Callers should guard
// the factory with #ifdef BAGWIZ_WITH_SLAM_CUDA (or query_cuda_device) and fall
// back to make_cpu_colorize_rasterizer otherwise.
namespace bagwiz::core::slam
{

// GPU rasterizer factory. Returns nullptr when no CUDA device is available at
// construction time, so the caller can fall back to the CPU rasterizer.
// `tree` is accepted for API symmetry with the CPU factory but is unused: the
// GPU path projects the full point span and relies on the per-point max_range
// cull in the kernel.
[[nodiscard]] std::unique_ptr<ColorizeRasterizer> make_gpu_colorize_rasterizer(
  std::span<const std::array<float, 3>> points, std::span<const float> spacings,
  const ColorizeRasterizerConfig & config, const pointcloud::KdTree * tree = nullptr);

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__COLORIZE_RASTERIZER_GPU_HPP_
