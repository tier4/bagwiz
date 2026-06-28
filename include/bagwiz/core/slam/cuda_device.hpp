// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__CUDA_DEVICE_HPP_
#define BAGWIZ__CORE__SLAM__CUDA_DEVICE_HPP_

#include <string>

// GLIM-free AND CUDA-free runtime probe for the `--backend gpu` pre-flight. The
// declaration carries no CUDA types so the GLIM-free command layer
// (map_slam.cpp) can call it without pulling in <cuda_runtime_api.h>; the body
// (cuda_device.cpp) compiles the actual CUDA call only under
// BAGWIZ_WITH_SLAM_CUDA, returning a GPU-free "not a CUDA build" status
// otherwise. Always linked into a SLAM build so the call site resolves whether
// or not CUDA is enabled.
namespace bagwiz::core::slam
{

struct CudaDeviceStatus
{
  bool has_cuda_build = false;  // true iff this binary was built with BAGWIZ_WITH_SLAM_CUDA
  int device_count = 0;         // CUDA-visible devices (0 when none, or not a CUDA build)
  std::string error;            // non-empty on a CUDA runtime error (e.g. driver mismatch)
};

// Query CUDA device availability. In a non-CUDA build returns
// {has_cuda_build=false, device_count=0}; in a CUDA build runs
// cudaGetDeviceCount and reports the count or the error string.
CudaDeviceStatus query_cuda_device();

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__CUDA_DEVICE_HPP_
