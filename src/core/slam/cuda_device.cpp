// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/cuda_device.hpp"

#ifdef BAGWIZ_WITH_SLAM_CUDA
#include <cuda_runtime_api.h>
#endif

namespace bagwiz::core::slam
{

CudaDeviceStatus query_cuda_device()
{
  CudaDeviceStatus status;
#ifdef BAGWIZ_WITH_SLAM_CUDA
  status.has_cuda_build = true;
  int count = 0;
  const cudaError_t err = cudaGetDeviceCount(&count);
  if (err != cudaSuccess) {
    // Surface the driver/runtime error (e.g. "no CUDA-capable device is
    // detected", driver/runtime version mismatch) so the caller can report it.
    status.error = cudaGetErrorString(err);
    // cudaGetDeviceCount leaves the error sticky; clear it so a later legitimate
    // CUDA call in the same process is not misattributed to this probe.
    cudaGetLastError();
    return status;
  }
  status.device_count = count;
#endif
  return status;
}

}  // namespace bagwiz::core::slam
