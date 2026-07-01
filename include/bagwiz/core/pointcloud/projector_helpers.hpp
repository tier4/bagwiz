// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__POINTCLOUD__PROJECTOR_HELPERS_HPP_
#define BAGWIZ__CORE__POINTCLOUD__PROJECTOR_HELPERS_HPP_

#include "bagwiz/core/image/camera_info.hpp"
#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/core/pointcloud/projector.hpp"
#include "bagwiz/core/pointcloud/property.hpp"

#include <tf2/buffer_core.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace bagwiz::core::pointcloud
{

// Transform a point cloud into the camera frame and project it onto the image.
// `use_rectified` should be true when the target image has been undistorted, so
// the projection aligns with the rectified image using camera_info.p.
// If `timestamp_ns` is provided, the TF lookup uses that time; otherwise it uses
// the latest available transform.
[[nodiscard]] ProjectionResult project_cloud_for_frame(
  const PointCloud2 & cloud, const image::CameraInfo & camera_info, tf2::BufferCore & tf_buffer,
  std::uint32_t image_width, std::uint32_t image_height, PointCloudProperty property,
  bool use_rectified, std::optional<std::int64_t> timestamp_ns = std::nullopt);

}  // namespace bagwiz::core::pointcloud

#endif  // BAGWIZ__CORE__POINTCLOUD__PROJECTOR_HELPERS_HPP_
