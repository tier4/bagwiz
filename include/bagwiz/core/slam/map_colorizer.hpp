// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__MAP_COLORIZER_HPP_
#define BAGWIZ__CORE__SLAM__MAP_COLORIZER_HPP_

#include "bagwiz/core/image/camera_info.hpp"
#include "bagwiz/core/slam/sensor_transform.hpp"
#include "bagwiz/core/trajectory.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

// Colorize a world-frame SLAM map from a stream of camera images (`map slam
// --cam`). For each image the camera pose is interpolated from the optimized
// trajectory, the map points are projected onto the raw image with the
// camera's lens-distortion model, a per-pixel z-buffer rejects occluded
// points, and each surviving point accumulates the sampled pixel color; the
// final color is the per-point average over all observations. GLIM-free plain
// data throughout, like point_cloud_io.
namespace bagwiz::core::slam
{

struct MapColorizerConfig
{
  // Intrinsics of the RAW (unrectified) camera image: k is used for the
  // pinhole projection and d (when non-empty) applies the lens distortion so
  // points land where the distorted image actually shows them. width/height
  // are the calibrated resolution; images delivered at a different size get
  // the intrinsics rescaled per image.
  image::CameraInfo camera;

  // Camera extrinsic in the trajectory's body (cloud/LiDAR) frame:
  // p_cloud = T * p_cam, as looked up from the bag's static TF via
  // lookupTransform(cloud_frame, camera_frame). Identity mounts the camera at
  // the cloud origin looking along +z (the optical convention).
  SensorTransform t_cloud_cam;

  // Cull points farther than this from the camera position [m] before
  // projection. Bounds the per-image work and avoids coloring geometry the
  // camera cannot meaningfully resolve. <= 0 disables the cull.
  double max_range = 100.0;

  // Worker threads for the per-image projection sweep. <= 1 runs serially.
  // The result is identical for any thread count (each point accumulates at
  // most one observation per image).
  int num_threads = 4;

  // Side length [px] of one z-buffer cell. The map is sparser than the pixel
  // grid, so per-pixel occlusion would leave holes a distant occluded point
  // could slip through; coarser cells close those holes at the cost of
  // over-culling near depth edges.
  int zbuffer_cell_px = 2;

  // A point survives occlusion when its depth is within
  // cell_min_depth * (1 + depth_rel_tolerance) + depth_abs_tolerance. The
  // slack keeps same-surface neighbors (which spread in depth at grazing
  // angles) from speckling, while still rejecting genuinely occluded points.
  double depth_rel_tolerance = 0.02;
  double depth_abs_tolerance = 0.2;  // [m]
};

// Result of MapColorizer::finish().
struct MapColorizeResult
{
  // Per-point {r, g, b}, parallel to the input points. Points never observed
  // by any accepted image keep a neutral gray so they stay visible without
  // reading as colored data.
  std::vector<std::array<std::uint8_t, 3>> colors;

  std::size_t colored_points = 0;  // points with at least one observation
  std::size_t images_used = 0;     // images accumulated
  std::size_t images_skipped = 0;  // images rejected (span/raster mismatch)
};

class MapColorizer
{
public:
  // `points` are world-frame map points and `trajectory` the optimized body
  // (cloud-frame) poses sorted ascending by timestamp — exactly CloudMap's
  // points/trajectory. Both spans must outlive this object; neither is copied.
  MapColorizer(
    MapColorizerConfig config, std::span<const std::array<float, 3>> points,
    std::span<const core::TrajectoryPose> trajectory);

  MapColorizer(const MapColorizer &) = delete;
  MapColorizer & operator=(const MapColorizer &) = delete;

  // Accumulate one camera image: `bgr` is a packed BGR24 raster (exactly
  // width * 3 * height bytes, the layout image_decoder / to_packed_raster
  // produce) stamped `stamp_ns`. Returns false — counting the image as
  // skipped — when the stamp falls outside the trajectory span (a clamped
  // pose would smear colors from a wrong viewpoint) or the raster size does
  // not match. Images may arrive in any order.
  bool add_image(
    std::int64_t stamp_ns, std::span<const std::byte> bgr, std::uint32_t width,
    std::uint32_t height);

  // Average the accumulated observations into the final per-point colors.
  [[nodiscard]] MapColorizeResult finish() const;

private:
  struct Accumulator
  {
    std::uint32_t r = 0;
    std::uint32_t g = 0;
    std::uint32_t b = 0;
    std::uint32_t count = 0;
  };

  MapColorizerConfig config_;
  std::span<const std::array<float, 3>> points_;
  std::span<const core::TrajectoryPose> trajectory_;
  std::vector<Accumulator> accumulators_;  // parallel to points_
  std::size_t images_used_ = 0;
  std::size_t images_skipped_ = 0;
};

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__MAP_COLORIZER_HPP_
