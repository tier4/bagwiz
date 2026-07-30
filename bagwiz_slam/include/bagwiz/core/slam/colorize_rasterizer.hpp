// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__COLORIZE_RASTERIZER_HPP_
#define BAGWIZ__CORE__SLAM__COLORIZE_RASTERIZER_HPP_

#include "bagwiz/core/image/camera_info.hpp"
#include "bagwiz/core/pointcloud/kdtree.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

// Projection + occlusion backend seam for the SLAM map colorizer (`map slam
// --color`). Rasterizing the map into a per-image depth buffer and deciding
// which points are visible is the O(points x images) hot part of the
// colorizer; it is isolated behind the ColorizeRasterizer interface so the
// MapColorizer itself (weighting, gain compensation, robust aggregation)
// stays backend-independent. The CPU implementation below is always
// available; a future CUDA implementation can be injected from the command
// layer through the same interface. GLIM-free plain data throughout.
namespace bagwiz::core::slam
{

// One camera view to rasterize: the world->camera rigid transform and the
// intrinsics of the RAW (unrectified) image, already rescaled to the
// delivered image resolution.
struct ColorizeView
{
  std::array<double, 9> r_cam_world;  // world->camera rotation, row-major
  std::array<double, 3> t_cam_world;  // world->camera translation
  image::CameraInfo camera;           // rescaled to width x height below
  std::uint32_t width = 0;
  std::uint32_t height = 0;
};

// One map point that survived occlusion in a view: where it landed and how
// deep. (u, v) are kept as subpixel doubles so the caller can bilinear-sample
// the image; depth is the camera-frame z in meters.
struct VisiblePoint
{
  std::uint32_t index = 0;  // into the map points span
  double u = 0.0;
  double v = 0.0;
  float depth = 0.0F;
};

struct ColorizeRasterizerConfig
{
  // Cull points farther than this from the camera position [m] before
  // projection. Bounds the per-image work and avoids coloring geometry the
  // camera cannot meaningfully resolve. <= 0 disables the cull.
  double max_range = 100.0;

  // A point survives occlusion when its depth is within
  // pixel_min_depth * (1 + depth_rel_tolerance) + depth_abs_tolerance. The
  // slack keeps same-surface neighbors (which spread in depth at grazing
  // angles) from speckling, while still rejecting genuinely occluded points.
  double depth_rel_tolerance = 0.02;
  double depth_abs_tolerance = 0.2;  // [m]

  // Splat each projected point over the footprint of its local surface
  // instead of writing only its center pixel. The map is sparser than the
  // pixel grid, so per-pixel occlusion without splatting leaves holes a
  // distant occluded point could slip through; splatting closes them at the
  // cost of over-culling near depth edges.
  //
  // The footprint is the SURFEL the point stands for — the disc of radius
  // spacing/2 lying in the plane the point's normal defines — projected into
  // the image, which makes it an ellipse rather than a disc (see
  // colorize_splat.hpp). Grazing-angle geometry, which is most of what a
  // vehicle-mounted camera sees, is where that matters: its samples pile up
  // on screen along the tilt direction, so a circular footprint made every
  // point occlude its own neighbors. Points with no normal (and callers that
  // supply none) keep the isotropic disc.
  bool splat = true;

  // Cap [px] on the splat footprint's major semi-axis (the data-driven size
  // grows unbounded for isolated points; the cap keeps a stray point from
  // blanketing half the image). The ellipse is scaled down uniformly to meet
  // the cap, so the cap bounds the footprint without undoing its
  // foreshortening.
  double splat_radius_max_px = 4.0;

  // Where the per-image scan has a return at a point's pixel, it replaces
  // the static depth test: the point survives when its depth is within
  // scan_min_depth * (1 + dynamic_rel_tolerance) + dynamic_abs_tolerance.
  // The scan measured the scene at the image's own time, making it the
  // visibility oracle for pixels it covers: vehicles and pedestrians that
  // moved through without leaving map geometry occlude there (their pixels
  // never reach the points behind them), while a static surface the scan
  // confirms is accepted even when STALE dynamic geometry in the
  // accumulated map — a vehicle smeared at an earlier position — sits in
  // front. The slack absorbs voxel discretization, range noise, and
  // sub-frame motion between the scan and the image; keep it tight enough
  // that a vehicle laterally adjacent to the surface behind it still
  // rejects (their depth gap must exceed the band).
  double dynamic_rel_tolerance = 0.02;
  double dynamic_abs_tolerance = 0.3;  // [m]

  // Nominal spacing [m] between adjacent scan returns near the sensor. The
  // dynamic splat gives each scan point a depth-dependent disc radius
  // radius_px = f_avg * spacing / depth so the buffer closes the pixel-grid
  // gaps between returns (at a few meters the returns land a dozen pixels
  // apart); clamped to [dynamic_splat_radius_min_px,
  // dynamic_splat_radius_max_px]. The disc also bleeds an occluder's depth
  // onto a narrow band of surrounding pixels — a moving vehicle shifts by a
  // few pixels between the scan and the paired image — but the cap keeps
  // the bleed from swallowing an object's surroundings at close range.
  double dynamic_splat_spacing = 0.06;
  double dynamic_splat_radius_min_px = 1.5;
  double dynamic_splat_radius_max_px = 16.0;

  // Worker threads for the per-image projection/visibility sweeps (the
  // MapColorizer reuses the same value for its per-image weight, gain-vote,
  // and accumulation sweeps). <= 1 runs serially. The result is identical
  // for any thread count (the depth buffer is reduced with
  // order-independent atomic min).
  int num_threads = 4;
};

// Occlusion backend: rasterizes one view at a time into an internal depth
// buffer and reports the visible map points. Implementations may keep scratch
// state between calls (the depth buffer, per-chunk candidate lists) and are
// NOT thread-safe: callers invoke visible_points() serially.
class ColorizeRasterizer
{
public:
  virtual ~ColorizeRasterizer() = default;

  // Fills `out` with the points visible in `view` (deterministic for a fixed
  // input, independent of the configured thread count; the order itself is an
  // implementation detail — sorted by point index when unculled, the spatial
  // index's query order when a tree culls the sweep). `out` is cleared
  // first; keeping the buffer across calls avoids reallocation.
  // `dynamic_points` is the occluder geometry of the scene at the image's
  // own time — the raw LiDAR scan nearest to it, in the same world frame as
  // the map points. A map point sitting well behind a dynamic return is
  // rejected even though the accumulated map holds nothing there, which is
  // how vehicles and pedestrians are kept from staining the colors. An
  // empty span disables the dynamic test (static occlusion only).
  virtual void visible_points(
    const ColorizeView & view, std::span<const std::array<float, 3>> dynamic_points,
    std::vector<VisiblePoint> & out) = 0;
};

// CPU ColorizeRasterizer, always available. `points`, `spacings` and
// `normals` are referenced, NOT copied, and must outlive the returned
// rasterizer.
//
// `spacings` is the per-point local point spacing that sizes the splat
// footprint and must be parallel to `points` (an empty span is accepted and
// treated as all zeros, which reduces the splat to the single center pixel).
// `normals` is the per-point unit surface normal in the same WORLD frame as
// `points`, likewise parallel to them — the geometry pre-pass's
// ColorizeGeometry::normals, sign arbitrary, {0, 0, 0} meaning "no normal".
// It orients the elliptical splat footprint; an empty or mismatched span
// falls back to the isotropic disc for every point, which is exactly how the
// rasterizer behaved before footprints became normal aware.
//
// When `tree` (a spatial index over the same `points`) is non-null and
// config.max_range is positive, each view first collects only the points
// within max_range of the camera position from the tree and sweeps just
// those — far cheaper than projecting a whole-map span per image, with an
// identical visible set (the projection culls those points anyway). The tree
// is referenced, not owned.
[[nodiscard]] std::unique_ptr<ColorizeRasterizer> make_cpu_colorize_rasterizer(
  std::span<const std::array<float, 3>> points, std::span<const float> spacings,
  std::span<const std::array<float, 3>> normals, const ColorizeRasterizerConfig & config,
  const pointcloud::KdTree * tree = nullptr);

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__COLORIZE_RASTERIZER_HPP_
