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
#include "bagwiz/core/pointcloud/kdtree.hpp"
#include "bagwiz/core/slam/colorize_rasterizer.hpp"
#include "bagwiz/core/slam/sensor_transform.hpp"
#include "bagwiz/core/tf/trajectory.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

// Colorize a world-frame SLAM map from a stream of camera images (`map slam
// --cam`). For each image the camera pose is interpolated from the optimized
// trajectory and a ColorizeRasterizer projects the map points onto the raw
// image with the camera's lens-distortion model, splatting a per-pixel depth
// buffer to reject occluded points. Each surviving observation is weighted
// (depth distance, surface incidence, image sharpness, image border),
// corrected by a per-image gain estimate that tracks auto-exposure / white
// balance drift (the gain only lifts underexposed frames toward the
// established reference, and only appearance-stable points vote, so genuine
// brightening cannot ratchet the stored colors toward black), and stored in a
// bounded per-point reservoir; finish() reduces each reservoir by trimming
// around the 75th-luminance-percentile observation and averaging the
// survivors — shadows are illumination, not surface color, so the lit-mode
// cluster wins, while moving-object and occlusion-leak outliers are trimmed
// away. A geometry pre-pass (kd-tree, surface normals, local point spacings)
// feeds the incidence weight and the splat footprint. When the caller also
// supplies the raw LiDAR scan nearest an image (add_image's dynamic_points),
// map points sitting well behind a scan return are skipped for that image,
// so vehicles and pedestrians that left no geometry in the accumulated map
// do not stain the colors either.
// GLIM-free plain data throughout, like point_cloud_io.
namespace bagwiz::core::slam
{

// Capacity of each point's observation reservoir: at most this many
// weighted observations are kept per map point; once full, a deterministic
// reservoir-sampling rule (see MapColorizer) decides which observations to
// replace, so the reduction in finish() stays bounded no matter how many
// images observed the point.
inline constexpr std::size_t kMaxObservations = 16;

// Geometry pre-pass over the map points, shared by every camera's
// MapColorizer (building the kd-tree is the expensive part and is camera
// independent). The kd-tree references the point array, which must outlive
// the geometry and must not be modified while it is in use.
struct ColorizeGeometry
{
  pointcloud::KdTree tree;
  std::vector<std::array<float, 3>> normals;  // unit normals; {0,0,0} = no normal
  std::vector<float> spacings;                // local point spacing per point [m]
};

// Builds the geometry over `points`: the kd-tree plus per-point normals and
// spacings from `k_neighbors`-neighbor PCA. Work is split over `num_threads`
// std::threads; the result is identical for any thread count.
[[nodiscard]] ColorizeGeometry build_colorize_geometry(
  std::span<const std::array<float, 3>> points, int k_neighbors, int num_threads);

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

  // Weight each observation by how trustworthy it is (depth distance,
  // surface incidence, image sharpness, image border) and drop observations
  // below weight_min. When false every surviving observation has weight 1.
  bool use_weights = true;

  // Depth [m] at which the distance weight saturates to 1:
  // w_dist = clamp((weight_distance_ref / z)^2, 0, 1).
  double weight_distance_ref = 15.0;

  // Half-saturation of the sharpness weight: w_sharp = g / (g + g0) where g
  // is the bilinear Sobel gradient magnitude (|gx| + |gy|) at the projected
  // pixel. <= 0 disables the sharpness factor (treated as 1).
  double weight_sharpness_g0 = 10.0;

  // Width [px] of the linear border falloff: the weight ramps from 0 at the
  // image edge to 1 this many pixels inside. <= 0 disables the falloff.
  double weight_border_margin_px = 16.0;

  // Observations with a total weight below this are dropped before they can
  // pollute a point's reservoir.
  double weight_min = 1e-3;

  // Estimate a per-image RGB gain from the points this image re-observes and
  // apply it before accumulation, tracking auto-exposure / white-balance
  // drift between frames. Only appearance-stable points vote (their ratio
  // carries the exposure change); mixed-lighting reservoirs abstain. The
  // gain only ever lifts an underexposed frame toward the established
  // reference (>= 1), never pulls a brighter frame down, so genuine
  // brightening cannot ratchet the stored colors toward black.
  bool gain_compensation = true;

  // Minimum number of re-observed points required to trust a gain estimate;
  // below this the image is accumulated with gain (1, 1, 1).
  std::size_t gain_min_samples = 256;

  // Minimum observations a point's reservoir must already hold for that
  // point to vote on the current image's gain.
  std::size_t gain_min_prior_obs = 4;

  // Neighbor count for the geometry pre-pass (normals + spacings).
  int geometry_neighbors = 12;

  // Occlusion backend configuration (range cull, depth tolerance, splat,
  // threads). The rasterizer owns per-image projection and visibility.
  ColorizeRasterizerConfig rasterizer;
};

// Result of MapColorizer::finish().
struct MapColorizeResult
{
  // Per-point {r, g, b}, parallel to the input points. Points never observed
  // by any accepted image keep a neutral gray so they stay visible without
  // reading as colored data.
  std::vector<std::array<std::uint8_t, 3>> colors;

  // Per-point observation flag, parallel to `colors`: 0 = unobserved,
  // 1 = observed by at least one accepted image. The value 2 is reserved for
  // colors propagated from neighbors and is only set downstream of the
  // colorizer (see pointcloud/color_propagation.hpp).
  std::vector<std::uint8_t> observed;

  // Per-point confidence, parallel to `colors`: the sum of the weights of
  // the observations that survived trimming. 0 for unobserved points.
  std::vector<float> weights;

  std::size_t colored_points = 0;  // points with at least one observation
  std::size_t images_used = 0;     // images accumulated
  std::size_t images_skipped = 0;  // images rejected (span/raster mismatch)
};

// Merge per-camera colorize results (`map slam --cam` given more than once).
// Cameras are first aligned in gain: for each result after the first, the
// per-channel median color ratio over the points observed by both that
// camera and the FIRST result (span order — alignment is always against the
// first result, never chained) scales its colors toward the first camera;
// fewer than 64 shared well-exposed samples leaves the camera unscaled.
// Then each point blends the cameras that observed it by their weights:
// color = sum(w_c * color_c) / sum(w_c). A point no camera observed keeps
// the neutral gray with observed = 0 and weight 0.
// `results` must be parallel (same point count; the first result's size is
// authoritative). images_used/images_skipped are summed across results. An
// empty span yields an empty result.
[[nodiscard]] MapColorizeResult merge_colorize_results(std::span<const MapColorizeResult> results);

class MapColorizer
{
public:
  // Single-camera convenience: builds the geometry pre-pass itself (using
  // config.geometry_neighbors and config.rasterizer.num_threads) and always
  // rasterizes on the CPU. `points` are world-frame map points and
  // `trajectory` the optimized body (cloud-frame) poses sorted ascending by
  // timestamp — exactly CloudMap's points/trajectory. Both spans must
  // outlive this object; neither is copied.
  MapColorizer(
    MapColorizerConfig config, std::span<const std::array<float, 3>> points,
    std::span<const core::TrajectoryPose> trajectory);

  // Multi-camera form: `geometry` is the shared pre-pass over the same
  // `points` (see ColorizeGeometry for the lifetime rule). A nullptr
  // `rasterizer` selects the CPU backend. The rasterizer parameter is the
  // injection seam: it exists so a future CUDA rasterizer can be injected
  // from the command layer without touching this class.
  MapColorizer(
    MapColorizerConfig config, std::shared_ptr<const ColorizeGeometry> geometry,
    std::span<const std::array<float, 3>> points, std::span<const core::TrajectoryPose> trajectory,
    std::unique_ptr<ColorizeRasterizer> rasterizer = nullptr);

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

  // Same as above, with the occluder geometry of the scene at the image's
  // own time: `dynamic_points` is the raw LiDAR scan nearest to the image,
  // in the same world frame as the map points. A map point well behind a
  // scan return — a vehicle that drove through the view but left nothing in
  // the accumulated map — is rejected for this image instead of sampling
  // the vehicle's pixels. Pass an empty span for the static-only behavior
  // of the four-argument form.
  bool add_image(
    std::int64_t stamp_ns, std::span<const std::byte> bgr, std::uint32_t width,
    std::uint32_t height, std::span<const std::array<float, 3>> dynamic_points);

  // Reduce every point's reservoir into the final colors: trim the
  // observations deviating too far from the 75th-luminance-percentile
  // observation (the lit-mode anchor; shadows are illumination, not surface
  // color), then a weighted mean over the survivors. See MapColorizeResult
  // for the output semantics.
  [[nodiscard]] MapColorizeResult finish() const;

private:
  // One reservoir page covering kPageSize consecutive points. Pages are
  // allocated lazily on a point's first observation: the observed set is
  // typically sparse, so a flat per-point array of reservoirs would cost
  // ~300 MB on multi-million-point maps that mostly never see a camera.
  static constexpr std::size_t kPageSize = 1024;
  struct ObservationPage
  {
    // Packed observations per point, most-significant byte first:
    // bits 31..24 = weight quantized to 8 bits linear (w * 255 rounded),
    // bits 23..16 = r, 15..8 = g, 7..0 = b.
    std::array<std::array<std::uint32_t, kMaxObservations>, kPageSize> slots{};
    // Total observations offered per point (grows past kMaxObservations;
    // the reservoir-sampling rule needs the full count).
    std::array<std::uint32_t, kPageSize> seen{};
  };

  // Adds one packed observation to a point's reservoir: stored directly
  // while fewer than kMaxObservations were seen, otherwise the (seen + 1)-th
  // observation replaces slot j when j = mix64(point_index, seen) %
  // (seen + 1) falls below kMaxObservations (Vitter-style reservoir
  // sampling). Each image contributes at most one observation per point and
  // images are processed serially, so a point's observation sequence — and
  // therefore the reservoir contents — is fixed regardless of thread count.
  void reservoir_add(std::uint32_t point_index, std::uint32_t packed);

  // What resolve_colorize_view produces for one accepted image: the
  // rasterized view (world->camera transform plus the rescaled intrinsics)
  // and the camera center in the world frame, which the incidence weight
  // reads its view directions from.
  struct ResolvedView
  {
    ColorizeView view;
    std::array<double, 3> cam_center;
  };

  // add_image's first phase: validate the image (non-empty map/trajectory,
  // matching raster size, stamp inside the trajectory span, interpolatable
  // pose), resolve the camera pose/intrinsics, and rasterize the map into
  // visible_scratch_. std::nullopt rejects the image; add_image counts it as
  // skipped.
  std::optional<ResolvedView> resolve_colorize_view(
    std::int64_t stamp_ns, std::span<const std::byte> bgr, std::uint32_t width,
    std::uint32_t height, std::span<const std::array<float, 3>> dynamic_points);

  // Second phase: weight and sample every visible point into
  // pending_scratch_ (per-chunk pending lists, merged in chunk order).
  void weight_and_sample(
    std::span<const std::byte> bgr, std::uint32_t width, std::uint32_t height,
    const std::array<double, 3> & cam_center);

  // Third phase (gain compensation, pass A): estimate this image's RGB gain
  // from the ratio of each re-observed point's reservoir mean to its new
  // observation. {1, 1, 1} when gain compensation is disabled or too few
  // points vote.
  std::array<double, 3> estimate_image_gain();

  // Fourth phase: ensure every pending point's reservoir page exists, then
  // apply the gain, quantize the weight, and reservoir-add (pass B).
  void reservoir_add_all(const std::array<double, 3> & gain);

  // Worker count for the per-image sweeps: the configured thread count
  // clamped to [1, max(1, visible count)] so an empty or tiny visible set
  // spawns no idle workers. The sweeps merge per-chunk results in chunk
  // order, keeping the result deterministic for any thread count.
  int num_sweep_threads() const;

  MapColorizerConfig config_;
  std::span<const std::array<float, 3>> points_;
  std::span<const core::TrajectoryPose> trajectory_;
  std::shared_ptr<const ColorizeGeometry> geometry_;
  std::unique_ptr<ColorizeRasterizer> rasterizer_;
  std::vector<std::unique_ptr<ObservationPage>> pages_;  // pages_[index / kPageSize]

  // Per-image scratch, kept between add_image calls to avoid reallocation.
  std::vector<VisiblePoint> visible_scratch_;
  struct PendingObservation
  {
    std::uint32_t index = 0;
    double r = 0.0;
    double g = 0.0;
    double b = 0.0;
    double weight = 0.0;
  };
  std::vector<PendingObservation> pending_scratch_;
  std::vector<std::vector<PendingObservation>> pending_chunks_;  // per worker, merged in order
  std::array<std::vector<double>, 3> gain_ratio_scratch_;
  std::vector<std::array<std::vector<double>, 3>> gain_ratio_chunks_;  // per worker

  std::size_t images_used_ = 0;
  std::size_t images_skipped_ = 0;
};

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__MAP_COLORIZER_HPP_
