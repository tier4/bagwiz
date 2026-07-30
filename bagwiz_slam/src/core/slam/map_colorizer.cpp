// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/map_colorizer.hpp"

#include "bagwiz/core/image/sampling.hpp"
#include "bagwiz/core/image/srgb.hpp"
#include "bagwiz/core/pointcloud/cloud_transform.hpp"
#include "bagwiz/core/pointcloud/normals.hpp"
#include "bagwiz/core/slam/colorize_weight.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <thread>
#include <utility>
#include <vector>

namespace bagwiz::core::slam
{

namespace
{

// Neutral gray for points no accepted image ever observed: visible in a
// viewer without reading as real color data.
constexpr std::array<std::uint8_t, 3> kUncoloredGray{128, 128, 128};

// Maximum per-channel deviation from the reservoir's anchor observation an
// observation may have and still survive trimming in finish() [gray levels].
// Dynamic objects and occlusion leaks land far from the anchor; honest
// observations of one surface cluster well inside this band.
constexpr double kTrimDeviation = 48.0;

// merge_colorize_results clamps its per-camera alignment to this range of
// linear-light ratios: wider swings mean the estimate locked onto something
// other than exposure drift. The bounds cover the reach of a factor-two swing
// in sRGB code values, which is a linear-light ratio of up to ~4.7 in the
// highlights (and less in the shadows).
constexpr double kGainMin = 0.2;
constexpr double kGainMax = 5.0;

// The per-image gain (also a linear-light ratio) is clamped to
// [1, kGainImageMax] rather than
// [kGainMin, kGainMax]: it may only LIFT an underexposed frame toward the
// established reference, never pull a brighter frame down. A scene that
// genuinely brightens (driving out of shade) would otherwise drag the
// reservoir's running mean below the true surface color, the below-1 gain
// would then chase that falling mean, and the estimate would ratchet itself
// down to the clamp floor. Lifting underexposed frames keeps the reservoir at
// the scene's best-lit exposure, which the lit-mode anchor in finish()
// prefers anyway.
constexpr double kGainImageMin = 1.0;
constexpr double kGainImageMax = 5.0;

// Maximum per-channel spread (max - min over the stored observations, gray
// levels) for a point's reservoir to vote on an image's gain. A reservoir
// that mixes lighting modes (sunlit and shadowed, or an occluder's color)
// has a dark-tilted mean; letting it vote would align every later bright
// observation toward that dark mode. Only appearance-stable points carry a
// trustworthy exposure ratio, so only they vote.
constexpr double kGainVoteStableRange = 32.0;

// A rigid transform with a row-major 3x3 rotation, p' = R * p + t. Same
// convention as pointcloud::RigidTransform (whose quat_to_rotation_matrix
// builds the rotation below), but kept local for the `compose` chaining this
// pass needs and so the header stays free of matrix types.
struct Rigid
{
  std::array<double, 9> r{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  std::array<double, 3> t{0.0, 0.0, 0.0};
};

Rigid compose(const Rigid & a, const Rigid & b)
{
  Rigid out;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      out.r[row * 3 + col] = a.r[row * 3 + 0] * b.r[0 * 3 + col] +
                             a.r[row * 3 + 1] * b.r[1 * 3 + col] +
                             a.r[row * 3 + 2] * b.r[2 * 3 + col];
    }
    out.t[row] =
      a.r[row * 3 + 0] * b.t[0] + a.r[row * 3 + 1] * b.t[1] + a.r[row * 3 + 2] * b.t[2] + a.t[row];
  }
  return out;
}

Rigid invert(const Rigid & a)
{
  Rigid out;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      out.r[row * 3 + col] = a.r[col * 3 + row];
    }
  }
  for (int row = 0; row < 3; ++row) {
    out.t[row] =
      -(out.r[row * 3 + 0] * a.t[0] + out.r[row * 3 + 1] * a.t[1] + out.r[row * 3 + 2] * a.t[2]);
  }
  return out;
}

// splitmix64-style bit mixer over (point_index, seen). Drives the reservoir
// replacement slot; the only requirement is determinism — identical inputs
// must give identical outputs on every platform and thread count.
std::uint64_t mix64(std::uint64_t point_index, std::uint64_t seen)
{
  std::uint64_t z = point_index * 0x9E3779B97F4A7C15ULL + seen * 0xC2B2AE3D27D4EB4FULL;
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
  return z ^ (z >> 31);
}

// Median of `values` (average of the two middle values for an even count).
// Sorts the caller's scratch buffer in place.
double median_of(std::vector<double> & values)
{
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const std::size_t mid = values.size() / 2;
  if (values.size() % 2 == 1) {
    return values[mid];
  }
  return 0.5 * (values[mid - 1] + values[mid]);
}

// Round-to-nearest into [0, 255], for weight quantization (colors go through
// image::linear_to_srgb_u8 instead, which quantizes and gamma-encodes in one
// step).
std::uint32_t quantize8(double value)
{
  return static_cast<std::uint32_t>(std::clamp(value, 0.0, 255.0) + 0.5);
}

std::shared_ptr<const ColorizeGeometry> make_owned_geometry(
  std::span<const std::array<float, 3>> points, const MapColorizerConfig & config)
{
  return std::make_shared<const ColorizeGeometry>(
    build_colorize_geometry(points, config.geometry_neighbors, config.rasterizer.num_threads));
}

// Runs `fn(begin, end, chunk_index)` over `count` items split into
// num_threads contiguous chunks, one chunk per worker thread; serial when
// num_threads <= 1 or count == 0. Deterministic for any thread count when
// per-chunk results are merged in chunk order.
template <typename Fn>
void run_parallel(int num_threads, std::size_t count, Fn && fn)
{
  if (num_threads <= 1 || count == 0) {
    fn(0, count, 0);
    return;
  }
  std::vector<std::thread> workers;
  workers.reserve(static_cast<std::size_t>(num_threads));
  const std::size_t chunk =
    (count + static_cast<std::size_t>(num_threads) - 1) / static_cast<std::size_t>(num_threads);
  for (int t = 0; t < num_threads; ++t) {
    const std::size_t begin = std::min(count, static_cast<std::size_t>(t) * chunk);
    const std::size_t end = std::min(count, begin + chunk);
    workers.emplace_back([&, begin, end, t]() { fn(begin, end, static_cast<std::size_t>(t)); });
  }
  for (auto & w : workers) {
    w.join();
  }
}

}  // namespace

ColorizeGeometry build_colorize_geometry(
  std::span<const std::array<float, 3>> points, int k_neighbors, int num_threads)
{
  ColorizeGeometry geometry;
  geometry.tree = pointcloud::KdTree(points);
  auto local = pointcloud::estimate_local_geometry(points, geometry.tree, k_neighbors, num_threads);
  geometry.normals = std::move(local.normals);
  geometry.spacings = std::move(local.spacings);
  return geometry;
}

MapColorizer::MapColorizer(
  MapColorizerConfig config, std::span<const std::array<float, 3>> points,
  std::span<const core::TrajectoryPose> trajectory)
: MapColorizer(config, make_owned_geometry(points, config), points, trajectory, nullptr)
{
}

MapColorizer::MapColorizer(
  MapColorizerConfig config, std::shared_ptr<const ColorizeGeometry> geometry,
  std::span<const std::array<float, 3>> points, std::span<const core::TrajectoryPose> trajectory,
  std::unique_ptr<ColorizeRasterizer> rasterizer)
: config_(std::move(config)),
  points_(points),
  trajectory_(trajectory),
  geometry_(std::move(geometry)),
  rasterizer_(std::move(rasterizer)),
  pages_((points.size() + kPageSize - 1) / kPageSize)
{
  if (!rasterizer_) {
    rasterizer_ = make_cpu_colorize_rasterizer(
      points, geometry_ ? std::span<const float>(geometry_->spacings) : std::span<const float>{},
      geometry_ ? std::span<const std::array<float, 3>>(geometry_->normals)
                : std::span<const std::array<float, 3>>{},
      config_.rasterizer, geometry_ ? &geometry_->tree : nullptr);
  }
}

void MapColorizer::reservoir_add(std::uint32_t point_index, std::uint32_t packed)
{
  auto & page = pages_[point_index / kPageSize];
  if (!page) {
    page = std::make_unique<ObservationPage>();
  }
  const std::size_t offset = point_index % kPageSize;
  const std::uint32_t seen = page->seen[offset];
  if (seen < kMaxObservations) {
    page->slots[offset][seen] = packed;
  } else {
    const std::uint64_t j = mix64(point_index, seen) % (static_cast<std::uint64_t>(seen) + 1ULL);
    if (j < kMaxObservations) {
      page->slots[offset][static_cast<std::size_t>(j)] = packed;
    }
  }
  page->seen[offset] = seen + 1;
}

bool MapColorizer::add_image(
  std::int64_t stamp_ns, std::span<const std::byte> bgr, std::uint32_t width, std::uint32_t height)
{
  return add_image(stamp_ns, bgr, width, height, {});
}

std::optional<MapColorizer::ResolvedView> MapColorizer::resolve_colorize_view(
  std::int64_t stamp_ns, std::span<const std::byte> bgr, std::uint32_t width, std::uint32_t height,
  std::span<const std::array<float, 3>> dynamic_points)
{
  if (points_.empty() || trajectory_.empty()) {
    return std::nullopt;
  }
  if (width == 0 || height == 0 || bgr.size() != static_cast<std::size_t>(width) * 3U * height) {
    return std::nullopt;
  }
  // Reject stamps outside the trajectory span: lookup_pose would clamp to an
  // endpoint pose, projecting the map from a viewpoint the platform never had
  // at that time and smearing wrong colors over it.
  if (stamp_ns < trajectory_.front().timestamp_ns || stamp_ns > trajectory_.back().timestamp_ns) {
    return std::nullopt;
  }
  const auto pose = core::lookup_pose(stamp_ns, trajectory_);
  if (!pose) {
    return std::nullopt;
  }

  // Camera pose in the world: T_world_cam = T_world_cloud * T_cloud_cam, then
  // inverted once so each point costs one rotate+translate.
  Rigid t_world_cloud;
  t_world_cloud.r = pointcloud::quat_to_rotation_matrix(pose->qx, pose->qy, pose->qz, pose->qw);
  t_world_cloud.t = {pose->tx, pose->ty, pose->tz};
  Rigid t_cloud_cam;
  t_cloud_cam.r = pointcloud::quat_to_rotation_matrix(
    config_.t_cloud_cam.rotation_xyzw[0], config_.t_cloud_cam.rotation_xyzw[1],
    config_.t_cloud_cam.rotation_xyzw[2], config_.t_cloud_cam.rotation_xyzw[3]);
  t_cloud_cam.t = config_.t_cloud_cam.translation;
  const Rigid t_cam_world = invert(compose(t_world_cloud, t_cloud_cam));

  ResolvedView resolved;
  // Camera center in the world, -R^T t of the world->camera transform, for
  // the incidence view directions.
  resolved.cam_center = {
    -(t_cam_world.r[0] * t_cam_world.t[0] + t_cam_world.r[3] * t_cam_world.t[1] +
      t_cam_world.r[6] * t_cam_world.t[2]),
    -(t_cam_world.r[1] * t_cam_world.t[0] + t_cam_world.r[4] * t_cam_world.t[1] +
      t_cam_world.r[7] * t_cam_world.t[2]),
    -(t_cam_world.r[2] * t_cam_world.t[0] + t_cam_world.r[5] * t_cam_world.t[1] +
      t_cam_world.r[8] * t_cam_world.t[2])};

  // Rescale the intrinsics when the delivered image differs from the
  // calibrated resolution (e.g. a downscaled republished stream).
  resolved.view.camera = config_.camera;
  if (
    resolved.view.camera.width != 0 && resolved.view.camera.height != 0 &&
    (resolved.view.camera.width != width || resolved.view.camera.height != height)) {
    resolved.view.camera = image::scale_camera_info(
      resolved.view.camera, static_cast<double>(width) / resolved.view.camera.width,
      static_cast<double>(height) / resolved.view.camera.height);
  }
  resolved.view.r_cam_world = t_cam_world.r;
  resolved.view.t_cam_world = t_cam_world.t;
  resolved.view.width = width;
  resolved.view.height = height;
  rasterizer_->visible_points(resolved.view, dynamic_points, visible_scratch_);
  return resolved;
}

int MapColorizer::num_sweep_threads() const
{
  return std::clamp(
    config_.rasterizer.num_threads, 1,
    static_cast<int>(std::max<std::size_t>(1, visible_scratch_.size())));
}

void MapColorizer::weight_and_sample(
  std::span<const std::byte> bgr, std::uint32_t width, std::uint32_t height,
  const std::array<double, 3> & cam_center)
{
  const ObservationWeightParams weight_params{
    config_.weight_distance_ref, config_.weight_sharpness_g0, config_.weight_border_margin_px};
  const std::span<const std::array<float, 3>> normals =
    geometry_ ? std::span<const std::array<float, 3>>(geometry_->normals)
              : std::span<const std::array<float, 3>>{};
  const int num_threads = num_sweep_threads();
  if (pending_chunks_.size() != static_cast<std::size_t>(num_threads)) {
    pending_chunks_.assign(static_cast<std::size_t>(num_threads), {});
  }

  // Weight and sample every visible point into per-chunk pending lists.
  auto weight_chunk = [&](std::size_t begin, std::size_t end, std::size_t chunk) {
    auto & out = pending_chunks_[chunk];
    out.clear();
    for (std::size_t j = begin; j < end; ++j) {
      const auto & vp = visible_scratch_[j];
      double weight = 1.0;
      if (config_.use_weights) {
        weight = compute_observation_weight(
          vp, points_, normals, cam_center, bgr, width, height, weight_params);
        if (weight < config_.weight_min) {
          continue;
        }
      }
      // Linear-light sample: color arithmetic downstream (gain ratios, the
      // reservoir means) is only meaningful on the radiance underneath the
      // gamma-encoded pixel values.
      const auto sample = image::bilinear_sample_bgr_linear(bgr, width, height, vp.u, vp.v);
      out.push_back(PendingObservation{vp.index, sample[2], sample[1], sample[0], weight});
    }
  };
  run_parallel(num_threads, visible_scratch_.size(), weight_chunk);
  pending_scratch_.clear();
  for (const auto & chunk : pending_chunks_) {
    pending_scratch_.insert(pending_scratch_.end(), chunk.begin(), chunk.end());
  }
}

std::array<double, 3> MapColorizer::estimate_image_gain()
{
  // Gain compensation, pass A: estimate this image's RGB gain from the ratio
  // of each re-observed point's reservoir mean to its new observation. Votes
  // accumulate per chunk and merge below; the median is order-independent.
  std::array<double, 3> gain{1.0, 1.0, 1.0};
  if (!config_.gain_compensation) {
    return gain;
  }
  const int num_threads = num_sweep_threads();
  if (gain_ratio_chunks_.size() != static_cast<std::size_t>(num_threads)) {
    gain_ratio_chunks_.assign(static_cast<std::size_t>(num_threads), {});
  }
  auto vote_chunk = [&](std::size_t begin, std::size_t end, std::size_t chunk) {
    auto & ratios = gain_ratio_chunks_[chunk];
    for (auto & r : ratios) {
      r.clear();
    }
    for (std::size_t j = begin; j < end; ++j) {
      const auto & obs = pending_scratch_[j];
      const auto & page = pages_[obs.index / kPageSize];
      if (!page) {
        continue;
      }
      const std::size_t offset = obs.index % kPageSize;
      const std::size_t stored = std::min<std::size_t>(page->seen[offset], kMaxObservations);
      if (stored < config_.gain_min_prior_obs) {
        continue;
      }
      std::array<double, 3> mean{0.0, 0.0, 0.0};        // linear light
      std::array<double, 3> cmin{255.0, 255.0, 255.0};  // sRGB code values
      std::array<double, 3> cmax{0.0, 0.0, 0.0};
      for (std::size_t s = 0; s < stored; ++s) {
        const std::uint32_t packed = page->slots[offset][s];
        const std::array<std::uint8_t, 3> value{
          static_cast<std::uint8_t>((packed >> 16) & 0xFFU),
          static_cast<std::uint8_t>((packed >> 8) & 0xFFU),
          static_cast<std::uint8_t>(packed & 0xFFU)};
        for (std::size_t c = 0; c < 3; ++c) {
          mean[c] += image::srgb_u8_to_linear(value[c]);
          cmin[c] = std::min(cmin[c], static_cast<double>(value[c]));
          cmax[c] = std::max(cmax[c], static_cast<double>(value[c]));
        }
      }
      const std::array<double, 3> current{obs.r, obs.g, obs.b};  // linear light
      // The exposure ratio is linear light (a uniform exposure change scales
      // every point's radiance by the same factor there, which no single
      // factor does on gamma-encoded values). The appearance-stability window
      // stays on the stored sRGB code values: a fixed gray-level band is only
      // meaningful in the perceptually allocated encoding.
      const double black_floor = image::srgb_u8_to_linear(8);
      bool usable = true;
      for (std::size_t c = 0; c < 3; ++c) {
        mean[c] /= static_cast<double>(stored);
        // Appearance-unstable reservoirs abstain (see kGainVoteStableRange);
        // near-black channels make the ratio numerically meaningless.
        usable = usable && cmax[c] - cmin[c] <= kGainVoteStableRange && mean[c] >= black_floor &&
                 current[c] >= black_floor;
      }
      if (!usable) {
        continue;
      }
      for (std::size_t c = 0; c < 3; ++c) {
        ratios[c].push_back(mean[c] / current[c]);
      }
    }
  };
  run_parallel(num_threads, pending_scratch_.size(), vote_chunk);
  for (auto & ratios : gain_ratio_scratch_) {
    ratios.clear();
  }
  for (const auto & chunk : gain_ratio_chunks_) {
    for (std::size_t c = 0; c < 3; ++c) {
      gain_ratio_scratch_[c].insert(gain_ratio_scratch_[c].end(), chunk[c].begin(), chunk[c].end());
    }
  }
  if (gain_ratio_scratch_[0].size() >= config_.gain_min_samples) {
    for (std::size_t c = 0; c < 3; ++c) {
      gain[c] = std::clamp(median_of(gain_ratio_scratch_[c]), kGainImageMin, kGainImageMax);
    }
  }
  return gain;
}

void MapColorizer::reservoir_add_all(const std::array<double, 3> & gain)
{
  // Reservoir pages are allocated lazily on a point's first observation;
  // the parallel pass below must not race the allocation, so ensure every
  // pending point's page exists first (pointer checks only, cheap).
  for (const auto & obs : pending_scratch_) {
    auto & page = pages_[obs.index / kPageSize];
    if (!page) {
      page = std::make_unique<ObservationPage>();
    }
  }

  // Pass B: apply the gain in linear light, re-encode to sRGB for storage,
  // quantize the weight, and reservoir-add. Points are unique within an
  // image, so per-point writes never collide.
  auto add_chunk = [&](std::size_t begin, std::size_t end, std::size_t /*chunk*/) {
    for (std::size_t j = begin; j < end; ++j) {
      const auto & obs = pending_scratch_[j];
      const std::uint32_t r = image::linear_to_srgb_u8(obs.r * gain[0]);
      const std::uint32_t g = image::linear_to_srgb_u8(obs.g * gain[1]);
      const std::uint32_t b = image::linear_to_srgb_u8(obs.b * gain[2]);
      const std::uint32_t w_q = quantize8(obs.weight * 255.0);
      reservoir_add(obs.index, (w_q << 24) | (r << 16) | (g << 8) | b);
    }
  };
  run_parallel(num_sweep_threads(), pending_scratch_.size(), add_chunk);
}

bool MapColorizer::add_image(
  std::int64_t stamp_ns, std::span<const std::byte> bgr, std::uint32_t width, std::uint32_t height,
  std::span<const std::array<float, 3>> dynamic_points)
{
  const auto resolved = resolve_colorize_view(stamp_ns, bgr, width, height, dynamic_points);
  if (!resolved) {
    ++images_skipped_;
    return false;
  }
  // The per-image sweeps are independent per point, so each runs over
  // num_sweep_threads() chunks (merged in chunk order, keeping the result
  // deterministic for any thread count): weight + sample, gain vote, then
  // gain-apply + reservoir-add.
  weight_and_sample(bgr, width, height, resolved->cam_center);
  const std::array<double, 3> gain = estimate_image_gain();
  reservoir_add_all(gain);
  ++images_used_;
  return true;
}

MapColorizeResult MapColorizer::finish() const
{
  MapColorizeResult result;
  result.colors.assign(points_.size(), kUncoloredGray);
  result.observed.assign(points_.size(), 0);
  result.weights.assign(points_.size(), 0.0F);
  for (std::size_t i = 0; i < points_.size(); ++i) {
    const auto & page = pages_[i / kPageSize];
    if (!page) {
      continue;
    }
    const std::size_t offset = i % kPageSize;
    const std::size_t stored = std::min<std::size_t>(page->seen[offset], kMaxObservations);
    if (stored == 0) {
      continue;
    }
    // Decode the kept slots: the sRGB code values feed the luminance anchor
    // and the trim (fixed gray-level bands are only meaningful in the
    // perceptually allocated encoding), the linear-light values feed the mean.
    std::array<double, kMaxObservations> ws;
    std::array<std::array<double, kMaxObservations>, 3> channels;
    std::array<std::array<double, kMaxObservations>, 3> linear;
    for (std::size_t s = 0; s < stored; ++s) {
      const std::uint32_t packed = page->slots[offset][s];
      ws[s] = static_cast<double>(packed >> 24) / 255.0;
      const std::array<std::uint8_t, 3> value{
        static_cast<std::uint8_t>((packed >> 16) & 0xFFU),
        static_cast<std::uint8_t>((packed >> 8) & 0xFFU),
        static_cast<std::uint8_t>(packed & 0xFFU)};
      for (std::size_t c = 0; c < 3; ++c) {
        channels[c][s] = static_cast<double>(value[c]);
        linear[c][s] = image::srgb_u8_to_linear(value[c]);
      }
    }
    // Per-observation luminance and the index order sorting by it.
    std::array<double, kMaxObservations> lum;
    std::array<std::size_t, kMaxObservations> order;
    for (std::size_t s = 0; s < stored; ++s) {
      lum[s] = 0.299 * channels[0][s] + 0.587 * channels[1][s] + 0.114 * channels[2][s];
      order[s] = s;
    }
    std::sort(
      order.begin(), order.begin() + static_cast<std::ptrdiff_t>(stored),
      [&](std::size_t a, std::size_t b) { return lum[a] < lum[b]; });
    // Anchor the trim at the 75th luminance percentile, not the median:
    // shadows are illumination, not surface color, and occluders never reach
    // a point's reservoir (the depth test rejects them while they occlude),
    // so the lit-mode cluster is the honest estimate of the surface's
    // appearance. One whole observation anchors all three channels, which
    // keeps tinted light (e.g. blue shadow) from mixing channels across
    // lighting modes.
    const std::size_t anchor = order[(3 * stored + 3) / 4 - 1];
    const std::array<double, 3> anchor_color{
      channels[0][anchor], channels[1][anchor], channels[2][anchor]};
    // Trim observations too far from the anchor (dynamic objects, occlusion
    // leaks). If every observation trims — e.g. two complementary colors
    // straddling the anchor — fall back to the untrimmed set: the data is
    // genuinely multimodal and any mean is as defensible as another.
    std::uint32_t kept_mask = 0;
    for (std::size_t s = 0; s < stored; ++s) {
      const double dev = std::max(
        std::abs(channels[0][s] - anchor_color[0]),
        std::max(
          std::abs(channels[1][s] - anchor_color[1]), std::abs(channels[2][s] - anchor_color[2])));
      if (dev <= kTrimDeviation) {
        kept_mask |= 1U << s;
      }
    }
    if (kept_mask == 0) {
      kept_mask = (1U << stored) - 1U;
    }
    std::array<double, 3> sum{0.0, 0.0, 0.0};
    std::array<double, 3> plain_sum{0.0, 0.0, 0.0};
    double weight_sum = 0.0;
    std::size_t kept = 0;
    for (std::size_t s = 0; s < stored; ++s) {
      if ((kept_mask & (1U << s)) == 0) {
        continue;
      }
      for (std::size_t c = 0; c < 3; ++c) {
        // The mean runs in linear light: averaging gamma-encoded values would
        // land systematically darker (the sRGB encode is concave).
        sum[c] += ws[s] * linear[c][s];
        plain_sum[c] += linear[c][s];
      }
      weight_sum += ws[s];
      ++kept;
    }
    std::array<double, 3> color;
    if (weight_sum > 0.0) {
      for (std::size_t c = 0; c < 3; ++c) {
        color[c] = sum[c] / weight_sum;
      }
    } else {
      // Every kept observation quantized to zero weight; fall back to a
      // plain mean rather than divide by zero.
      for (std::size_t c = 0; c < 3; ++c) {
        color[c] = plain_sum[c] / static_cast<double>(kept);
      }
    }
    result.colors[i] = {
      image::linear_to_srgb_u8(color[0]), image::linear_to_srgb_u8(color[1]),
      image::linear_to_srgb_u8(color[2])};
    result.observed[i] = 1;
    result.weights[i] = static_cast<float>(weight_sum);
    ++result.colored_points;
  }
  result.images_used = images_used_;
  result.images_skipped = images_skipped_;
  return result;
}

MapColorizeResult merge_colorize_results(std::span<const MapColorizeResult> results)
{
  MapColorizeResult merged;
  if (results.empty()) {
    return merged;
  }
  const std::size_t count = results.front().colors.size();
  merged.colors.assign(count, kUncoloredGray);
  merged.observed.assign(count, 0);
  merged.weights.assign(count, 0.0F);
  for (const auto & result : results) {
    merged.images_used += result.images_used;
    merged.images_skipped += result.images_skipped;
  }

  // Shared well-exposed samples below which a camera's gain alignment is not
  // trusted (the median ratio needs a real sample base to be meaningful).
  constexpr std::size_t kMinAlignmentSamples = 64;

  // Per-camera gain alignment against the first result, so a camera that ran
  // a different exposure or white balance does not drag the blend. Camera 0
  // is the reference by definition; aligned[cam] views the original colors
  // when no scaling applies, avoiding copies.
  std::vector<std::span<const std::array<std::uint8_t, 3>>> aligned(results.size());
  std::vector<std::vector<std::array<std::uint8_t, 3>>> aligned_storage(results.size());
  for (std::size_t cam = 0; cam < results.size(); ++cam) {
    aligned[cam] = results[cam].colors;
    if (cam == 0) {
      continue;
    }
    const auto & reference = results[0];
    const auto & current = results[cam];
    const std::size_t shared = std::min(
      {reference.colors.size(), current.colors.size(), reference.observed.size(),
       current.observed.size(), reference.weights.size(), current.weights.size()});
    std::array<std::vector<double>, 3> ratios;
    for (std::size_t i = 0; i < shared; ++i) {
      if (reference.observed[i] != 1 || current.observed[i] != 1) {
        continue;
      }
      if (reference.weights[i] <= 0.0F || current.weights[i] <= 0.0F) {
        continue;
      }
      bool usable = true;
      for (std::size_t c = 0; c < 3; ++c) {
        usable = usable && reference.colors[i][c] >= 8 && current.colors[i][c] >= 8;
      }
      if (!usable) {
        continue;
      }
      for (std::size_t c = 0; c < 3; ++c) {
        // Exposure alignment is a linear-light ratio, like the per-image gain.
        ratios[c].push_back(
          image::srgb_u8_to_linear(reference.colors[i][c]) /
          image::srgb_u8_to_linear(current.colors[i][c]));
      }
    }
    if (ratios[0].size() < kMinAlignmentSamples) {
      continue;
    }
    std::array<double, 3> gain;
    for (std::size_t c = 0; c < 3; ++c) {
      gain[c] = std::clamp(median_of(ratios[c]), kGainMin, kGainMax);
    }
    auto & copy = aligned_storage[cam];
    copy = current.colors;
    for (auto & color : copy) {
      for (std::size_t c = 0; c < 3; ++c) {
        color[c] = image::linear_to_srgb_u8(image::srgb_u8_to_linear(color[c]) * gain[c]);
      }
    }
    aligned[cam] = copy;
  }

  // Weighted blend across cameras per point.
  for (std::size_t i = 0; i < count; ++i) {
    std::array<double, 3> sum{0.0, 0.0, 0.0};
    double weight_sum = 0.0;
    for (std::size_t cam = 0; cam < results.size(); ++cam) {
      if (i >= results[cam].observed.size() || results[cam].observed[i] != 1) {
        continue;
      }
      if (i >= results[cam].weights.size() || i >= aligned[cam].size()) {
        continue;
      }
      const double w = results[cam].weights[i];
      if (w <= 0.0) {
        continue;
      }
      for (std::size_t c = 0; c < 3; ++c) {
        // Like finish(), the cross-camera blend averages in linear light.
        sum[c] += w * image::srgb_u8_to_linear(aligned[cam][i][c]);
      }
      weight_sum += w;
    }
    if (weight_sum <= 0.0) {
      continue;
    }
    merged.colors[i] = {
      image::linear_to_srgb_u8(sum[0] / weight_sum), image::linear_to_srgb_u8(sum[1] / weight_sum),
      image::linear_to_srgb_u8(sum[2] / weight_sum)};
    merged.observed[i] = 1;
    merged.weights[i] = static_cast<float>(weight_sum);
    ++merged.colored_points;
  }
  return merged;
}

}  // namespace bagwiz::core::slam
