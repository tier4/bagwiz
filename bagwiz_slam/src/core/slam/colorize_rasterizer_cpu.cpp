// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/image/camera_distortion.hpp"
#include "bagwiz/core/slam/colorize_rasterizer.hpp"

#include <algorithm>
#include <atomic>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

namespace bagwiz::core::slam
{

namespace
{

// Bit pattern of +infinity, the depth buffer's "no write yet" sentinel.
// Non-negative IEEE-754 floats are bit-monotone (for a, b >= 0, a < b iff
// bits(a) < bits(b)), so racing splat writes can combine as an atomic min on
// the bit patterns and still yield the exact float minimum, independent of
// thread count or scheduling.
constexpr std::uint32_t kInfinityBits = 0x7F800000U;

class CpuColorizeRasterizer : public ColorizeRasterizer
{
public:
  CpuColorizeRasterizer(
    std::span<const std::array<float, 3>> points, std::span<const float> spacings,
    const ColorizeRasterizerConfig & config, const pointcloud::KdTree * tree)
  : points_(points), spacings_(spacings), config_(config), tree_(tree)
  {
  }

  void visible_points(
    const ColorizeView & view, std::span<const std::array<float, 3>> dynamic_points,
    std::vector<VisiblePoint> & out) override
  {
    out.clear();
    if (points_.empty() || view.width == 0 || view.height == 0) {
      return;
    }

    // Persistent depth buffers: reallocated only when the view size changes,
    // refilled to the +inf sentinel every call. The dynamic buffer is built
    // only when a scan was supplied for this image.
    if (view.width != depth_width_ || view.height != depth_height_) {
      depth_width_ = view.width;
      depth_height_ = view.height;
      depth_buffer_.assign(static_cast<std::size_t>(view.width) * view.height, kInfinityBits);
      dynamic_buffer_.assign(static_cast<std::size_t>(view.width) * view.height, kInfinityBits);
    } else {
      std::fill(depth_buffer_.begin(), depth_buffer_.end(), kInfinityBits);
      std::fill(dynamic_buffer_.begin(), dynamic_buffer_.end(), kInfinityBits);
    }

    const image::CameraInfo & cam = view.camera;
    const double fx = cam.k[0];
    const double fy = cam.k[4];
    const double cx = cam.k[2];
    const double cy = cam.k[5];
    const bool apply_distortion = !cam.d.empty();
    const image::DistortionModel distortion_model =
      apply_distortion ? image::select_distortion_model(cam.distortion_model)
                       : image::DistortionModel::kNone;
    const double max_range_sq = config_.max_range > 0.0 ? config_.max_range * config_.max_range
                                                        : std::numeric_limits<double>::infinity();
    const double f_avg = 0.5 * (fx + fy);
    const bool splat = config_.splat && spacings_.size() == points_.size();

    // Spatial cull: with a kd-tree attached and a positive max_range, sweep
    // only the points within range of this view's camera position. The
    // visible set is identical to a full sweep (the projection's own range
    // check rejects the rest); it just costs one radius search instead of a
    // whole-map projection. The query returns indices in tree order (not
    // sorted), which changes no result: every downstream accumulation is
    // per-point.
    std::size_t sweep_count = points_.size();
    const std::uint32_t * sweep_indices = nullptr;  // null -> sweep everything
    if (tree_ != nullptr && tree_->size() == points_.size() && config_.max_range > 0.0) {
      // The camera center in the world frame is -R^T t of the world->camera
      // rigid transform.
      const std::array<float, 3> cam_center{
        static_cast<float>(
          -(view.r_cam_world[0] * view.t_cam_world[0] + view.r_cam_world[3] * view.t_cam_world[1] +
            view.r_cam_world[6] * view.t_cam_world[2])),
        static_cast<float>(
          -(view.r_cam_world[1] * view.t_cam_world[0] + view.r_cam_world[4] * view.t_cam_world[1] +
            view.r_cam_world[7] * view.t_cam_world[2])),
        static_cast<float>(
          -(view.r_cam_world[2] * view.t_cam_world[0] + view.r_cam_world[5] * view.t_cam_world[1] +
            view.r_cam_world[8] * view.t_cam_world[2]))};
      tree_->radius_search(
        cam_center, static_cast<float>(config_.max_range), sweep_indices_scratch_);
      sweep_count = sweep_indices_scratch_.size();
      sweep_indices = sweep_indices_scratch_.data();
    }

    const int num_threads =
      std::clamp(config_.num_threads, 1, static_cast<int>(std::max<std::size_t>(1, sweep_count)));
    // Per-chunk candidate storage, kept between calls to limit allocations;
    // each chunk's list is touched by exactly one worker at a time.
    if (chunk_candidates_.size() != static_cast<std::size_t>(num_threads)) {
      chunk_candidates_.assign(static_cast<std::size_t>(num_threads), {});
    }

    // Projects one world-frame point into the view, returning {u, v, depth};
    // nullopt when behind the camera, beyond max_range, outside the lens
    // model's domain (fold-back), or out of the image.
    auto project = [&](const std::array<float, 3> & p) -> std::optional<std::array<double, 3>> {
      const double x = view.r_cam_world[0] * p[0] + view.r_cam_world[1] * p[1] +
                       view.r_cam_world[2] * p[2] + view.t_cam_world[0];
      const double y = view.r_cam_world[3] * p[0] + view.r_cam_world[4] * p[1] +
                       view.r_cam_world[5] * p[2] + view.t_cam_world[1];
      const double z = view.r_cam_world[6] * p[0] + view.r_cam_world[7] * p[1] +
                       view.r_cam_world[8] * p[2] + view.t_cam_world[2];
      if (z <= 0.0) {
        return std::nullopt;
      }
      if (x * x + y * y + z * z > max_range_sq) {
        return std::nullopt;
      }
      const double nx = x / z;
      const double ny = y / z;
      if (apply_distortion) {
        const auto distorted = image::distort_normalized(nx, ny, distortion_model, cam.d);
        const double u = fx * distorted.x + cx;
        const double v = fy * distorted.y + cy;
        if (u < 0.0 || u >= view.width || v < 0.0 || v >= view.height) {
          return std::nullopt;
        }
        // The fold-back round trip costs several times the forward distortion,
        // so it runs only for points landing inside the image — the only place
        // a folded ray can be mistaken for a real projection. Out-of-image
        // points are rejected either way, so the visible set is unchanged.
        if (image::distortion_round_trip_fails(
              nx, ny, distorted, distortion_model, cam.d, fx, fy)) {
          return std::nullopt;
        }
        return std::array<double, 3>{u, v, z};
      }
      const double u = fx * nx + cx;
      const double v = fy * ny + cy;
      if (u < 0.0 || u >= view.width || v < 0.0 || v >= view.height) {
        return std::nullopt;
      }
      return std::array<double, 3>{u, v, z};
    };

    // Pass 1 (parallel over chunks of the sweep set): transform + project
    // every swept map point, splat its depth into the buffer, and stash the
    // in-bounds candidates for pass 2. Chunks index the sweep list (identity
    // when unculled, the kd-tree query result when culled).
    auto project_chunk =
      [&](std::size_t begin, std::size_t end, std::vector<VisiblePoint> & candidates) {
        candidates.clear();
        for (std::size_t j = begin; j < end; ++j) {
          const std::uint32_t i =
            sweep_indices != nullptr ? sweep_indices[j] : static_cast<std::uint32_t>(j);
          const auto projected = project(points_[i]);
          if (!projected) {
            continue;
          }
          const double u = (*projected)[0];
          const double v = (*projected)[1];
          const float depth = static_cast<float>((*projected)[2]);
          candidates.push_back(VisiblePoint{i, u, v, depth});
          const double radius_px =
            splat ? std::clamp(
                      f_avg * static_cast<double>(spacings_[i]) / (2.0 * (*projected)[2]), 0.0,
                      config_.splat_radius_max_px)
                  : 0.0;
          splat_depth(depth_buffer_, u, v, radius_px, std::bit_cast<std::uint32_t>(depth));
        }
      };
    run_chunked(num_threads, sweep_count, project_chunk);

    // Pass 1b (parallel over scan chunks): splat the dynamic occluders into
    // their own depth buffer, each with the depth-dependent disc radius that
    // closes the gaps between returns (see config_.dynamic_splat_spacing).
    // No candidates are kept — the scan only gates map points in pass 2, it
    // is never colored itself.
    if (!dynamic_points.empty()) {
      auto splat_dynamic = [&](std::size_t begin, std::size_t end) {
        for (std::size_t i = begin; i < end; ++i) {
          const auto projected = project(dynamic_points[i]);
          if (!projected) {
            continue;
          }
          const double radius_px = std::clamp(
            f_avg * config_.dynamic_splat_spacing / (*projected)[2],
            config_.dynamic_splat_radius_min_px, config_.dynamic_splat_radius_max_px);
          splat_depth(
            dynamic_buffer_, (*projected)[0], (*projected)[1], radius_px,
            std::bit_cast<std::uint32_t>(static_cast<float>((*projected)[2])));
        }
      };
      run_chunked(num_threads, dynamic_points.size(), splat_dynamic);
    }

    // Pass 2 (parallel over the same chunks): visibility per candidate. Where
    // the scan covers the pixel it is the visibility ORACLE — it measured
    // the scene at the image's own time, so a candidate well behind its
    // return (a vehicle that left nothing in the accumulated map) is
    // rejected, and a candidate matching its return is accepted even when
    // stale dynamic geometry in the map would block it. Where the scan has
    // no return, the static map depth decides. Filtering in place preserves
    // the chunk's point-index order.
    const double rel = config_.depth_rel_tolerance;
    const double abs = config_.depth_abs_tolerance;
    const double drel = config_.dynamic_rel_tolerance;
    const double dabs = config_.dynamic_abs_tolerance;
    const bool has_dynamic = !dynamic_points.empty();
    const std::uint32_t width = depth_width_;
    auto filter_chunk = [&](std::vector<VisiblePoint> & candidates) {
      std::size_t kept = 0;
      for (const auto & c : candidates) {
        const std::size_t pixel =
          static_cast<std::size_t>(static_cast<std::uint32_t>(c.v)) * width +
          static_cast<std::uint32_t>(c.u);
        const std::uint32_t dyn_bits = has_dynamic ? dynamic_buffer_[pixel] : kInfinityBits;
        if (dyn_bits != kInfinityBits) {
          const double dzbuf = static_cast<double>(std::bit_cast<float>(dyn_bits));
          if (static_cast<double>(c.depth) > dzbuf * (1.0 + drel) + dabs) {
            continue;
          }
        } else {
          const double zbuf = static_cast<double>(std::bit_cast<float>(depth_buffer_[pixel]));
          if (static_cast<double>(c.depth) > zbuf * (1.0 + rel) + abs) {
            continue;
          }
        }
        candidates[kept++] = c;
      }
      candidates.resize(kept);
    };
    run_chunked(num_threads, filter_chunk);

    // Merge in chunk order (each chunk kept its sweep order), so the output
    // is deterministic for any thread count.
    for (const auto & candidates : chunk_candidates_) {
      out.insert(out.end(), candidates.begin(), candidates.end());
    }
  }

private:
  // Runs `fn` over the point-range chunks, one chunk per worker thread.
  // `fn` takes either (begin, end, chunk_scratch) for pass 1 or
  // (chunk_scratch) for pass 2; both variants iterate the same chunking.
  template <typename Fn>
  void run_chunked(int num_threads, Fn && fn)
  {
    run_chunked(num_threads, points_.size(), std::forward<Fn>(fn));
  }

  // Chunk-runner over an arbitrary item count: pass 1 / pass 2 iterate the
  // map points, the dynamic splat iterates the scan points. `fn` takes
  // (begin, end, chunk_scratch), (chunk_scratch), or (begin, end).
  template <typename Fn>
  void run_chunked(int num_threads, std::size_t count, Fn && fn)
  {
    if (num_threads == 1) {
      run_chunk(fn, 0, count, 0);
      return;
    }
    std::vector<std::thread> workers;
    workers.reserve(static_cast<std::size_t>(num_threads));
    const std::size_t chunk =
      (count + static_cast<std::size_t>(num_threads) - 1) / static_cast<std::size_t>(num_threads);
    for (int t = 0; t < num_threads; ++t) {
      const std::size_t begin = std::min(count, static_cast<std::size_t>(t) * chunk);
      const std::size_t end = std::min(count, begin + chunk);
      workers.emplace_back(
        [this, &fn, begin, end, t]() { run_chunk(fn, begin, end, static_cast<std::size_t>(t)); });
    }
    for (auto & w : workers) {
      w.join();
    }
  }

  // Dispatches one chunk to the supported `fn` signatures.
  template <typename Fn>
  void run_chunk(Fn & fn, std::size_t begin, std::size_t end, std::size_t chunk_index)
  {
    if constexpr (std::is_invocable_v<Fn, std::size_t, std::size_t, std::vector<VisiblePoint> &>) {
      fn(begin, end, chunk_candidates_[chunk_index]);
    } else if constexpr (std::is_invocable_v<Fn, std::vector<VisiblePoint> &>) {
      fn(chunk_candidates_[chunk_index]);
    } else {
      fn(begin, end);
    }
  }

  // Writes `depth_bits` into every pixel of `buffer` whose center lies within
  // `radius_px` of (u, v), clamped to the image; the center pixel at
  // (int)u, (int)v is always written, so a zero radius splats exactly one
  // pixel. Pixel centers sit on integer coordinates, matching
  // image::bilinear_sample_bgr.
  void splat_depth(
    std::vector<std::uint32_t> & buffer, double u, double v, double radius_px,
    std::uint32_t depth_bits)
  {
    const std::int32_t center_u = static_cast<std::int32_t>(u);
    const std::int32_t center_v = static_cast<std::int32_t>(v);
    depth_min(
      buffer,
      static_cast<std::size_t>(center_v) * depth_width_ + static_cast<std::uint32_t>(center_u),
      depth_bits);
    if (radius_px <= 0.0) {
      return;
    }
    const std::int32_t u_min =
      std::max<std::int32_t>(0, static_cast<std::int32_t>(std::ceil(u - radius_px)));
    const std::int32_t u_max = std::min<std::int32_t>(
      static_cast<std::int32_t>(depth_width_) - 1,
      static_cast<std::int32_t>(std::floor(u + radius_px)));
    const std::int32_t v_min =
      std::max<std::int32_t>(0, static_cast<std::int32_t>(std::ceil(v - radius_px)));
    const std::int32_t v_max = std::min<std::int32_t>(
      static_cast<std::int32_t>(depth_height_) - 1,
      static_cast<std::int32_t>(std::floor(v + radius_px)));
    const double radius_sq = radius_px * radius_px;
    for (std::int32_t py = v_min; py <= v_max; ++py) {
      const double dy = static_cast<double>(py) - v;
      for (std::int32_t px = u_min; px <= u_max; ++px) {
        const double dx = static_cast<double>(px) - u;
        if (dx * dx + dy * dy > radius_sq) {
          continue;
        }
        depth_min(
          buffer, static_cast<std::size_t>(py) * depth_width_ + static_cast<std::uint32_t>(px),
          depth_bits);
      }
    }
  }

  // Atomic min via compare-exchange (C++20 atomic_ref has no fetch_min):
  // the buffer converges to the true minimum regardless of write order.
  void depth_min(std::vector<std::uint32_t> & buffer, std::size_t pixel, std::uint32_t depth_bits)
  {
    std::atomic_ref<std::uint32_t> slot(buffer[pixel]);
    std::uint32_t current = slot.load(std::memory_order_relaxed);
    while (depth_bits < current &&
           !slot.compare_exchange_weak(current, depth_bits, std::memory_order_relaxed)) {
    }
  }

  std::span<const std::array<float, 3>> points_;
  std::span<const float> spacings_;
  ColorizeRasterizerConfig config_;
  const pointcloud::KdTree * tree_ = nullptr;  // optional spatial cull index
  std::vector<std::uint32_t> depth_buffer_;    // float bit patterns, see kInfinityBits
  std::vector<std::uint32_t> dynamic_buffer_;  // per-image scan splat, same layout
  std::uint32_t depth_width_ = 0;
  std::uint32_t depth_height_ = 0;
  std::vector<std::vector<VisiblePoint>> chunk_candidates_;
  std::vector<std::uint32_t> sweep_indices_scratch_;  // per-view kd-tree query result
};

}  // namespace

std::unique_ptr<ColorizeRasterizer> make_cpu_colorize_rasterizer(
  std::span<const std::array<float, 3>> points, std::span<const float> spacings,
  const ColorizeRasterizerConfig & config, const pointcloud::KdTree * tree)
{
  return std::make_unique<CpuColorizeRasterizer>(points, spacings, config, tree);
}

}  // namespace bagwiz::core::slam
