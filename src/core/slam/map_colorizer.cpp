// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/map_colorizer.hpp"

#include "bagwiz/core/image/camera_distortion.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
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

// A rigid transform with a row-major 3x3 rotation, p' = R * p + t. Mirrors
// pcd_concat's quaternion-to-matrix conversion; kept local so the header
// stays free of matrix types.
struct Rigid
{
  std::array<double, 9> r{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  std::array<double, 3> t{0.0, 0.0, 0.0};
};

std::array<double, 9> quat_to_rot(double x, double y, double z, double w)
{
  return {1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y),
          2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
          2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)};
}

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

// One point that projected inside the image: where it landed and how deep.
struct Candidate
{
  std::uint32_t index = 0;  // into the map points span
  std::int32_t u = 0;
  std::int32_t v = 0;
  float depth = 0.0F;
};

}  // namespace

MapColorizer::MapColorizer(
  MapColorizerConfig config, std::span<const std::array<float, 3>> points,
  std::span<const core::TrajectoryPose> trajectory)
: config_(std::move(config)), points_(points), trajectory_(trajectory), accumulators_(points.size())
{
}

bool MapColorizer::add_image(
  std::int64_t stamp_ns, std::span<const std::byte> bgr, std::uint32_t width, std::uint32_t height)
{
  if (points_.empty() || trajectory_.empty()) {
    ++images_skipped_;
    return false;
  }
  if (width == 0 || height == 0 || bgr.size() != static_cast<std::size_t>(width) * 3U * height) {
    ++images_skipped_;
    return false;
  }
  // Reject stamps outside the trajectory span: lookup_pose would clamp to an
  // endpoint pose, projecting the map from a viewpoint the platform never had
  // at that time and smearing wrong colors over it.
  if (stamp_ns < trajectory_.front().timestamp_ns || stamp_ns > trajectory_.back().timestamp_ns) {
    ++images_skipped_;
    return false;
  }
  const auto pose = core::lookup_pose(stamp_ns, trajectory_);
  if (!pose) {
    ++images_skipped_;
    return false;
  }

  // Camera pose in the world: T_world_cam = T_world_cloud * T_cloud_cam, then
  // inverted once so each point costs one rotate+translate.
  Rigid t_world_cloud;
  t_world_cloud.r = quat_to_rot(pose->qx, pose->qy, pose->qz, pose->qw);
  t_world_cloud.t = {pose->tx, pose->ty, pose->tz};
  Rigid t_cloud_cam;
  t_cloud_cam.r = quat_to_rot(
    config_.t_cloud_cam.rotation_xyzw[0], config_.t_cloud_cam.rotation_xyzw[1],
    config_.t_cloud_cam.rotation_xyzw[2], config_.t_cloud_cam.rotation_xyzw[3]);
  t_cloud_cam.t = config_.t_cloud_cam.translation;
  const Rigid t_cam_world = invert(compose(t_world_cloud, t_cloud_cam));

  // Rescale the intrinsics when the delivered image differs from the
  // calibrated resolution (e.g. a downscaled republished stream).
  image::CameraInfo cam = config_.camera;
  if (cam.width != 0 && cam.height != 0 && (cam.width != width || cam.height != height)) {
    cam = image::scale_camera_info(
      cam, static_cast<double>(width) / cam.width, static_cast<double>(height) / cam.height);
  }
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

  // Pass 1 (parallel over point chunks): project every map point into the
  // image, collecting the in-bounds hits per chunk.
  const int num_threads =
    std::clamp(config_.num_threads, 1, static_cast<int>(std::max<std::size_t>(1, points_.size())));
  std::vector<std::vector<Candidate>> chunk_candidates(static_cast<std::size_t>(num_threads));
  auto project_chunk = [&](std::size_t begin, std::size_t end, std::vector<Candidate> & out) {
    out.reserve((end - begin) / 4);  // rough estimate, mirrors project_pointcloud
    for (std::size_t i = begin; i < end; ++i) {
      const auto & p = points_[i];
      const double x = t_cam_world.r[0] * p[0] + t_cam_world.r[1] * p[1] + t_cam_world.r[2] * p[2] +
                       t_cam_world.t[0];
      const double y = t_cam_world.r[3] * p[0] + t_cam_world.r[4] * p[1] + t_cam_world.r[5] * p[2] +
                       t_cam_world.t[1];
      const double z = t_cam_world.r[6] * p[0] + t_cam_world.r[7] * p[1] + t_cam_world.r[8] * p[2] +
                       t_cam_world.t[2];
      if (z <= 0.0) {
        continue;
      }
      if (x * x + y * y + z * z > max_range_sq) {
        continue;
      }
      double nx = x / z;
      double ny = y / z;
      if (apply_distortion) {
        // Fold-back artifacts (points beyond the lens model's domain) must
        // not sample a color; see camera_distortion.hpp.
        const auto distorted =
          image::distort_for_raw_image(nx, ny, distortion_model, cam.d, fx, fy);
        if (!distorted) {
          continue;
        }
        nx = distorted->x;
        ny = distorted->y;
      }
      const double u = fx * nx + cx;
      const double v = fy * ny + cy;
      if (u < 0.0 || u >= width || v < 0.0 || v >= height) {
        continue;
      }
      Candidate c;
      c.index = static_cast<std::uint32_t>(i);
      c.u = static_cast<std::int32_t>(u);
      c.v = static_cast<std::int32_t>(v);
      c.depth = static_cast<float>(z);
      out.push_back(c);
    }
  };
  if (num_threads == 1) {
    project_chunk(0, points_.size(), chunk_candidates[0]);
  } else {
    std::vector<std::thread> workers;
    workers.reserve(static_cast<std::size_t>(num_threads));
    const std::size_t chunk = (points_.size() + static_cast<std::size_t>(num_threads) - 1) /
                              static_cast<std::size_t>(num_threads);
    for (int t = 0; t < num_threads; ++t) {
      const std::size_t begin = std::min(points_.size(), static_cast<std::size_t>(t) * chunk);
      const std::size_t end = std::min(points_.size(), begin + chunk);
      workers.emplace_back(
        project_chunk, begin, end, std::ref(chunk_candidates[static_cast<std::size_t>(t)]));
    }
    for (auto & w : workers) {
      w.join();
    }
  }

  // Z-buffer over coarse pixel cells: keep the nearest depth per cell. Serial
  // — the merge is a fraction of the projection work.
  const std::uint32_t cell = static_cast<std::uint32_t>(std::max(1, config_.zbuffer_cell_px));
  const std::uint32_t cells_w = (width + cell - 1) / cell;
  const std::uint32_t cells_h = (height + cell - 1) / cell;
  std::vector<float> zbuffer(
    static_cast<std::size_t>(cells_w) * cells_h, std::numeric_limits<float>::infinity());
  for (const auto & candidates : chunk_candidates) {
    for (const auto & c : candidates) {
      const std::size_t cell_index =
        static_cast<std::size_t>(static_cast<std::uint32_t>(c.v) / cell) * cells_w +
        static_cast<std::uint32_t>(c.u) / cell;
      zbuffer[cell_index] = std::min(zbuffer[cell_index], c.depth);
    }
  }

  // Pass 2 (parallel over the same chunks): points near their cell's nearest
  // depth sample the pixel color. Point indices are unique within an image,
  // so the per-point accumulators need no synchronization.
  auto accumulate_chunk = [&](const std::vector<Candidate> & candidates) {
    for (const auto & c : candidates) {
      const std::size_t cell_index =
        static_cast<std::size_t>(static_cast<std::uint32_t>(c.v) / cell) * cells_w +
        static_cast<std::uint32_t>(c.u) / cell;
      const double limit =
        static_cast<double>(zbuffer[cell_index]) * (1.0 + config_.depth_rel_tolerance) +
        config_.depth_abs_tolerance;
      if (static_cast<double>(c.depth) > limit) {
        continue;
      }
      const std::size_t pixel =
        (static_cast<std::size_t>(c.v) * width + static_cast<std::size_t>(c.u)) * 3U;
      auto & acc = accumulators_[c.index];
      acc.b += static_cast<std::uint32_t>(std::to_integer<std::uint8_t>(bgr[pixel + 0]));
      acc.g += static_cast<std::uint32_t>(std::to_integer<std::uint8_t>(bgr[pixel + 1]));
      acc.r += static_cast<std::uint32_t>(std::to_integer<std::uint8_t>(bgr[pixel + 2]));
      ++acc.count;
    }
  };
  if (num_threads == 1) {
    accumulate_chunk(chunk_candidates[0]);
  } else {
    std::vector<std::thread> workers;
    workers.reserve(chunk_candidates.size());
    for (const auto & candidates : chunk_candidates) {
      workers.emplace_back(accumulate_chunk, std::cref(candidates));
    }
    for (auto & w : workers) {
      w.join();
    }
  }

  ++images_used_;
  return true;
}

MapColorizeResult MapColorizer::finish() const
{
  MapColorizeResult result;
  result.colors.assign(points_.size(), kUncoloredGray);
  result.observed.assign(points_.size(), 0);
  for (std::size_t i = 0; i < accumulators_.size(); ++i) {
    const auto & acc = accumulators_[i];
    if (acc.count == 0) {
      continue;
    }
    // Round-to-nearest average per channel.
    result.colors[i] = {
      static_cast<std::uint8_t>((acc.r + acc.count / 2) / acc.count),
      static_cast<std::uint8_t>((acc.g + acc.count / 2) / acc.count),
      static_cast<std::uint8_t>((acc.b + acc.count / 2) / acc.count)};
    result.observed[i] = 1;
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
  for (const auto & result : results) {
    merged.images_used += result.images_used;
    merged.images_skipped += result.images_skipped;
  }
  for (std::size_t i = 0; i < count; ++i) {
    for (const auto & result : results) {
      if (i < result.observed.size() && result.observed[i] != 0) {
        merged.colors[i] = result.colors[i];
        merged.observed[i] = 1;
        ++merged.colored_points;
        break;
      }
    }
  }
  return merged;
}

}  // namespace bagwiz::core::slam
