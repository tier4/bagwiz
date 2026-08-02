// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "visual_factors.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholePose.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/triangulation.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/SmartFactorParams.h>
#include <gtsam/slam/SmartProjectionRigFactor.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace bagwiz::core::slam::visual
{

std::optional<Eigen::Isometry3d> interpolate_origin_pose(const SubmapView & view, double t)
{
  const auto & stamps = view.frame_stamps;
  if (stamps.empty() || t < stamps.front() || t > stamps.back()) {
    return std::nullopt;
  }

  // First stamp strictly greater than t. Since t >= stamps.front() (checked
  // above), hi is never 0; hi == stamps.size() means t == stamps.back() (no
  // stamp is greater), handled below without a second frame to pair with.
  const auto hi_it = std::upper_bound(stamps.begin(), stamps.end(), t);
  const auto hi = static_cast<std::size_t>(hi_it - stamps.begin());
  if (hi == stamps.size()) {
    return view.T_origin_frames.back();
  }

  const std::size_t lo = hi - 1;
  const double t0 = stamps[lo];
  const double t1 = stamps[hi];
  const double alpha = (t - t0) / (t1 - t0);
  const Eigen::Isometry3d & p0 = view.T_origin_frames[lo];
  const Eigen::Isometry3d & p1 = view.T_origin_frames[hi];

  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = p0.translation() + alpha * (p1.translation() - p0.translation());
  result.linear() = Eigen::Quaterniond(p0.rotation())
                      .slerp(alpha, Eigen::Quaterniond(p1.rotation()))
                      .toRotationMatrix();
  return result;
}

std::optional<std::size_t> submap_for_stamp(std::span<const SubmapView> views, double t)
{
  // Last view whose span start is <= t, then verify t is still within its span
  // (the search alone cannot detect an inter-submap gap).
  const auto it = std::upper_bound(
    views.begin(), views.end(), t,
    [](double value, const SubmapView & view) { return value < view.frame_stamps.front(); });
  if (it == views.begin()) {
    return std::nullopt;
  }

  const std::size_t idx = static_cast<std::size_t>(it - views.begin()) - 1;
  const SubmapView & view = views[idx];
  if (view.frame_stamps.empty() || t > view.frame_stamps.back()) {
    return std::nullopt;
  }
  return idx;
}

namespace
{

using VoxelSet = std::unordered_set<std::uint64_t>;
using VoxelIndex = Eigen::Matrix<std::int64_t, 3, 1>;

// Two views degenerate under forward motion, so a track is only worth a factor
// once it has been seen at least three times.
constexpr std::size_t kMinObservations = 3;

// Beyond this range a submap pair's few-metre baseline yields no usable
// parallax, so a point triangulated that far away is treated as a failure
// rather than a weak constraint.
constexpr double kLandmarkDistanceThreshold = 150.0;

// Reject a track whose worst reprojection error exceeds this many obs_sigma.
// This is what removes moving objects and drifted tracks.
constexpr double kOutlierSigmas = 3.0;

// Observations are undistorted NORMALIZED image coordinates, so every rig camera
// is the identity pinhole and all image-plane quantities here (obs_sigma,
// reprojection errors, rank tolerance) are in normalized units, not pixels.
// GTSAM's TriangulationParameters default rank tolerance (1.0) is sized for
// pixel measurements, whose DLT matrix carries focal-length-scaled entries; at
// focal 1 that same absolute threshold flags well-conditioned tracks as
// rank-deficient, so use triangulateDLT's own default and let the distance,
// cheirality and reprojection checks judge quality instead.
constexpr double kRankTolerance = 1.0e-9;

VoxelIndex voxel_index(const Eigen::Vector3d & p, double voxel)
{
  return VoxelIndex(
    static_cast<std::int64_t>(std::floor(p.x() / voxel)),
    static_cast<std::int64_t>(std::floor(p.y() / voxel)),
    static_cast<std::int64_t>(std::floor(p.z() / voxel)));
}

// 21 bits per axis, biased so the cast stays in range. Exact for |index| < 2^20
// (a million voxels per axis, far beyond any submap's extent); past that the
// wrap only aliases distant voxels onto each other, which can make the support
// gate slightly more permissive but never stricter.
std::uint64_t voxel_key(const VoxelIndex & index)
{
  constexpr std::int64_t kBias = 1 << 20;
  constexpr std::uint64_t kMask = (1ULL << 21) - 1;
  const auto axis = [](std::int64_t v) { return static_cast<std::uint64_t>(v + kBias) & kMask; };
  return axis(index.x()) | (axis(index.y()) << 21) | (axis(index.z()) << 42);
}

VoxelSet build_voxel_set(const gtsam_points::PointCloud & cloud, double voxel)
{
  VoxelSet occupied;
  occupied.reserve(cloud.size());
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    occupied.insert(voxel_key(voxel_index(cloud.points[i].head<3>(), voxel)));
  }
  return occupied;
}

// Supported when the cloud occupies the point's own voxel or any of the 26
// around it, i.e. there is a LiDAR return within roughly `voxel` of the point.
bool voxel_set_covers(const VoxelSet & occupied, const Eigen::Vector3d & p, double voxel)
{
  const VoxelIndex center = voxel_index(p, voxel);
  for (std::int64_t dx = -1; dx <= 1; ++dx) {
    for (std::int64_t dy = -1; dy <= 1; ++dy) {
      for (std::int64_t dz = -1; dz <= 1; ++dz) {
        if (occupied.count(voxel_key(center + VoxelIndex(dx, dy, dz))) != 0) {
          return true;
        }
      }
    }
  }
  return false;
}

// One observation once it has been tied to a submap: the camera pose it was
// taken from, expressed in that submap's own origin frame (so it stays valid
// when global mapping re-optimizes the origin), plus the measurement and the
// observation's sampled color (used only by the landmark export; factor
// construction ignores it).
struct TrackObs
{
  std::size_t view = 0;
  Eigen::Isometry3d T_origin_cam;
  gtsam::Point2 measurement;
  std::array<std::uint8_t, 3> rgb{};
};

// Keyed by (camera_id, track_id), never by track_id alone: see TrackKey.
using TrackGroups =
  std::unordered_map<TrackKey, std::vector<const VisualObservation *>, TrackKeyHash>;

TrackGroups group_by_track(std::span<const VisualObservation> observations)
{
  TrackGroups groups;
  for (const VisualObservation & obs : observations) {
    groups[track_key(obs)].push_back(&obs);
  }
  for (auto & entry : groups) {
    std::sort(
      entry.second.begin(), entry.second.end(),
      [](const VisualObservation * a, const VisualObservation * b) {
        return a->stamp_ns < b->stamp_ns;
      });
  }
  return groups;
}

// Thin a long track down to `max_obs` evenly spaced observations. First and last
// are always kept, so the thinned track still spans the same submaps (and the
// same parallax) as the full one.
std::vector<const VisualObservation *> subsample(
  const std::vector<const VisualObservation *> & group, int max_obs)
{
  if (max_obs <= 0) {
    return group;
  }
  const auto limit = static_cast<std::size_t>(max_obs);
  if (group.size() <= limit) {
    return group;
  }
  if (limit < 2) {
    return {group.front()};
  }

  std::vector<const VisualObservation *> kept;
  kept.reserve(limit);
  const double step = static_cast<double>(group.size() - 1) / static_cast<double>(limit - 1);
  for (std::size_t i = 0; i < limit; ++i) {
    kept.push_back(group[static_cast<std::size_t>(std::llround(static_cast<double>(i) * step))]);
  }
  return kept;
}

std::vector<TrackObs> associate(
  const std::vector<const VisualObservation *> & group,
  std::span<const Eigen::Isometry3d> t_lidar_cams, std::span<const SubmapView> views)
{
  std::vector<TrackObs> track;
  track.reserve(group.size());
  for (const VisualObservation * obs : group) {
    if (obs->camera_id < 0 || static_cast<std::size_t>(obs->camera_id) >= t_lidar_cams.size()) {
      continue;  // no extrinsic for this camera: nothing to project through
    }
    const double stamp = static_cast<double>(obs->stamp_ns) * 1.0e-9;
    const auto view = submap_for_stamp(views, stamp);
    if (!view.has_value()) {
      continue;  // inter-submap gap: no LiDAR pose to hang the observation on
    }
    const auto T_origin_lidar = interpolate_origin_pose(views[*view], stamp);
    if (!T_origin_lidar.has_value()) {
      continue;  // submap_for_stamp already bounds the stamp; defensive only
    }
    track.push_back(
      TrackObs{
        *view, *T_origin_lidar * t_lidar_cams[static_cast<std::size_t>(obs->camera_id)],
        gtsam::Point2(obs->x, obs->y), obs->rgb});
  }
  return track;
}

std::size_t distinct_views(const std::vector<TrackObs> & track)
{
  std::unordered_set<std::size_t> views;
  for (const TrackObs & obs : track) {
    views.insert(obs.view);
  }
  return views.size();
}

// Triangulate in the world frame at the submaps' current pose estimates: both
// the seed the smart factor re-triangulates from and the point the LiDAR gate
// tests.
gtsam::TriangulationResult triangulate_world(
  const std::vector<TrackObs> & track, std::span<const SubmapView> views,
  const gtsam::Cal3_S2::shared_ptr & calibration, const gtsam::TriangulationParameters & params)
{
  gtsam::CameraSet<RigCamera> world_cams;
  RigCamera::MeasurementVector measurements;
  world_cams.reserve(track.size());
  measurements.reserve(track.size());
  for (const TrackObs & obs : track) {
    const Eigen::Isometry3d T_world_cam = views[obs.view].T_world_origin * obs.T_origin_cam;
    world_cams.emplace_back(gtsam::Pose3(T_world_cam.matrix()), calibration);
    measurements.push_back(obs.measurement);
  }
  return gtsam::triangulateSafe(world_cams, measurements, params);
}

// Whether the gate has host geometry to judge this submap with at all. A
// gtsam_points cloud can be GPU-resident — num_points > 0 while the host
// `points` array is null — and an empty cloud supports nothing anywhere, so
// both must read as "no cloud" and make the gate abstain. Treating either as a
// usable cloud would reject every track in that submap (or dereference null).
bool has_host_points(const SubmapView & view)
{
  return view.cloud != nullptr && view.cloud->size() > 0 && view.cloud->points != nullptr;
}

bool all_clouds_present(const std::vector<TrackObs> & track, std::span<const SubmapView> views)
{
  return std::all_of(track.begin(), track.end(), [views](const TrackObs & obs) {
    return has_host_points(views[obs.view]);
  });
}

// A point is kept when at least one of its submaps has a LiDAR return next to
// it: purely visual points floating in free space (sky, reflections, distant
// background) would tie the submaps together through geometry the LiDAR map
// never saw. Requires all_clouds_present(track, views) - the gate abstains
// rather than judges when a submap has no host points, so the caller checks
// that first and this dereferences view.cloud unconditionally.
bool lidar_supported(
  const std::vector<TrackObs> & track, std::span<const SubmapView> views,
  const gtsam::Point3 & p_world, double voxel, std::unordered_map<std::size_t, VoxelSet> & cache)
{
  for (const TrackObs & obs : track) {
    const SubmapView & view = views[obs.view];
    auto cached = cache.find(obs.view);
    if (cached == cache.end()) {
      cached = cache.emplace(obs.view, build_voxel_set(*view.cloud, voxel)).first;
    }
    if (voxel_set_covers(cached->second, view.T_world_origin.inverse() * p_world, voxel)) {
      return true;
    }
  }
  return false;
}

// One rig camera per observation, its pose the interpolated intra-submap
// extrinsic (constant with respect to the optimized submap origins), keyed by
// the submap the observation fell in. Observations from the same submap
// therefore repeat a pose key with distinct camera ids, which is the rig
// factor's documented non-unique-keys mode.
RigFactor::shared_ptr make_factor(
  const std::vector<TrackObs> & track, std::span<const SubmapView> views,
  const gtsam::Cal3_S2::shared_ptr & calibration, const gtsam::SharedNoiseModel & noise,
  const gtsam::SmartProjectionParams & params)
{
  using gtsam::symbol_shorthand::X;

  auto rig = std::make_shared<gtsam::CameraSet<RigCamera>>();
  rig->reserve(track.size());
  auto factor = std::make_shared<RigFactor>(noise, rig, params);
  for (std::size_t i = 0; i < track.size(); ++i) {
    rig->emplace_back(gtsam::Pose3(track[i].T_origin_cam.matrix()), calibration);
    factor->add(track[i].measurement, X(views[track[i].view].id), i);
  }
  return factor;
}

}  // namespace

gtsam::Cal3_S2::shared_ptr normalized_calibration()
{
  return std::make_shared<gtsam::Cal3_S2>(1.0, 1.0, 0.0, 0.0, 0.0);
}

gtsam::SmartProjectionParams make_smart_projection_params()
{
  // HESSIAN + ZERO_ON_DEGENERACY are not choices — the rig factor's
  // constructor throws on anything else. setRankTolerance(1e-9) is the
  // load-bearing line: GTSAM's default of 1.0 is sized for pixel measurements
  // and flags every normalized-coordinate factor as degenerate, zeroing it.
  // Do NOT add setDynamicOutlierRejectionThreshold: a 3-sigma gate there
  // zeroes exactly the factors pulling hardest, measured to leave a 0.36 m
  // submap perturbation completely uncorrected.
  gtsam::SmartProjectionParams params(gtsam::HESSIAN, gtsam::ZERO_ON_DEGENERACY);
  params.setRankTolerance(kRankTolerance);
  params.setLandmarkDistanceThreshold(kLandmarkDistanceThreshold);
  return params;
}

Stats build_visual_factors(
  std::span<const VisualObservation> observations, std::span<const Eigen::Isometry3d> t_lidar_cams,
  std::span<const SubmapView> submaps, const Params & params,
  std::vector<gtsam::NonlinearFactor::shared_ptr> & out)
{
  const auto calibration = normalized_calibration();
  const gtsam::TriangulationParameters seed_params(
    kRankTolerance, /*enableEPI=*/false, kLandmarkDistanceThreshold,
    kOutlierSigmas * params.obs_sigma);

  const auto factor_params = make_smart_projection_params();
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(2, params.obs_sigma);

  const TrackGroups groups = group_by_track(observations);
  Stats stats;
  stats.tracks_total = groups.size();

  // The emitted factor order is part of the graph handed to global mapping, so
  // walk the tracks in key order instead of the hash map's.
  std::vector<TrackKey> keys;
  keys.reserve(groups.size());
  for (const auto & entry : groups) {
    keys.push_back(entry.first);
  }
  std::sort(keys.begin(), keys.end());

  std::unordered_map<std::size_t, VoxelSet> voxel_cache;  // per call, per submap
  for (const TrackKey & key : keys) {
    const std::vector<TrackObs> track =
      associate(subsample(groups.at(key), params.max_obs_per_track), t_lidar_cams, submaps);
    if (distinct_views(track) < 2) {
      ++stats.tracks_single_submap;  // constrains nothing: one key only
      continue;
    }
    if (track.size() < kMinObservations) {
      ++stats.tracks_too_short;
      continue;
    }

    const gtsam::TriangulationResult point =
      triangulate_world(track, submaps, calibration, seed_params);
    if (!point.valid()) {
      ++stats.tracks_triangulation_failed;
      continue;
    }
    if (
      params.gate_distance > 0.0 && all_clouds_present(track, submaps) &&
      !lidar_supported(track, submaps, *point, params.gate_distance, voxel_cache)) {
      ++stats.tracks_gated;
      continue;
    }

    out.push_back(make_factor(track, submaps, calibration, noise, factor_params));
    ++stats.factors;
  }
  return stats;
}

std::vector<Landmark> triangulate_landmarks(
  std::span<const VisualObservation> observations, std::span<const Eigen::Isometry3d> t_lidar_cams,
  std::span<const SubmapView> submaps, const Params & params)
{
  const auto calibration = normalized_calibration();
  const gtsam::TriangulationParameters seed_params(
    kRankTolerance, /*enableEPI=*/false, kLandmarkDistanceThreshold,
    kOutlierSigmas * params.obs_sigma);

  const TrackGroups groups = group_by_track(observations);

  // Same deterministic track order as build_visual_factors, so the exported
  // landmark set is stable across runs over identical input.
  std::vector<TrackKey> keys;
  keys.reserve(groups.size());
  for (const auto & entry : groups) {
    keys.push_back(entry.first);
  }
  std::sort(keys.begin(), keys.end());

  std::vector<Landmark> landmarks;
  landmarks.reserve(keys.size());
  for (const TrackKey & key : keys) {
    const std::vector<TrackObs> track =
      associate(subsample(groups.at(key), params.max_obs_per_track), t_lidar_cams, submaps);
    // Same selection as factor construction: a track qualifies only with
    // enough parallax (>= 2 submaps) and enough observations to triangulate.
    if (distinct_views(track) < 2 || track.size() < kMinObservations) {
      continue;
    }
    const gtsam::TriangulationResult point =
      triangulate_world(track, submaps, calibration, seed_params);
    if (!point.valid()) {
      continue;
    }
    landmarks.push_back(
      Landmark{
        {static_cast<float>(point->x()), static_cast<float>(point->y()),
         static_cast<float>(point->z())},
        track.front().rgb});
  }
  return landmarks;
}

}  // namespace bagwiz::core::slam::visual
