// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "visual_factors.hpp"

#include <algorithm>

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

Stats build_visual_factors(
  std::span<const VisualObservation> observations, std::span<const Eigen::Isometry3d> t_lidar_cams,
  std::span<const SubmapView> submaps, const Params & params,
  std::vector<gtsam::NonlinearFactor::shared_ptr> & out)
{
  (void)observations;
  (void)t_lidar_cams;
  (void)submaps;
  (void)params;
  (void)out;
  return Stats{};
}

}  // namespace bagwiz::core::slam::visual
