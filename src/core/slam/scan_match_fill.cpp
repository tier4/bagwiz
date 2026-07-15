// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/scan_match_fill.hpp"

#include <gtsam_points/ann/ivox.hpp>
#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/features/covariance_estimation.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <gtsam/base/make_shared.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <memory>
#include <vector>

namespace bagwiz::core::slam
{
namespace
{

// iVox LRU horizon set effectively unbounded: the fill inserts at most a few
// hundred frames/scans per run (far below this), so no voxel is ever evicted and
// the whole boundary neighborhood stays resident as the registration target.
constexpr std::size_t kLruHorizonUnbounded = 1'000'000;

// gtsam_points stores points as homogeneous Eigen::Vector4d (w = 1). Both the
// PointCloudCPU constructor and estimate_covariances() take that layout.
std::vector<Eigen::Vector4d> to_homogeneous(const std::vector<Eigen::Vector3d> & pts)
{
  std::vector<Eigen::Vector4d> out;
  out.reserve(pts.size());
  for (const auto & p : pts) {
    out.emplace_back(p.x(), p.y(), p.z(), 1.0);
  }
  return out;
}

}  // namespace

struct ScanMatchFiller::Impl
{
  const ScanMatchParams params;                      // Con.4: set once, never mutated
  const std::shared_ptr<gtsam_points::iVox> target;  // pointer const; pointee grows
  bool has_target = false;  // set once a batch is inserted (iVox exposes no count)

  explicit Impl(const ScanMatchParams & p)
  : params(p), target(std::make_shared<gtsam_points::iVox>(p.voxel_resolution))
  {
    // The fill windows insert only a few dozen frames, so keep every voxel
    // resident (defeat the LRU eviction) — the whole boundary neighborhood must
    // stay available as a registration target across the chain.
    target->set_lru_horizon(kLruHorizonUnbounded);
  }

  // Build a covariance-bearing gtsam_points cloud from world/LiDAR points, the
  // exact preparation GLIM's loose-init applies before a GICP registration:
  // per-point covariances estimated from each point's own k-neighborhood.
  gtsam_points::PointCloudCPU::Ptr make_cloud(const std::vector<Eigen::Vector3d> & pts) const
  {
    const auto homogeneous = to_homogeneous(pts);
    auto cloud = std::make_shared<gtsam_points::PointCloudCPU>(homogeneous);
    cloud->add_covs(
      gtsam_points::estimate_covariances(homogeneous, params.k_neighbors, params.num_threads));
    return cloud;
  }
};

ScanMatchFiller::ScanMatchFiller(ScanMatchParams params) : impl_(std::make_unique<Impl>(params))
{
}
ScanMatchFiller::~ScanMatchFiller() = default;
ScanMatchFiller::ScanMatchFiller(ScanMatchFiller &&) noexcept = default;
ScanMatchFiller & ScanMatchFiller::operator=(ScanMatchFiller &&) noexcept = default;

void ScanMatchFiller::insert_target(const std::vector<Eigen::Vector3d> & world_points)
{
  if (static_cast<int>(world_points.size()) < impl_->params.min_points) {
    return;  // too sparse for a stable covariance estimate
  }
  const auto cloud = impl_->make_cloud(world_points);
  impl_->target->insert(*cloud);
  impl_->has_target = true;
}

bool ScanMatchFiller::target_empty() const noexcept
{
  return !impl_->has_target;
}

ScanMatchResult ScanMatchFiller::register_scan(
  const std::vector<Eigen::Vector3d> & source_lidar_points,
  const Eigen::Isometry3d & init_T_world_lidar) const
{
  ScanMatchResult result;
  result.T_world_lidar = init_T_world_lidar;  // unchanged unless the fit is accepted

  if (!impl_->has_target) {
    return result;  // nothing to register against
  }
  if (static_cast<int>(source_lidar_points.size()) < impl_->params.min_points) {
    return result;
  }

  const auto source = impl_->make_cloud(source_lidar_points);

  // GICP factor: target held fixed at identity (the points are already in the
  // world frame), variable 0 is the world<-LiDAR pose that aligns the source to
  // it. Mirrors GLIM's loose_initial_state_estimation registration.
  auto factor = gtsam::make_shared<
    gtsam_points::IntegratedGICPFactor_<gtsam_points::iVox, gtsam_points::PointCloud>>(
    gtsam::Pose3::Identity(), 0, impl_->target, source, impl_->target);
  factor->set_num_threads(impl_->params.num_threads);
  factor->set_max_correspondence_distance(impl_->params.max_correspondence_distance);

  gtsam::NonlinearFactorGraph graph;
  graph.add(factor);

  gtsam::Values values;
  values.insert(0, gtsam::Pose3(init_T_world_lidar.matrix()));

  gtsam_points::LevenbergMarquardtExtParams lm_params;
  lm_params.setMaxIterations(impl_->params.max_iterations);
  values = gtsam_points::LevenbergMarquardtOptimizerExt(graph, values, lm_params).optimize();

  const gtsam::Pose3 optimized = values.at<gtsam::Pose3>(0);

  // Refresh correspondences at the optimized pose so inlier_fraction() reflects
  // the final fit: error() -> evaluate() -> update_correspondences().
  factor->error(values);
  result.inlier_fraction = factor->inlier_fraction();
  if (result.inlier_fraction >= impl_->params.min_inlier_fraction) {
    result.converged = true;
    result.T_world_lidar = Eigen::Isometry3d(optimized.matrix());
  }
  return result;
}

}  // namespace bagwiz::core::slam
