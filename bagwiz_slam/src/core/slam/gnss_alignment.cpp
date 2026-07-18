// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/gnss_alignment.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace bagwiz::core::slam
{
namespace
{

// The planar (yaw + x-y translation + z offset) world<-GNSS transform fitted by the
// closed-form 2-D Procrustes below. `valid` is false for empty/mismatched/degenerate
// input. R = [[c,-s],[s,c]] is the est->gnss rotation; world = R^T(gnss.xy - t).
struct Planar2D
{
  bool valid = false;
  double c = 1.0;
  double s = 0.0;
  double tx = 0.0;
  double ty = 0.0;
  double dz = 0.0;
};

// Closed-form 2-D Procrustes (the planar special case of glim_ext's SVD-based
// Umeyama): for centered point sets the optimal proper rotation mapping `est`
// onto `gnss` has angle atan2(Syx - Sxy, Sxx + Syy), where S = Σ (gnss_c)(est_c)^T
// over the x-y components. This always yields a proper rotation (no reflection
// case to handle) and needs no matrix library. The resulting R maps est -> gnss.
Planar2D fit_planar(
  const std::vector<std::array<double, 3>> & est, const std::vector<std::array<double, 3>> & gnss)
{
  Planar2D out;
  if (est.empty() || est.size() != gnss.size()) {
    return out;  // valid == false
  }

  const auto n = static_cast<double>(est.size());
  std::array<double, 3> mean_est{0.0, 0.0, 0.0};
  std::array<double, 3> mean_gnss{0.0, 0.0, 0.0};
  for (std::size_t i = 0; i < est.size(); ++i) {
    for (int k = 0; k < 3; ++k) {
      mean_est[k] += est[i][k];
      mean_gnss[k] += gnss[i][k];
    }
  }
  for (int k = 0; k < 3; ++k) {
    mean_est[k] /= n;
    mean_gnss[k] /= n;
  }

  // 2-D cross-covariance S = Σ (gnss_c)(est_c)^T over x-y (row = gnss, col = est).
  double s_xx = 0.0;
  double s_xy = 0.0;
  double s_yx = 0.0;
  double s_yy = 0.0;
  for (std::size_t i = 0; i < est.size(); ++i) {
    const double ex = est[i][0] - mean_est[0];
    const double ey = est[i][1] - mean_est[1];
    const double gx = gnss[i][0] - mean_gnss[0];
    const double gy = gnss[i][1] - mean_gnss[1];
    s_xx += gx * ex;
    s_xy += gx * ey;
    s_yx += gy * ex;
    s_yy += gy * ey;
  }

  // Rotation est -> gnss. With a degenerate spread (single point / collinear at a
  // point) both args are ~0 and atan2(0,0)=0, i.e. identity rotation.
  const double theta = std::atan2(s_yx - s_xy, s_xx + s_yy);
  out.c = std::cos(theta);
  out.s = std::sin(theta);
  // T_gnss_world (2-D): gnss.xy ~= R * est.xy + t, with R = [[c,-s],[s,c]].
  out.tx = mean_gnss[0] - (out.c * mean_est[0] - out.s * mean_est[1]);
  out.ty = mean_gnss[1] - (out.s * mean_est[0] + out.c * mean_est[1]);
  // z is not rotated; its translation mirrors glim_ext (T.translation().z()).
  out.dz = mean_gnss[2] - mean_est[2];
  out.valid = true;
  return out;
}

// Map one GNSS point into the world frame: world = R^T(gnss.xy - t); world.z =
// gnss.z - dz. R^T = [[c,s],[-s,c]].
std::array<double, 3> map_world(const Planar2D & a, const std::array<double, 3> & g)
{
  const double qx = g[0] - a.tx;
  const double qy = g[1] - a.ty;
  return {a.c * qx + a.s * qy, -a.s * qx + a.c * qy, g[2] - a.dz};
}

}  // namespace

std::vector<std::array<double, 3>> align_gnss_to_world(
  const std::vector<std::array<double, 3>> & est, const std::vector<std::array<double, 3>> & gnss)
{
  std::vector<std::array<double, 3>> world;
  const Planar2D a = fit_planar(est, gnss);
  if (!a.valid) {
    return world;
  }
  world.reserve(gnss.size());
  for (const auto & g : gnss) {
    world.push_back(map_world(a, g));
  }
  return world;
}

GnssOffsetTargets gnss_targets_with_offset(
  const std::vector<std::array<double, 3>> & origins,
  const std::vector<std::array<double, 3>> & offsets_world,
  const std::vector<std::array<double, 3>> & gnss)
{
  GnssOffsetTargets out;  // cos=1, sin=0, empty targets by default
  if (origins.empty() || origins.size() != gnss.size() || origins.size() != offsets_world.size()) {
    return out;
  }

  // Predicted antenna position per submap = origin + (R_origin · lever_arm). The
  // rigid world<-GNSS fit then matches antenna-to-antenna, so the heading-
  // dependent lever-arm no longer leaks into the alignment residual.
  std::vector<std::array<double, 3>> antenna;
  antenna.reserve(origins.size());
  for (std::size_t i = 0; i < origins.size(); ++i) {
    antenna.push_back(
      {origins[i][0] + offsets_world[i][0], origins[i][1] + offsets_world[i][1],
       origins[i][2] + offsets_world[i][2]});
  }

  const Planar2D a = fit_planar(antenna, gnss);
  if (!a.valid) {
    return out;  // degenerate alignment
  }
  // world = R^T(gnss - t); the ENU->world rotation is R^T = M = [[c,s],[-s,c]],
  // so report cos = c, sin = s for the covariance rotation downstream.
  out.world_from_enu_cos = a.c;
  out.world_from_enu_sin = a.s;

  // Map each antenna fix into the world frame, then remove the same world-frame
  // offset to land the prior target on the submap origin (lever-arm-free).
  out.targets.reserve(origins.size());
  for (std::size_t i = 0; i < origins.size(); ++i) {
    const std::array<double, 3> w = map_world(a, gnss[i]);
    out.targets.push_back(
      {w[0] - offsets_world[i][0], w[1] - offsets_world[i][1], w[2] - offsets_world[i][2]});
  }
  return out;
}

std::array<double, 9> gnss_world_prior_covariance(
  const std::array<double, 4> & cov_enu_horizontal, double world_from_enu_cos,
  double world_from_enu_sin, double floor_sigma, double inflation, double z_variance)
{
  // Symmetrize the input 2x2 (off-diagonals should match; average defensively).
  const double a = cov_enu_horizontal[0];
  const double b = 0.5 * (cov_enu_horizontal[1] + cov_enu_horizontal[2]);
  const double d = cov_enu_horizontal[3];
  const double c = world_from_enu_cos;
  const double s = world_from_enu_sin;

  // Sigma_w = M * [[a,b],[b,d]] * M^T, with M = [[c,s],[-s,c]].
  const double m00 = c * a + s * b;  // (M * Sigma) row 0
  const double m01 = c * b + s * d;
  const double m10 = -s * a + c * b;  // (M * Sigma) row 1
  const double m11 = -s * b + c * d;
  // (M * Sigma) * M^T, M^T = [[c,-s],[s,c]]. Symmetric for symmetric Sigma.
  double w00 = m00 * c + m01 * s;
  double w01 = m00 * (-s) + m01 * c;
  double w11 = m10 * (-s) + m11 * c;

  // Inflate (formal covariance is optimistic / time-correlated), then add an
  // isotropic noise floor to the horizontal diagonal (PSD-preserving).
  const double inflation_sq = inflation * inflation;
  w00 *= inflation_sq;
  w01 *= inflation_sq;
  w11 *= inflation_sq;
  const double floor_var = floor_sigma * floor_sigma;
  w00 += floor_var;
  w11 += floor_var;

  return {w00, w01, 0.0, w01, w11, 0.0, 0.0, 0.0, z_variance};
}

}  // namespace bagwiz::core::slam
