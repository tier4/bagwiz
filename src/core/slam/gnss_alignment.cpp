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

// Closed-form 2-D Procrustes (the planar special case of glim_ext's SVD-based
// Umeyama): for centered point sets the optimal proper rotation mapping `est`
// onto `gnss` has angle atan2(Syx - Sxy, Sxx + Syy), where S = Σ (gnss_c)(est_c)^T
// over the x-y components. This always yields a proper rotation (no reflection
// case to handle) and needs no matrix library. The resulting T_gnss_world maps
// est -> gnss; its inverse maps each GNSS position into the world frame.
std::vector<std::array<double, 3>> align_gnss_to_world(
  const std::vector<std::array<double, 3>> & est, const std::vector<std::array<double, 3>> & gnss)
{
  std::vector<std::array<double, 3>> world;
  if (est.empty() || est.size() != gnss.size()) {
    return world;
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
  const double c = std::cos(theta);
  const double s = std::sin(theta);

  // T_gnss_world (2-D): gnss.xy ~= R * est.xy + t, with R = [[c,-s],[s,c]].
  const double tx = mean_gnss[0] - (c * mean_est[0] - s * mean_est[1]);
  const double ty = mean_gnss[1] - (s * mean_est[0] + c * mean_est[1]);
  // z is not rotated; its translation mirrors glim_ext (T.translation().z()).
  const double dz = mean_gnss[2] - mean_est[2];

  // world = T_world_gnss * gnss = R^T * (gnss.xy - t); world.z = gnss.z - dz.
  world.reserve(est.size());
  for (std::size_t i = 0; i < gnss.size(); ++i) {
    const double qx = gnss[i][0] - tx;
    const double qy = gnss[i][1] - ty;
    const double wx = c * qx + s * qy;
    const double wy = -s * qx + c * qy;
    const double wz = gnss[i][2] - dz;
    world.push_back({wx, wy, wz});
  }
  return world;
}

}  // namespace bagwiz::core::slam
