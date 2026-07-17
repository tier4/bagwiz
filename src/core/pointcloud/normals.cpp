// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/normals.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <thread>
#include <vector>

namespace bagwiz::core::pointcloud
{

namespace
{

// Total-variance threshold (trace of the covariance, averaged per point) below
// which a neighborhood is treated as coincident points and yields no normal.
// At ~1e-6 units of coordinate spread this only fires for truly degenerate
// neighborhoods, independent of the neighbor count.
constexpr double kDegenerateTotalVariance = 1e-12;

// Jacobi eigenvalue iteration for a symmetric 3x3 matrix `a` (row-major). On
// return `d` holds the eigenvalues and the columns of `v` (v[row * 3 + col])
// the corresponding eigenvectors. Written by hand because this translation
// unit must stay free of linear-algebra dependencies.
void jacobi_eigen_3x3(const double a[9], double d[3], double v[9])
{
  double m[9];
  std::copy(a, a + 9, m);
  v[0] = 1.0;
  v[1] = 0.0;
  v[2] = 0.0;
  v[3] = 0.0;
  v[4] = 1.0;
  v[5] = 0.0;
  v[6] = 0.0;
  v[7] = 0.0;
  v[8] = 1.0;
  constexpr int kMaxSweeps = 32;  // 3 rotations per sweep; 3x3 converges in a few
  for (int sweep = 0; sweep < kMaxSweeps; ++sweep) {
    if (std::abs(m[1]) + std::abs(m[2]) + std::abs(m[4]) == 0.0) {
      break;  // already diagonal
    }
    constexpr std::size_t pairs[3][2] = {{0, 1}, {0, 2}, {1, 2}};
    for (std::size_t i = 0; i < 3; ++i) {
      const std::size_t p = pairs[i][0];
      const std::size_t q = pairs[i][1];
      const double apq = m[p * 3 + q];
      if (apq == 0.0) {
        continue;
      }
      // tan of the rotation angle that annihilates m[p][q].
      const double theta = (m[q * 3 + q] - m[p * 3 + p]) / (2.0 * apq);
      const double t =
        (theta >= 0.0 ? 1.0 : -1.0) / (std::abs(theta) + std::sqrt(theta * theta + 1.0));
      const double c = 1.0 / std::sqrt(t * t + 1.0);
      const double s = t * c;
      // m = J^T * m * J and v = v * J for the (p, q) plane rotation J.
      for (std::size_t k = 0; k < 3; ++k) {
        const double mkp = m[k * 3 + p];
        const double mkq = m[k * 3 + q];
        m[k * 3 + p] = c * mkp - s * mkq;
        m[k * 3 + q] = s * mkp + c * mkq;
      }
      for (std::size_t k = 0; k < 3; ++k) {
        const double mpk = m[p * 3 + k];
        const double mqk = m[q * 3 + k];
        m[p * 3 + k] = c * mpk - s * mqk;
        m[q * 3 + k] = s * mpk + c * mqk;
      }
      for (std::size_t k = 0; k < 3; ++k) {
        const double vkp = v[k * 3 + p];
        const double vkq = v[k * 3 + q];
        v[k * 3 + p] = c * vkp - s * vkq;
        v[k * 3 + q] = s * vkp + c * vkq;
      }
    }
  }
  d[0] = m[0];
  d[1] = m[4];
  d[2] = m[8];
}

}  // namespace

LocalGeometry estimate_local_geometry(
  std::span<const std::array<float, 3>> points, const KdTree & tree, int k_neighbors,
  int num_threads)
{
  LocalGeometry result;
  const std::size_t n = points.size();
  result.normals.assign(n, {0.0f, 0.0f, 0.0f});
  result.spacings.assign(n, 0.0f);
  if (n == 0 || k_neighbors <= 0 || tree.empty()) {
    return result;
  }
  // One extra candidate because the query point itself is returned by knn.
  const std::size_t query_k = std::min(static_cast<std::size_t>(k_neighbors) + 1, tree.size());
  const std::size_t k = static_cast<std::size_t>(k_neighbors);

  const auto estimate_chunk = [&](std::size_t begin, std::size_t end) {
    std::vector<std::uint32_t> knn_indices;
    std::vector<float> knn_dists;
    std::vector<std::uint32_t> neighbors;
    for (std::size_t i = begin; i < end; ++i) {
      tree.knn(points[i], query_k, knn_indices, knn_dists);
      neighbors.clear();
      double dist_sum = 0.0;
      for (std::size_t j = 0; j < knn_indices.size() && neighbors.size() < k; ++j) {
        if (knn_indices[j] == i) {
          continue;  // skip self
        }
        neighbors.push_back(knn_indices[j]);
        dist_sum += std::sqrt(static_cast<double>(knn_dists[j]));
      }
      if (neighbors.empty()) {
        continue;
      }
      result.spacings[i] = static_cast<float>(dist_sum / static_cast<double>(neighbors.size()));
      if (neighbors.size() < 3) {
        continue;
      }
      // Covariance of the query point plus its kept neighbors, in double.
      double centroid[3] = {0.0, 0.0, 0.0};
      const auto accumulate_centroid = [&centroid](const std::array<float, 3> & p) {
        centroid[0] += p[0];
        centroid[1] += p[1];
        centroid[2] += p[2];
      };
      accumulate_centroid(points[i]);
      for (const std::uint32_t index : neighbors) {
        accumulate_centroid(points[index]);
      }
      const double count = static_cast<double>(neighbors.size() + 1);
      centroid[0] /= count;
      centroid[1] /= count;
      centroid[2] /= count;
      double cov[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      const auto accumulate_covariance = [&centroid, &cov](const std::array<float, 3> & p) {
        const double dx = p[0] - centroid[0];
        const double dy = p[1] - centroid[1];
        const double dz = p[2] - centroid[2];
        cov[0] += dx * dx;
        cov[1] += dx * dy;
        cov[2] += dx * dz;
        cov[4] += dy * dy;
        cov[5] += dy * dz;
        cov[8] += dz * dz;
      };
      accumulate_covariance(points[i]);
      for (const std::uint32_t index : neighbors) {
        accumulate_covariance(points[index]);
      }
      cov[3] = cov[1];
      cov[6] = cov[2];
      cov[7] = cov[5];
      for (double & entry : cov) {
        entry /= count;
      }
      if (cov[0] + cov[4] + cov[8] <= kDegenerateTotalVariance) {
        continue;
      }
      double eigenvalues[3];
      double eigenvectors[9];
      jacobi_eigen_3x3(cov, eigenvalues, eigenvectors);
      std::size_t smallest = 0;
      if (eigenvalues[1] < eigenvalues[smallest]) {
        smallest = 1;
      }
      if (eigenvalues[2] < eigenvalues[smallest]) {
        smallest = 2;
      }
      const double nx = eigenvectors[smallest];
      const double ny = eigenvectors[3 + smallest];
      const double nz = eigenvectors[6 + smallest];
      const double norm = std::sqrt(nx * nx + ny * ny + nz * nz);
      if (norm > 0.0) {
        result.normals[i] = {
          static_cast<float>(nx / norm), static_cast<float>(ny / norm),
          static_cast<float>(nz / norm)};
      }
    }
  };

  const int threads = std::max(1, num_threads);
  if (threads == 1) {
    estimate_chunk(0, n);
  } else {
    std::vector<std::thread> workers;
    workers.reserve(static_cast<std::size_t>(threads));
    const std::size_t chunk =
      (n + static_cast<std::size_t>(threads) - 1) / static_cast<std::size_t>(threads);
    for (int t = 0; t < threads; ++t) {
      const std::size_t begin = std::min(n, static_cast<std::size_t>(t) * chunk);
      const std::size_t end = std::min(n, begin + chunk);
      workers.emplace_back(estimate_chunk, begin, end);
    }
    for (auto & w : workers) {
      w.join();
    }
  }
  return result;
}

}  // namespace bagwiz::core::pointcloud
