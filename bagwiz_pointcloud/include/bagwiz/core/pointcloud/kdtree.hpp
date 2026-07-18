// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__POINTCLOUD__KDTREE_HPP_
#define BAGWIZ__CORE__POINTCLOUD__KDTREE_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace bagwiz::core::pointcloud
{

// A dependency-free 3D kd-tree over float points, built once and queried many
// times. The tree stores an index permutation into the caller's point array:
// points are referenced, NOT copied, so the referenced array must outlive the
// tree and must not be modified while the tree is in use.
//
// The build is a recursive median split (std::nth_element on the longest axis
// of each subrange), so the tree is balanced by construction and the recursion
// depth is O(log n). Queries are const, hold no mutable state, and are safe to
// run concurrently from multiple threads. All results are deterministic: ties
// in distance are broken by point index, so repeated runs reproduce exactly.
class KdTree
{
public:
  KdTree() = default;

  // Builds the tree over `points` (referenced, not copied — see class comment).
  explicit KdTree(std::span<const std::array<float, 3>> points);

  // Fills `indices` / `distances_sq` with the k nearest points to `query`,
  // nearest first (ties broken by point index). Fewer than k results are
  // returned when the tree holds fewer than k points. The caller-provided
  // buffers are cleared and reused; keeping them across calls avoids repeated
  // allocation.
  void knn(
    const std::array<float, 3> & query, std::size_t k, std::vector<std::uint32_t> & indices,
    std::vector<float> & distances_sq) const;

  // Fills `indices` with every point within `radius` of `query`, in no
  // particular order. A negative radius yields no results. The buffer is
  // cleared and reused as in knn().
  void radius_search(
    const std::array<float, 3> & query, float radius, std::vector<std::uint32_t> & indices) const;

  std::size_t size() const noexcept { return indices_.size(); }
  bool empty() const noexcept { return indices_.empty(); }

private:
  // Balanced-tree layout: node `node` covers the index-permutation subrange
  // [begin, end), stores point indices_[node], and its children cover
  // [begin, node) and [node + 1, end). axes_[node] is the split axis chosen at
  // build time for that node.
  void build(std::size_t begin, std::size_t end);
  void knn_node(
    std::size_t begin, std::size_t end, std::size_t node, const std::array<float, 3> & query,
    std::size_t k, std::vector<std::uint32_t> & indices, std::vector<float> & distances_sq) const;
  void radius_node(
    std::size_t begin, std::size_t end, std::size_t node, const std::array<float, 3> & query,
    float radius_sq, std::vector<std::uint32_t> & indices) const;

  std::span<const std::array<float, 3>> points_;
  std::vector<std::uint32_t> indices_;
  std::vector<std::uint8_t> axes_;
};

}  // namespace bagwiz::core::pointcloud

#endif  // BAGWIZ__CORE__POINTCLOUD__KDTREE_HPP_
