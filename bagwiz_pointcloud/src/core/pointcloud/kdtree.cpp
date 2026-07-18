// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/kdtree.hpp"

#include <algorithm>
#include <numeric>
#include <utility>
#include <vector>

namespace bagwiz::core::pointcloud
{

namespace
{

float dist_sq(const std::array<float, 3> & a, const std::array<float, 3> & b)
{
  const float dx = a[0] - b[0];
  const float dy = a[1] - b[1];
  const float dz = a[2] - b[2];
  return dx * dx + dy * dy + dz * dz;
}

// Lexicographic order on (distance, index): the tie-break by index is what makes
// query results reproducible for equidistant points.
bool entry_less(float dist_a, std::uint32_t index_a, float dist_b, std::uint32_t index_b)
{
  return dist_a < dist_b || (dist_a == dist_b && index_a < index_b);
}

// The knn output buffers double as a binary max-heap over parallel (distance,
// index) arrays: the heap top is the WORST of the current k-best entries.
void sift_down(
  std::vector<std::uint32_t> & indices, std::vector<float> & distances_sq, std::size_t root,
  std::size_t heap_size)
{
  while (true) {
    std::size_t largest = root;
    const std::size_t left = 2 * root + 1;
    const std::size_t right = 2 * root + 2;
    if (
      left < heap_size &&
      entry_less(distances_sq[largest], indices[largest], distances_sq[left], indices[left])) {
      largest = left;
    }
    if (
      right < heap_size &&
      entry_less(distances_sq[largest], indices[largest], distances_sq[right], indices[right])) {
      largest = right;
    }
    if (largest == root) {
      return;
    }
    std::swap(distances_sq[root], distances_sq[largest]);
    std::swap(indices[root], indices[largest]);
    root = largest;
  }
}

void heap_push(
  std::vector<std::uint32_t> & indices, std::vector<float> & distances_sq, std::uint32_t index,
  float dist)
{
  indices.push_back(index);
  distances_sq.push_back(dist);
  std::size_t child = indices.size() - 1;
  while (child > 0) {
    const std::size_t parent = (child - 1) / 2;
    if (!entry_less(distances_sq[parent], indices[parent], distances_sq[child], indices[child])) {
      break;
    }
    std::swap(distances_sq[parent], distances_sq[child]);
    std::swap(indices[parent], indices[child]);
    child = parent;
  }
}

// Replaces the current worst entry with a better one and restores the heap.
void heap_replace_top(
  std::vector<std::uint32_t> & indices, std::vector<float> & distances_sq, std::uint32_t index,
  float dist)
{
  indices[0] = index;
  distances_sq[0] = dist;
  sift_down(indices, distances_sq, 0, indices.size());
}

}  // namespace

KdTree::KdTree(std::span<const std::array<float, 3>> points) : points_(points)
{
  indices_.resize(points.size());
  std::iota(indices_.begin(), indices_.end(), std::uint32_t{0});
  axes_.assign(points.size(), 0);
  build(0, indices_.size());
}

void KdTree::build(std::size_t begin, std::size_t end)
{
  if (begin >= end) {
    return;
  }
  // Split on the longest axis of the subrange bounding box.
  std::array<float, 3> lo = points_[indices_[begin]];
  std::array<float, 3> hi = lo;
  for (std::size_t i = begin + 1; i < end; ++i) {
    const auto & p = points_[indices_[i]];
    for (std::size_t axis = 0; axis < 3; ++axis) {
      lo[axis] = std::min(lo[axis], p[axis]);
      hi[axis] = std::max(hi[axis], p[axis]);
    }
  }
  std::size_t axis = 0;
  if (hi[1] - lo[1] > hi[axis] - lo[axis]) {
    axis = 1;
  }
  if (hi[2] - lo[2] > hi[axis] - lo[axis]) {
    axis = 2;
  }
  const std::size_t node = begin + (end - begin) / 2;
  std::nth_element(
    indices_.begin() + static_cast<std::ptrdiff_t>(begin),
    indices_.begin() + static_cast<std::ptrdiff_t>(node),
    indices_.begin() + static_cast<std::ptrdiff_t>(end),
    [this, axis](std::uint32_t a, std::uint32_t b) {
      const float ca = points_[a][axis];
      const float cb = points_[b][axis];
      return ca < cb || (ca == cb && a < b);
    });
  axes_[node] = static_cast<std::uint8_t>(axis);
  build(begin, node);
  build(node + 1, end);
}

void KdTree::knn(
  const std::array<float, 3> & query, std::size_t k, std::vector<std::uint32_t> & indices,
  std::vector<float> & distances_sq) const
{
  indices.clear();
  distances_sq.clear();
  if (k == 0 || indices_.empty()) {
    return;
  }
  k = std::min(k, indices_.size());
  indices.reserve(k);
  distances_sq.reserve(k);
  knn_node(0, indices_.size(), indices_.size() / 2, query, k, indices, distances_sq);
  // The buffers hold a max-heap (worst on top); heapsort them into ascending
  // (distance, index) order so results come out nearest first.
  for (std::size_t heap_size = indices.size(); heap_size > 1; --heap_size) {
    std::swap(indices[0], indices[heap_size - 1]);
    std::swap(distances_sq[0], distances_sq[heap_size - 1]);
    sift_down(indices, distances_sq, 0, heap_size - 1);
  }
}

void KdTree::knn_node(
  std::size_t begin, std::size_t end, std::size_t node, const std::array<float, 3> & query,
  std::size_t k, std::vector<std::uint32_t> & indices, std::vector<float> & distances_sq) const
{
  const std::uint32_t index = indices_[node];
  const float dist = dist_sq(query, points_[index]);
  if (indices.size() < k) {
    heap_push(indices, distances_sq, index, dist);
  } else if (entry_less(dist, index, distances_sq.front(), indices.front())) {
    heap_replace_top(indices, distances_sq, index, dist);
  }

  const std::size_t axis = axes_[node];
  const float diff = query[axis] - points_[index][axis];
  const std::size_t near_begin = diff <= 0.0f ? begin : node + 1;
  const std::size_t near_end = diff <= 0.0f ? node : end;
  const std::size_t far_begin = diff <= 0.0f ? node + 1 : begin;
  const std::size_t far_end = diff <= 0.0f ? end : node;
  if (near_begin < near_end) {
    knn_node(
      near_begin, near_end, near_begin + (near_end - near_begin) / 2, query, k, indices,
      distances_sq);
  }
  // The far subtree can hold a better point only when the splitting plane is no
  // farther than the current worst distance (or the heap is not full yet). The
  // `<=` keeps points exactly on the plane, whose index may win the tie-break.
  if (far_begin < far_end && (indices.size() < k || diff * diff <= distances_sq.front())) {
    knn_node(
      far_begin, far_end, far_begin + (far_end - far_begin) / 2, query, k, indices, distances_sq);
  }
}

void KdTree::radius_search(
  const std::array<float, 3> & query, float radius, std::vector<std::uint32_t> & indices) const
{
  indices.clear();
  if (radius < 0.0f || indices_.empty()) {
    return;
  }
  radius_node(0, indices_.size(), indices_.size() / 2, query, radius * radius, indices);
}

void KdTree::radius_node(
  std::size_t begin, std::size_t end, std::size_t node, const std::array<float, 3> & query,
  float radius_sq, std::vector<std::uint32_t> & indices) const
{
  const std::uint32_t index = indices_[node];
  if (dist_sq(query, points_[index]) <= radius_sq) {
    indices.push_back(index);
  }
  const std::size_t axis = axes_[node];
  const float diff = query[axis] - points_[index][axis];
  const std::size_t near_begin = diff <= 0.0f ? begin : node + 1;
  const std::size_t near_end = diff <= 0.0f ? node : end;
  const std::size_t far_begin = diff <= 0.0f ? node + 1 : begin;
  const std::size_t far_end = diff <= 0.0f ? end : node;
  if (near_begin < near_end) {
    radius_node(
      near_begin, near_end, near_begin + (near_end - near_begin) / 2, query, radius_sq, indices);
  }
  if (far_begin < far_end && diff * diff <= radius_sq) {
    radius_node(
      far_begin, far_end, far_begin + (far_end - far_begin) / 2, query, radius_sq, indices);
  }
}

}  // namespace bagwiz::core::pointcloud
