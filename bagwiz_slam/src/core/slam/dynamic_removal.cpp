// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/dynamic_removal.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <span>
#include <thread>
#include <unordered_map>
#include <vector>

namespace bagwiz::core::slam
{
namespace
{
// Same guard as VoxelGrid: never divide by a non-positive voxel size.
constexpr double kMinVoxelSize = 1e-3;

// Voxels are grouped into 16^3-voxel blocks so the free-space state costs bits,
// not hash-map entries: one block holds 4096 seen-free bits (64 words) plus the
// same again for the void mask, keyed by (voxel index >> 4) per axis.
constexpr int kBlockShift = 4;
constexpr std::int32_t kBlockMask = (1 << kBlockShift) - 1;
constexpr std::size_t kWordsPerBlock = (1U << (3 * kBlockShift)) / 64U;

// The block index is split into independent shards so concurrent integrate()
// calls only contend when two threads create/find blocks that hash to the same
// shard; setting a bit inside a block is a lock-free atomic OR.
constexpr std::size_t kShardCount = 64;

// Teschner et al. spatial-hash constants, as in cloud_filters.cpp. Unsigned so
// the multiply wraps (defined) rather than risking signed overflow (UB).
constexpr std::size_t kHashX = 73856093U;
constexpr std::size_t kHashY = 19349663U;
constexpr std::size_t kHashZ = 83492791U;

std::size_t mix(std::int32_t index, std::size_t multiplier)
{
  return static_cast<std::size_t>(static_cast<std::uint32_t>(index)) * multiplier;
}

struct VoxelIndex
{
  std::int32_t x;
  std::int32_t y;
  std::int32_t z;
  bool operator==(const VoxelIndex & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
  bool operator!=(const VoxelIndex & other) const { return !(*this == other); }
};

struct BlockKey
{
  std::int32_t x;
  std::int32_t y;
  std::int32_t z;
  bool operator==(const BlockKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
  bool operator!=(const BlockKey & other) const { return !(*this == other); }
};

struct BlockKeyHash
{
  std::size_t operator()(const BlockKey & key) const
  {
    return mix(key.x, kHashX) ^ mix(key.y, kHashY) ^ mix(key.z, kHashZ);
  }
};

// >> and & work as floor-division/modulo for negative indices in two's
// complement, so the voxel->block mapping is continuous across zero.
BlockKey block_key_of(const VoxelIndex & voxel)
{
  return {voxel.x >> kBlockShift, voxel.y >> kBlockShift, voxel.z >> kBlockShift};
}

std::size_t bit_index_of(const VoxelIndex & voxel)
{
  const auto lx = static_cast<std::size_t>(voxel.x & kBlockMask);
  const auto ly = static_cast<std::size_t>(voxel.y & kBlockMask);
  const auto lz = static_cast<std::size_t>(voxel.z & kBlockMask);
  return lx + ((ly + (lz << kBlockShift)) << kBlockShift);
}

struct Block
{
  // seen_free is written concurrently by integrate() (atomic OR); is_void is
  // written only by finalize(), each block by exactly one worker.
  std::array<std::atomic<std::uint64_t>, kWordsPerBlock> seen_free{};
  std::array<std::uint64_t, kWordsPerBlock> is_void{};

  [[nodiscard]] bool seen_free_bit(std::size_t bit) const
  {
    return (seen_free[bit >> 6].load(std::memory_order_relaxed) & (1ULL << (bit & 63U))) != 0U;
  }
  [[nodiscard]] bool void_bit(std::size_t bit) const
  {
    return (is_void[bit >> 6] & (1ULL << (bit & 63U))) != 0U;
  }
};

struct Shard
{
  std::mutex mutex;
  std::unordered_map<BlockKey, std::unique_ptr<Block>, BlockKeyHash> blocks;
};

}  // namespace

struct VoidRegionClassifier::Impl
{
  const double voxel_size;
  const double inv_voxel_size;
  const double sensor_offset;
  const int neighborhood;
  const double max_ray_length;

  std::array<Shard, kShardCount> shards;

  // Read-only block index assembled by finalize(); classify()/is_void() use it
  // lock-free afterwards.
  std::unordered_map<BlockKey, Block *, BlockKeyHash> index;
  bool finalized = false;

  explicit Impl(const VoidRegionConfig & config)
  : voxel_size(config.voxel_size > kMinVoxelSize ? config.voxel_size : kMinVoxelSize),
    inv_voxel_size(1.0 / voxel_size),
    sensor_offset(config.sensor_offset),
    neighborhood(config.neighborhood > 0 ? config.neighborhood : 0),
    max_ray_length(config.max_ray_length)
  {
  }

  [[nodiscard]] VoxelIndex voxel_of(double x, double y, double z) const
  {
    return {
      static_cast<std::int32_t>(std::floor(x * inv_voxel_size)),
      static_cast<std::int32_t>(std::floor(y * inv_voxel_size)),
      static_cast<std::int32_t>(std::floor(z * inv_voxel_size))};
  }

  Block * find_or_create(const BlockKey & key)
  {
    Shard & shard = shards[BlockKeyHash{}(key) % kShardCount];
    const std::lock_guard<std::mutex> lock(shard.mutex);
    auto & slot = shard.blocks[key];
    if (!slot) {
      slot = std::make_unique<Block>();
    }
    return slot.get();
  }

  // Lookup without creating. Used by the quiescent-state queries; after
  // finalize() the flat index is used instead.
  [[nodiscard]] const Block * find(const BlockKey & key) const
  {
    if (finalized) {
      const auto found = index.find(key);
      return found == index.end() ? nullptr : found->second;
    }
    const Shard & shard = shards[BlockKeyHash{}(key) % kShardCount];
    const auto found = shard.blocks.find(key);
    return found == shard.blocks.end() ? nullptr : found->second.get();
  }

  [[nodiscard]] bool seen_free_voxel(const VoxelIndex & voxel) const
  {
    const Block * block = find(block_key_of(voxel));
    return block != nullptr && block->seen_free_bit(bit_index_of(voxel));
  }

  // Amanatides-Woo DDA: mark every voxel the segment [origin, origin + t_end*d]
  // passes through, never marking the hit point's own voxel.
  void carve_ray(
    const std::array<double, 3> & origin, const std::array<float, 3> & hit, BlockKey & cached_key,
    Block *& cached_block)
  {
    if (!std::isfinite(hit[0]) || !std::isfinite(hit[1]) || !std::isfinite(hit[2])) {
      return;
    }
    const double dx = static_cast<double>(hit[0]) - origin[0];
    const double dy = static_cast<double>(hit[1]) - origin[1];
    const double dz = static_cast<double>(hit[2]) - origin[2];
    const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double t_end = std::min(length - sensor_offset, max_ray_length);
    if (!(length > 0.0) || !(t_end > 0.0)) {
      return;
    }
    const double ux = dx / length;
    const double uy = dy / length;
    const double uz = dz / length;

    VoxelIndex voxel = voxel_of(origin[0], origin[1], origin[2]);
    const VoxelIndex hit_voxel = voxel_of(hit[0], hit[1], hit[2]);

    // Per axis: the step direction, the ray parameter t [m] at which the ray
    // crosses the next voxel boundary on that axis, and t per voxel width.
    std::array<std::int32_t, 3> step{};
    std::array<double, 3> t_max{};
    std::array<double, 3> t_delta{};
    const std::array<double, 3> unit{ux, uy, uz};
    const std::array<std::int32_t, 3> start{voxel.x, voxel.y, voxel.z};
    for (int axis = 0; axis < 3; ++axis) {
      if (unit[axis] > 0.0) {
        step[axis] = 1;
        const double boundary = (static_cast<double>(start[axis]) + 1.0) * voxel_size;
        t_max[axis] = (boundary - origin[axis]) / unit[axis];
        t_delta[axis] = voxel_size / unit[axis];
      } else if (unit[axis] < 0.0) {
        step[axis] = -1;
        const double boundary = static_cast<double>(start[axis]) * voxel_size;
        t_max[axis] = (boundary - origin[axis]) / unit[axis];
        t_delta[axis] = -voxel_size / unit[axis];
      } else {
        step[axis] = 0;
        t_max[axis] = std::numeric_limits<double>::infinity();
        t_delta[axis] = std::numeric_limits<double>::infinity();
      }
    }

    while (true) {
      if (voxel == hit_voxel) {
        return;  // the hit surface's voxel is never freed by its own ray
      }
      mark_seen_free(voxel, cached_key, cached_block);

      const int axis = (t_max[0] <= t_max[1] && t_max[0] <= t_max[2]) ? 0
                       : (t_max[1] <= t_max[2])                       ? 1
                                                                      : 2;
      if (t_max[axis] > t_end) {
        return;  // the segment ends inside the current voxel
      }
      t_max[axis] += t_delta[axis];
      if (axis == 0) {
        voxel.x += step[0];
      } else if (axis == 1) {
        voxel.y += step[1];
      } else {
        voxel.z += step[2];
      }
    }
  }

  void mark_seen_free(const VoxelIndex & voxel, BlockKey & cached_key, Block *& cached_block)
  {
    const BlockKey key = block_key_of(voxel);
    if (cached_block == nullptr || key != cached_key) {
      cached_block = find_or_create(key);
      cached_key = key;
    }
    const std::size_t bit = bit_index_of(voxel);
    cached_block->seen_free[bit >> 6].fetch_or(1ULL << (bit & 63U), std::memory_order_relaxed);
  }

  // A voxel is void when it and every voxel within Chebyshev distance
  // `neighborhood` were seen free. This is the general path, resolving every
  // neighbor through the block index; erode_block short-circuits to direct bit
  // tests when the whole neighborhood lies inside the block it already holds.
  [[nodiscard]] bool erodes_to_void(const VoxelIndex & voxel) const
  {
    for (std::int32_t ox = -neighborhood; ox <= neighborhood; ++ox) {
      for (std::int32_t oy = -neighborhood; oy <= neighborhood; ++oy) {
        for (std::int32_t oz = -neighborhood; oz <= neighborhood; ++oz) {
          if (!seen_free_voxel({voxel.x + ox, voxel.y + oy, voxel.z + oz})) {
            return false;
          }
        }
      }
    }
    return true;
  }

  // erodes_to_void for a voxel whose whole neighborhood lies inside `block`:
  // pure in-block bit tests, no hash lookups. (lx, ly, lz) are the voxel's
  // local coordinates; the caller guarantees every offset stays in [0, 16).
  [[nodiscard]] static bool interior_neighborhood_free(
    const Block & block, int lx, int ly, int lz, int radius)
  {
    for (int ox = -radius; ox <= radius; ++ox) {
      for (int oy = -radius; oy <= radius; ++oy) {
        for (int oz = -radius; oz <= radius; ++oz) {
          const std::size_t bit = static_cast<std::size_t>(lx + ox) +
                                  ((static_cast<std::size_t>(ly + oy) +
                                    (static_cast<std::size_t>(lz + oz) << kBlockShift))
                                   << kBlockShift);
          if (!block.seen_free_bit(bit)) {
            return false;
          }
        }
      }
    }
    return true;
  }

  void erode_block(const BlockKey & key, Block & block) const
  {
    const std::int32_t base_x = key.x << kBlockShift;
    const std::int32_t base_y = key.y << kBlockShift;
    const std::int32_t base_z = key.z << kBlockShift;
    for (std::size_t word = 0; word < kWordsPerBlock; ++word) {
      std::uint64_t bits = block.seen_free[word].load(std::memory_order_relaxed);
      if (neighborhood == 0) {
        block.is_void[word] = bits;
        continue;
      }
      std::uint64_t void_bits = 0U;
      while (bits != 0U) {
        const int bit = std::countr_zero(bits);
        bits &= bits - 1U;
        const int local = static_cast<int>(word) * 64 + bit;
        const int lx = local & kBlockMask;
        const int ly = (local >> kBlockShift) & kBlockMask;
        const int lz = local >> (2 * kBlockShift);
        // Interior voxels (the vast majority) resolve entirely inside this
        // block; only the boundary shell pays the cross-block lookups.
        const bool interior = lx >= neighborhood && lx + neighborhood <= kBlockMask &&
                              ly >= neighborhood && ly + neighborhood <= kBlockMask &&
                              lz >= neighborhood && lz + neighborhood <= kBlockMask;
        const bool voxel_is_void = interior
                                     ? interior_neighborhood_free(block, lx, ly, lz, neighborhood)
                                     : erodes_to_void({base_x + lx, base_y + ly, base_z + lz});
        if (voxel_is_void) {
          void_bits |= 1ULL << bit;
        }
      }
      block.is_void[word] = void_bits;
    }
  }
};

VoidRegionClassifier::VoidRegionClassifier(const VoidRegionConfig & config)
: impl_(std::make_unique<Impl>(config))
{
}

VoidRegionClassifier::~VoidRegionClassifier() = default;

void VoidRegionClassifier::integrate(
  std::span<const std::array<float, 3>> world_points, const std::array<double, 3> & sensor_origin)
{
  assert(!impl_->finalized);
  BlockKey cached_key{0, 0, 0};
  Block * cached_block = nullptr;
  for (const auto & point : world_points) {
    impl_->carve_ray(sensor_origin, point, cached_key, cached_block);
  }
}

void VoidRegionClassifier::finalize(int num_threads)
{
  assert(!impl_->finalized);
  // Assemble the flat read-only index and the erosion work list.
  std::vector<std::pair<BlockKey, Block *>> blocks;
  for (const Shard & shard : impl_->shards) {
    for (const auto & entry : shard.blocks) {
      blocks.emplace_back(entry.first, entry.second.get());
    }
  }
  impl_->index.reserve(blocks.size());
  for (const auto & entry : blocks) {
    impl_->index.emplace(entry.first, entry.second);
  }

  // Erode in parallel: each worker owns a disjoint chunk of blocks and writes
  // only its own blocks' void masks, reading the (now immutable) seen-free
  // bits, so the result is independent of the thread count.
  const std::size_t worker_count =
    std::min(blocks.size(), static_cast<std::size_t>(num_threads > 0 ? num_threads : 1));
  if (worker_count <= 1) {
    for (auto & entry : blocks) {
      impl_->erode_block(entry.first, *entry.second);
    }
  } else {
    std::vector<std::thread> workers;
    workers.reserve(worker_count);
    const std::size_t chunk = (blocks.size() + worker_count - 1) / worker_count;
    for (std::size_t w = 0; w < worker_count; ++w) {
      const std::size_t begin = w * chunk;
      const std::size_t end = std::min(begin + chunk, blocks.size());
      workers.emplace_back([this, &blocks, begin, end]() {
        for (std::size_t i = begin; i < end; ++i) {
          impl_->erode_block(blocks[i].first, *blocks[i].second);
        }
      });
    }
    for (auto & worker : workers) {
      worker.join();
    }
  }
  impl_->finalized = true;
}

std::size_t VoidRegionClassifier::classify(
  std::span<const std::array<float, 3>> world_points, std::span<std::uint8_t> keep) const
{
  assert(impl_->finalized);
  assert(keep.size() >= world_points.size());
  std::size_t dynamic_count = 0;
  BlockKey cached_key{0, 0, 0};
  const Block * cached_block = nullptr;
  bool cache_valid = false;
  for (std::size_t i = 0; i < world_points.size(); ++i) {
    const auto & point = world_points[i];
    const VoxelIndex voxel = impl_->voxel_of(point[0], point[1], point[2]);
    const BlockKey key = block_key_of(voxel);
    if (!cache_valid || key != cached_key) {
      cached_block = impl_->find(key);
      cached_key = key;
      cache_valid = true;
    }
    const bool is_dynamic = cached_block != nullptr && cached_block->void_bit(bit_index_of(voxel));
    keep[i] = is_dynamic ? 0U : 1U;
    dynamic_count += is_dynamic ? 1U : 0U;
  }
  return dynamic_count;
}

bool VoidRegionClassifier::seen_free(float x, float y, float z) const
{
  return impl_->seen_free_voxel(impl_->voxel_of(x, y, z));
}

bool VoidRegionClassifier::is_void(float x, float y, float z) const
{
  assert(impl_->finalized);
  const VoxelIndex voxel = impl_->voxel_of(x, y, z);
  const Block * block = impl_->find(block_key_of(voxel));
  return block != nullptr && block->void_bit(bit_index_of(voxel));
}

std::size_t VoidRegionClassifier::seen_free_voxel_count() const
{
  std::size_t count = 0;
  for (const Shard & shard : impl_->shards) {
    for (const auto & entry : shard.blocks) {
      for (const auto & word : entry.second->seen_free) {
        count += static_cast<std::size_t>(std::popcount(word.load(std::memory_order_relaxed)));
      }
    }
  }
  return count;
}

}  // namespace bagwiz::core::slam
