// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/dynamic_removal.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <thread>
#include <utility>
#include <vector>

// GLIM-free unit tests for the DUFOMap-style void-region classifier behind
// `map slam --remove-dynamic`. All scenes use a 1 m voxel so voxel indices read
// directly off the coordinates.
namespace
{
namespace slam = bagwiz::core::slam;

using Point = std::array<float, 3>;
using Origin = std::array<double, 3>;

slam::VoidRegionConfig make_config(
  double voxel_size, double sensor_offset, int neighborhood, double max_ray_length = 100.0)
{
  slam::VoidRegionConfig config;
  config.voxel_size = voxel_size;
  config.sensor_offset = sensor_offset;
  config.neighborhood = neighborhood;
  config.max_ray_length = max_ray_length;
  return config;
}

// Integrate one ray that marks exactly the voxel containing (x+0.5, y+0.5,
// z+0.5) at 1 m resolution: the ray starts at that voxel's center and hits the
// center of the +x neighbor, and the hit voxel is never marked by its own ray.
void mark_voxel(slam::VoidRegionClassifier & classifier, int x, int y, int z)
{
  const Origin origin{x + 0.5, y + 0.5, z + 0.5};
  const std::vector<Point> hit{
    {static_cast<float>(x) + 1.5F, static_cast<float>(y) + 0.5F, static_cast<float>(z) + 0.5F}};
  classifier.integrate(hit, origin);
}

TEST(VoidRegionClassifier, MarksTraversedVoxelsAndBacksOffTheSensorOffset)
{
  slam::VoidRegionClassifier classifier(make_config(1.0, 0.6, 0));
  const Origin origin{0.5, 0.5, 0.5};
  const std::vector<Point> scan{{5.5F, 0.5F, 0.5F}};
  classifier.integrate(scan, origin);

  // Ray length 5.0, stops 0.6 short -> x in [0.5, 4.9]: voxels 0..4 are free.
  for (int x = 0; x <= 4; ++x) {
    EXPECT_TRUE(classifier.seen_free(x + 0.5F, 0.5F, 0.5F)) << "voxel x=" << x;
  }
  // The hit voxel is not free, nor is anything past it or beside the ray.
  EXPECT_FALSE(classifier.seen_free(5.5F, 0.5F, 0.5F));
  EXPECT_FALSE(classifier.seen_free(6.5F, 0.5F, 0.5F));
  EXPECT_FALSE(classifier.seen_free(2.5F, 1.5F, 0.5F));
  EXPECT_FALSE(classifier.seen_free(2.5F, 0.5F, 1.5F));
  EXPECT_EQ(classifier.seen_free_voxel_count(), 5U);
}

TEST(VoidRegionClassifier, ALargerSensorOffsetStopsTheMarkingEarlier)
{
  slam::VoidRegionClassifier classifier(make_config(1.0, 1.6, 0));
  const Origin origin{0.5, 0.5, 0.5};
  const std::vector<Point> scan{{5.5F, 0.5F, 0.5F}};
  classifier.integrate(scan, origin);

  // Ray stops at x = 0.5 + (5.0 - 1.6) = 3.9: voxel 4 is no longer entered.
  EXPECT_TRUE(classifier.seen_free(3.5F, 0.5F, 0.5F));
  EXPECT_FALSE(classifier.seen_free(4.5F, 0.5F, 0.5F));
}

TEST(VoidRegionClassifier, NeverMarksTheHitVoxelEvenWithZeroOffset)
{
  slam::VoidRegionClassifier classifier(make_config(1.0, 0.0, 0));
  const Origin origin{0.5, 0.5, 0.5};
  const std::vector<Point> scan{{5.5F, 0.5F, 0.5F}};
  classifier.integrate(scan, origin);

  EXPECT_TRUE(classifier.seen_free(4.5F, 0.5F, 0.5F));
  EXPECT_FALSE(classifier.seen_free(5.5F, 0.5F, 0.5F));
}

TEST(VoidRegionClassifier, TraversesNegativeAndDiagonalDirections)
{
  slam::VoidRegionClassifier classifier(make_config(1.0, 0.5, 0));
  const Origin origin{0.2, 0.3, 0.4};
  const Point hit{-6.3F, 4.1F, -2.7F};
  const std::vector<Point> scan{hit};
  classifier.integrate(scan, origin);

  // Every sampled position on the shortened segment must lie in a free voxel.
  const double dx = hit[0] - origin[0];
  const double dy = hit[1] - origin[1];
  const double dz = hit[2] - origin[2];
  const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
  const double t_end = (length - 0.5 - 0.05) / length;  // small margin inside the cut
  for (int i = 0; i <= 20; ++i) {
    const double t = t_end * static_cast<double>(i) / 20.0;
    EXPECT_TRUE(classifier.seen_free(
      static_cast<float>(origin[0] + t * dx), static_cast<float>(origin[1] + t * dy),
      static_cast<float>(origin[2] + t * dz)))
      << "sample " << i;
  }
  EXPECT_FALSE(classifier.seen_free(hit[0], hit[1], hit[2]));
}

TEST(VoidRegionClassifier, ClampsTheTraversalAtMaxRayLength)
{
  slam::VoidRegionClassifier classifier(make_config(1.0, 0.0, 0, 10.2));
  const Origin origin{0.5, 0.5, 0.5};
  const std::vector<Point> scan{{50.5F, 0.5F, 0.5F}};
  classifier.integrate(scan, origin);

  // Traversal ends at x = 0.5 + 10.2 = 10.7: voxel 10 is entered, 11 is not.
  EXPECT_TRUE(classifier.seen_free(10.5F, 0.5F, 0.5F));
  EXPECT_FALSE(classifier.seen_free(11.5F, 0.5F, 0.5F));
  EXPECT_FALSE(classifier.seen_free(49.5F, 0.5F, 0.5F));
}

TEST(VoidRegionClassifier, SkipsDegenerateAndNonFiniteRays)
{
  slam::VoidRegionClassifier classifier(make_config(1.0, 0.5, 0));
  const Origin origin{0.5, 0.5, 0.5};
  const std::vector<Point> scan{
    {0.5F, 0.5F, 0.5F},                                     // zero-length ray
    {0.8F, 0.5F, 0.5F},                                     // shorter than d_s
    {std::numeric_limits<float>::quiet_NaN(), 0.5F, 0.5F},  // non-finite
    {std::numeric_limits<float>::infinity(), 0.5F, 0.5F},   // non-finite
  };
  classifier.integrate(scan, origin);

  EXPECT_EQ(classifier.seen_free_voxel_count(), 0U);
}

TEST(VoidRegionClassifier, VoidEqualsSeenFreeWithoutErosion)
{
  slam::VoidRegionClassifier classifier(make_config(1.0, 0.0, 0));
  mark_voxel(classifier, 3, 3, 3);
  classifier.finalize(1);

  EXPECT_TRUE(classifier.is_void(3.5F, 3.5F, 3.5F));
  EXPECT_FALSE(classifier.is_void(4.5F, 3.5F, 3.5F));
}

TEST(VoidRegionClassifier, ErosionRequiresTheFullChebyshevNeighborhood)
{
  // 26 of the 27 voxels around the center are free: the center is NOT void.
  slam::VoidRegionClassifier incomplete(make_config(1.0, 0.0, 1));
  for (int x = -1; x <= 1; ++x) {
    for (int y = -1; y <= 1; ++y) {
      for (int z = -1; z <= 1; ++z) {
        if (x == 1 && y == 1 && z == 1) {
          continue;  // leave one corner unobserved
        }
        mark_voxel(incomplete, x, y, z);
      }
    }
  }
  incomplete.finalize(1);
  EXPECT_TRUE(incomplete.seen_free(0.5F, 0.5F, 0.5F));
  EXPECT_FALSE(incomplete.is_void(0.5F, 0.5F, 0.5F));

  // All 27 free: the center IS void.
  slam::VoidRegionClassifier complete(make_config(1.0, 0.0, 1));
  for (int x = -1; x <= 1; ++x) {
    for (int y = -1; y <= 1; ++y) {
      for (int z = -1; z <= 1; ++z) {
        mark_voxel(complete, x, y, z);
      }
    }
  }
  complete.finalize(1);
  EXPECT_TRUE(complete.is_void(0.5F, 0.5F, 0.5F));
}

TEST(VoidRegionClassifier, ErosionLooksAcrossBlockBoundaries)
{
  // Voxels 15 and 16 fall in different 16^3 blocks; the neighborhood of voxel
  // (16, 0, 0) spans both, so this catches a block-local-only erosion.
  slam::VoidRegionClassifier classifier(make_config(1.0, 0.0, 1));
  for (int x = 15; x <= 17; ++x) {
    for (int y = -1; y <= 1; ++y) {
      for (int z = -1; z <= 1; ++z) {
        mark_voxel(classifier, x, y, z);
      }
    }
  }
  classifier.finalize(1);
  EXPECT_TRUE(classifier.is_void(16.5F, 0.5F, 0.5F));

  // The negative-side counterpart: voxel (-16, 0, 0) sits at the low edge of
  // block -1, its x-1 neighbor (-17) in block -2 — catches sign errors in the
  // floor-division block mapping.
  slam::VoidRegionClassifier negative(make_config(1.0, 0.0, 1));
  for (int x = -17; x <= -15; ++x) {
    for (int y = -1; y <= 1; ++y) {
      for (int z = -1; z <= 1; ++z) {
        mark_voxel(negative, x, y, z);
      }
    }
  }
  negative.finalize(1);
  EXPECT_TRUE(negative.is_void(-15.5F, 0.5F, 0.5F));
}

TEST(VoidRegionClassifier, RemovesATransientBlobAndKeepsTheWall)
{
  // A sensor stares at a wall; a blob sits in front of it for the first 5 of
  // 20 scans, then leaves. The rays of the later scans traverse the blob's
  // voxels, so its points are classified dynamic; the wall stays.
  slam::VoidRegionClassifier classifier(make_config(1.0, 0.2, 0));
  const Origin origin{0.5, 5.5, 0.5};

  std::vector<Point> wall;
  for (int y = 0; y <= 10; ++y) {
    for (int z = 0; z <= 2; ++z) {
      wall.push_back({10.5F, y + 0.5F, z + 0.5F});
    }
  }
  const std::vector<Point> blob{{5.5F, 5.5F, 0.5F}, {5.6F, 5.4F, 0.6F}};

  for (int scan = 0; scan < 20; ++scan) {
    const bool blob_present = scan < 5;
    std::vector<Point> points;
    for (const Point & p : wall) {
      if (blob_present && p[1] == 5.5F && p[2] == 0.5F) {
        continue;  // the wall point straight behind the blob is occluded
      }
      points.push_back(p);
    }
    if (blob_present) {
      points.insert(points.end(), blob.begin(), blob.end());
    }
    classifier.integrate(points, origin);
  }
  classifier.finalize(1);

  std::vector<std::uint8_t> keep(blob.size(), 2U);
  EXPECT_EQ(classifier.classify(blob, keep), blob.size());
  for (std::size_t i = 0; i < blob.size(); ++i) {
    EXPECT_EQ(keep[i], 0U) << "blob point " << i;
  }

  keep.assign(wall.size(), 2U);
  EXPECT_EQ(classifier.classify(wall, keep), 0U);
  for (std::size_t i = 0; i < wall.size(); ++i) {
    EXPECT_EQ(keep[i], 1U) << "wall point " << i;
  }
}

TEST(VoidRegionClassifier, KeepsPointsInUnobservedVoxels)
{
  slam::VoidRegionClassifier classifier(make_config(1.0, 0.0, 0));
  mark_voxel(classifier, 0, 0, 0);
  classifier.finalize(1);

  const std::vector<Point> unobserved{{100.5F, 100.5F, 100.5F}};
  std::vector<std::uint8_t> keep(1, 2U);
  EXPECT_EQ(classifier.classify(unobserved, keep), 0U);
  EXPECT_EQ(keep[0], 1U);
}

// A deterministic scene shared by the determinism tests: the wall-and-blob
// setup above, spread over 32 scans (blob present for the first 8), with all
// points at voxel centers so a millimeter of jitter never crosses a boundary.
struct Scene
{
  std::vector<std::vector<Point>> scans;
  std::vector<Origin> origins;
  std::vector<Point> targets;  // wall + blob samples to classify (blob last)
};

Scene make_scene()
{
  Scene scene;
  std::vector<Point> wall;
  for (int y = 0; y <= 10; ++y) {
    for (int z = 0; z <= 2; ++z) {
      wall.push_back({10.5F, y + 0.5F, z + 0.5F});
    }
  }
  const std::vector<Point> blob{{5.5F, 5.5F, 0.5F}, {5.6F, 5.4F, 0.6F}};

  constexpr int kScans = 32;
  for (int s = 0; s < kScans; ++s) {
    const bool blob_present = s < 8;
    std::vector<Point> points;
    for (const Point & p : wall) {
      if (blob_present && p[1] == 5.5F && p[2] == 0.5F) {
        continue;  // occluded by the blob
      }
      points.push_back(p);
    }
    if (blob_present) {
      points.insert(points.end(), blob.begin(), blob.end());
    }
    scene.origins.push_back({0.5, 5.5, 0.5});
    scene.scans.push_back(std::move(points));
  }
  scene.targets = wall;
  scene.targets.insert(scene.targets.end(), blob.begin(), blob.end());
  return scene;
}

std::vector<std::uint8_t> classify_scene(const Scene & scene, int integrate_threads)
{
  slam::VoidRegionClassifier classifier(make_config(1.0, 0.2, 0));
  if (integrate_threads <= 1) {
    for (std::size_t s = 0; s < scene.scans.size(); ++s) {
      classifier.integrate(scene.scans[s], scene.origins[s]);
    }
  } else {
    std::vector<std::thread> workers;
    for (int w = 0; w < integrate_threads; ++w) {
      workers.emplace_back([&scene, &classifier, w, integrate_threads]() {
        for (std::size_t s = static_cast<std::size_t>(w); s < scene.scans.size();
             s += static_cast<std::size_t>(integrate_threads)) {
          classifier.integrate(scene.scans[s], scene.origins[s]);
        }
      });
    }
    for (auto & worker : workers) {
      worker.join();
    }
  }
  classifier.finalize(integrate_threads);
  std::vector<std::uint8_t> keep(scene.targets.size(), 2U);
  classifier.classify(scene.targets, keep);
  return keep;
}

TEST(VoidRegionClassifier, ClassificationIsIdenticalAcrossThreadCounts)
{
  // Exact equality is legitimate here per AGENTS.md "Numerical
  // Reproducibility": the free-space marking is a monotone OR, a reduction
  // that is exactly commutative and associative, so no work split can change
  // the outcome.
  const Scene scene = make_scene();
  const std::vector<std::uint8_t> serial = classify_scene(scene, 1);
  const std::vector<std::uint8_t> threaded = classify_scene(scene, 4);
  EXPECT_EQ(serial, threaded);

  // Sanity: the mover (last target) was removed and at least one wall point
  // survived, so the equality above is not comparing all-kept masks.
  EXPECT_EQ(serial.back(), 0U);
  EXPECT_NE(std::count(serial.begin(), serial.end(), 1U), 0);
}

TEST(VoidRegionClassifier, MillimeterJitterDoesNotChangeTheClassification)
{
  // Proxy for the int16-quantized FramePoints path (~1 mm worst-case error,
  // far below the 1 m voxel): jittered classify targets land in the same
  // voxels as the originals.
  const Scene scene = make_scene();
  slam::VoidRegionClassifier classifier(make_config(1.0, 0.2, 0));
  for (std::size_t s = 0; s < scene.scans.size(); ++s) {
    classifier.integrate(scene.scans[s], scene.origins[s]);
  }
  classifier.finalize(1);

  std::vector<Point> jittered = scene.targets;
  for (std::size_t i = 0; i < jittered.size(); ++i) {
    const float sign = (i % 2 == 0) ? 1.0F : -1.0F;
    for (int axis = 0; axis < 3; ++axis) {
      jittered[i][axis] += sign * 0.001F;
    }
  }

  std::vector<std::uint8_t> base(scene.targets.size(), 2U);
  std::vector<std::uint8_t> moved(jittered.size(), 2U);
  classifier.classify(scene.targets, base);
  classifier.classify(jittered, moved);
  EXPECT_EQ(base, moved);
}

}  // namespace
