// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/cloud_mapper.hpp"

#include "bagwiz/core/slam/imu_sample.hpp"
#include "bagwiz/core/slam/lidar_scan.hpp"
#include "bagwiz/core/slam/sensor_transform.hpp"
#include "bagwiz/core/slam/visual_observation.hpp"

#include <Eigen/Geometry>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

// Integration test that drives the real GLIM SubMapping -> GlobalMapping
// pipeline through CloudMapper. Compiled only when BAGWIZ_WITH_SLAM is on (it
// links the vendored GLIM stack). A static sensor observing a fixed structured
// scene must yield a non-empty, finite optimized map plus a non-empty,
// time-monotonic, bounded trajectory — no claim about absolute accuracy, just
// that the in-process mapping pipeline runs and produces a sane result.
namespace
{
namespace slam = bagwiz::core::slam;

// A 10 x 10 x ~3 m "room": a floor grid plus four walls. Dense enough geometric
// structure for scan-to-model matching, expressed in the (static) sensor frame.
slam::LidarScan make_room_scan(std::int64_t stamp_ns)
{
  slam::LidarScan scan;
  scan.stamp_ns = stamp_ns;
  scan.frame_id = "lidar";

  constexpr double kHalf = 5.0;
  constexpr double kHeight = 3.0;
  constexpr int kN = 20;
  const auto lerp = [](double a, double b, int i, int n) {
    return a + (b - a) * static_cast<double>(i) / static_cast<double>(n - 1);
  };

  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j) {
      scan.points.push_back({lerp(-kHalf, kHalf, i, kN), lerp(-kHalf, kHalf, j, kN), -1.0});
    }
  }
  for (int i = 0; i < kN; ++i) {
    for (int k = 0; k < kN; ++k) {
      const double u = lerp(-kHalf, kHalf, i, kN);
      const double z = lerp(-1.0, kHeight - 1.0, k, kN);
      scan.points.push_back({u, -kHalf, z});
      scan.points.push_back({u, kHalf, z});
      scan.points.push_back({-kHalf, u, z});
      scan.points.push_back({kHalf, u, z});
    }
  }
  return scan;
}

// Same room, with a constant intensity attached to every point. A constant
// survives voxel-grid averaging exactly, so the exported map's intensities must
// all equal it — a tight check that intensity flows end to end.
slam::LidarScan make_room_scan_with_intensity(std::int64_t stamp_ns, float intensity)
{
  slam::LidarScan scan = make_room_scan(stamp_ns);
  scan.intensities.assign(scan.points.size(), static_cast<double>(intensity));
  return scan;
}

// finish() reports its wall-clock breakdown (global optimization / endpoint
// window fill / export fill) so the command layer can log where finalization time
// went — on LiDAR-only runs the scan-matching endpoint fill, not the iSAM2
// update, dominates. Fields must be populated: non-negative everywhere, and a
// run that exports points spends measurable time in the export fill.
TEST(CloudMapper, FinishReportsTimingBreakdown)
{
  slam::CloudMapper mapper;
  constexpr std::int64_t kDtNs = 100'000'000;  // 10 Hz
  std::int64_t stamp = 1'000'000'000'000'000'000LL;
  for (int i = 0; i < 120; ++i) {
    mapper.insert(make_room_scan(stamp));
    stamp += kDtNs;
  }

  const slam::CloudMap map = mapper.finish();

  ASSERT_FALSE(map.points.empty());
  EXPECT_GE(map.optimize_seconds, 0.0);
  EXPECT_GE(map.window_fill_seconds, 0.0);
  EXPECT_GT(map.export_seconds, 0.0);
}

TEST(CloudMapper, StationarySensorYieldsMapAndTrajectory)
{
  // Submaps form only after enough keyframes accumulate, and CT odometry
  // finalizes frames only as they leave its fixed-lag window. Feed a long
  // sequence (120 scans @ 10 Hz = 12 s) so at least one submap is created and
  // global optimization has something to optimize.
  slam::CloudMapper mapper;
  constexpr std::int64_t kDtNs = 100'000'000;  // 10 Hz
  std::int64_t stamp = 1'000'000'000'000'000'000LL;
  for (int i = 0; i < 120; ++i) {
    mapper.insert(make_room_scan(stamp));
    stamp += kDtNs;
  }

  const slam::CloudMap map = mapper.finish();

  // The optimized map is non-empty and every point is finite.
  ASSERT_FALSE(map.points.empty());
  for (const auto & p : map.points) {
    ASSERT_TRUE(std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]));
  }
  // Intensities are all-or-nothing: either empty or one per point.
  EXPECT_TRUE(map.intensities.empty() || map.intensities.size() == map.points.size());

  // The map of a 10 x 10 x ~3 m room should stay within a generous bounding box
  // even after global optimization (loose: this is a sanity bound, not an
  // accuracy claim).
  for (const auto & p : map.points) {
    EXPECT_LT(std::abs(p[0]), 20.0f);
    EXPECT_LT(std::abs(p[1]), 20.0f);
    EXPECT_LT(std::abs(p[2]), 20.0f);
  }

  // The globally-optimized trajectory is non-empty, time-monotonic, finite, and
  // bounded for a stationary sensor.
  ASSERT_FALSE(map.trajectory.empty());
  for (std::size_t i = 1; i < map.trajectory.size(); ++i) {
    EXPECT_LT(map.trajectory[i - 1].timestamp_ns, map.trajectory[i].timestamp_ns);
  }

  double min_x = map.trajectory.front().tx;
  double max_x = min_x;
  double min_y = map.trajectory.front().ty;
  double max_y = min_y;
  double min_z = map.trajectory.front().tz;
  double max_z = min_z;
  for (const auto & pose : map.trajectory) {
    ASSERT_TRUE(std::isfinite(pose.tx) && std::isfinite(pose.ty) && std::isfinite(pose.tz));
    ASSERT_TRUE(std::isfinite(pose.qx) && std::isfinite(pose.qy));
    ASSERT_TRUE(std::isfinite(pose.qz) && std::isfinite(pose.qw));
    min_x = std::min(min_x, pose.tx);
    max_x = std::max(max_x, pose.tx);
    min_y = std::min(min_y, pose.ty);
    max_y = std::max(max_y, pose.ty);
    min_z = std::min(min_z, pose.tz);
    max_z = std::max(max_z, pose.tz);
  }
  EXPECT_LT(max_x - min_x, 2.0) << "stationary trajectory drifted in x";
  EXPECT_LT(max_y - min_y, 2.0) << "stationary trajectory drifted in y";
  EXPECT_LT(max_z - min_z, 2.0) << "stationary trajectory drifted in z";
}

// A transient blob hangs in the room's interior for the first quarter of the
// sequence, then vanishes: with remove_dynamic_points on, the later scans' rays
// see through its voxels, so its ghost must be gone from the exported map while
// the room itself survives. A control run without the removal proves the blob
// otherwise persists (the assertion is not vacuous).
TEST(CloudMapper, RemoveDynamicPointsDropsATransientBlob)
{
  constexpr std::int64_t kDtNs = 100'000'000;  // 10 Hz
  constexpr double kBlobX = 2.0;
  constexpr double kBlobY = 0.0;
  constexpr double kBlobZ = 0.7;

  const auto make_scan = [&](std::int64_t stamp_ns, bool with_blob) {
    slam::LidarScan scan = make_room_scan(stamp_ns);
    if (with_blob) {
      for (int i = -1; i <= 1; ++i) {
        for (int j = -1; j <= 1; ++j) {
          for (int k = -1; k <= 1; ++k) {
            scan.points.push_back({kBlobX + 0.08 * i, kBlobY + 0.08 * j, kBlobZ + 0.08 * k});
          }
        }
      }
    }
    return scan;
  };
  const auto run_mapping = [&](bool remove_dynamic) {
    slam::CloudMapperConfig config;
    config.remove_dynamic_points = remove_dynamic;
    config.dynamic_voxel_size = 0.5;
    config.dynamic_sensor_offset = 0.15;
    config.dynamic_neighborhood = 0;  // the small synthetic room has too little
                                      // ray coverage for the erosion guard
    slam::CloudMapper mapper(config);
    std::int64_t stamp = 1'000'000'000'000'000'000LL;
    for (int i = 0; i < 120; ++i) {
      mapper.insert(make_scan(stamp, i < 30));
      stamp += kDtNs;
    }
    return mapper.finish();
  };
  const auto points_near_blob = [&](const slam::CloudMap & map) {
    std::size_t count = 0;
    for (const auto & p : map.points) {
      const double dx = p[0] - kBlobX;
      const double dy = p[1] - kBlobY;
      const double dz = p[2] - kBlobZ;
      if (dx * dx + dy * dy + dz * dz < 0.4 * 0.4) {
        ++count;
      }
    }
    return count;
  };

  const slam::CloudMap control = run_mapping(false);
  ASSERT_FALSE(control.points.empty());
  ASSERT_GT(points_near_blob(control), 0U) << "the blob never reached the control map";
  EXPECT_EQ(control.dynamic_input_point_count, 0U);
  EXPECT_EQ(control.dynamic_removed_point_count, 0U);

  const slam::CloudMap cleaned = run_mapping(true);
  ASSERT_FALSE(cleaned.points.empty());
  EXPECT_EQ(points_near_blob(cleaned), 0U) << "ghost points survived the removal";
  EXPECT_GT(cleaned.dynamic_removed_point_count, 0U);
  EXPECT_GT(cleaned.dynamic_input_point_count, cleaned.dynamic_removed_point_count);
  EXPECT_GE(cleaned.dynamic_removal_seconds, 0.0);
  // The room itself survives: the +x wall is still populated.
  bool wall_present = false;
  for (const auto & p : cleaned.points) {
    if (p[0] > 4.5F) {
      wall_present = true;
      break;
    }
  }
  EXPECT_TRUE(wall_present);
}

// IMU specific force for a level, static sensor whose frame is the LiDAR frame
// rotated 180 deg about X (the real Tamagawa mounting): gravity is "down" =
// LiDAR -z, so the LiDAR-frame specific force is (0,0,+g); rotating it into the
// 180-deg-X-flipped IMU frame gives (0,0,-g). No rotation.
slam::ImuSample make_flipped_gravity_imu(std::int64_t stamp_ns)
{
  slam::ImuSample imu;
  imu.stamp_ns = stamp_ns;
  imu.frame_id = "imu";
  imu.linear_acceleration = {0.0, 0.0, -9.80665};
  imu.angular_velocity = {0.0, 0.0, 0.0};
  return imu;
}

// Regression for the LiDAR-IMU map-placement bug: GLIM's CPU (IMU) backend stores
// each frame's points in the IMU frame (points_imu = T_imu_lidar * points_lidar),
// whereas the CT backend stores them in the LiDAR frame. The mapper places points
// with the LiDAR pose, so for a non-identity extrinsic the IMU-frame points were
// previously transformed by an extra T_imu_lidar — here a 180 deg rotation about
// X — which flips the whole map upside down (and smears it once the sensor moves).
// With an identity extrinsic the error vanishes, so the existing IMU test cannot
// see it; this one uses the 180-deg-X extrinsic that exposes it.
TEST(CloudMapper, ImuModeFlippedExtrinsicMapIsNotVerticallyFlipped)
{
  // 180 deg about X: quaternion (x,y,z,w) = (1,0,0,0). No translation. This is the
  // rotation half of the real Tamagawa LiDAR<-IMU extrinsic.
  slam::SensorTransform t_lidar_imu;
  t_lidar_imu.rotation_xyzw = {1.0, 0.0, 0.0, 0.0};
  t_lidar_imu.translation = {0.0, 0.0, 0.0};

  slam::CloudMapperConfig config;
  config.t_lidar_imu = t_lidar_imu;
  slam::CloudMapper mapper(config);

  constexpr std::int64_t kImuDtNs = 5'000'000;     // 200 Hz
  constexpr std::int64_t kScanDtNs = 100'000'000;  // 10 Hz
  const std::int64_t base = 1'000'000'000'000'000'000LL;

  // Prime 0.5 s of IMU so the backend can estimate its gravity-aligned state.
  std::int64_t imu_stamp = base;
  const std::int64_t first_scan = base + 500'000'000LL;
  while (imu_stamp < first_scan) {
    mapper.insert_imu(make_flipped_gravity_imu(imu_stamp));
    imu_stamp += kImuDtNs;
  }

  // 120 scans @ 10 Hz (12 s, well past the 5 s smoother lag) with IMU filling each
  // inter-scan interval and a sample exactly at the scan time.
  for (int i = 0; i < 120; ++i) {
    const std::int64_t scan_stamp = first_scan + static_cast<std::int64_t>(i) * kScanDtNs;
    while (imu_stamp < scan_stamp) {
      mapper.insert_imu(make_flipped_gravity_imu(imu_stamp));
      imu_stamp += kImuDtNs;
    }
    mapper.insert_imu(make_flipped_gravity_imu(scan_stamp));
    mapper.insert(make_room_scan(scan_stamp));
  }

  const slam::CloudMap map = mapper.finish();

  ASSERT_FALSE(map.points.empty());
  for (const auto & p : map.points) {
    ASSERT_TRUE(std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]));
  }

  // The world frame is gravity-aligned (+z up), so the room's floor (z = -1 in the
  // LiDAR frame, a dense planar spike of points) must sit in the LOWER half of the
  // map's z-extent. The pre-fix code placed IMU-frame points with the LiDAR pose,
  // flipping the room about X so the floor landed in the UPPER half. We test the
  // MEDIAN z relative to the z-extent — an origin-offset-independent invariant. The
  // floor spike pulls the median far below the midpoint when upright (~0.1) and far
  // above it when flipped (~0.9), so 0.5 separates the two with a wide margin.
  float z_min = std::numeric_limits<float>::infinity();
  float z_max = -std::numeric_limits<float>::infinity();
  std::vector<float> zs;
  zs.reserve(map.points.size());
  for (const auto & p : map.points) {
    z_min = std::min(z_min, p[2]);
    z_max = std::max(z_max, p[2]);
    zs.push_back(p[2]);
  }
  std::nth_element(zs.begin(), zs.begin() + zs.size() / 2, zs.end());
  const double z_median = zs[zs.size() / 2];
  ASSERT_GT(z_max - z_min, 0.5f) << "degenerate map: no vertical extent";
  const double floor_relative = (z_median - z_min) / (z_max - z_min);
  EXPECT_LT(floor_relative, 0.5)
    << "map is vertically flipped (floor above center): IMU-frame points placed with the "
       "LiDAR pose instead of being brought back into the LiDAR frame";

  // Sanity: the optimized trajectory still runs and is time-monotonic.
  ASSERT_FALSE(map.trajectory.empty());
  for (std::size_t i = 1; i < map.trajectory.size(); ++i) {
    EXPECT_LT(map.trajectory[i - 1].timestamp_ns, map.trajectory[i].timestamp_ns);
  }
}

// Regression: in LiDAR-only mode the odometry backend is GLIM's
// OdometryEstimationCT, which (unlike the LiDAR-IMU backend) never copies
// per-point intensities onto its estimation-frame cloud. The mapper must still
// export intensities by sourcing them from the preprocessed frame, so a scan fed
// with intensities yields a map that carries them — not an empty intensity set.
TEST(CloudMapper, LidarOnlyPreservesIntensity)
{
  constexpr float kIntensity = 7.0f;
  slam::CloudMapper mapper;                    // no extrinsic => LiDAR-only (CT) backend
  constexpr std::int64_t kDtNs = 100'000'000;  // 10 Hz
  std::int64_t stamp = 1'000'000'000'000'000'000LL;
  for (int i = 0; i < 120; ++i) {
    mapper.insert(make_room_scan_with_intensity(stamp, kIntensity));
    stamp += kDtNs;
  }

  const slam::CloudMap map = mapper.finish();

  ASSERT_FALSE(map.points.empty());
  // The defect: intensities came back empty in LiDAR-only mode. They must be
  // present and one-per-point.
  ASSERT_EQ(map.intensities.size(), map.points.size());
  // A constant intensity is invariant under per-voxel averaging, so every
  // exported value must equal what was fed (and crucially must not be zero).
  for (const float v : map.intensities) {
    EXPECT_NEAR(v, kIntensity, 1e-3f);
  }
}

// Regression guard for the IMU/CPU backend path. The LiDAR-only test above covers
// the CT backend; nothing otherwise exercises GLIM's LiDAR-IMU backend together
// with intensities. The exported point now flows through BOTH recently-merged
// fixes inside stash_frame: intensity sourced from raw_frame->intensities (#196)
// and geometry brought back into the LiDAR frame via T_lidar_sensor (#197). This
// pins that they coexist on the IMU path — a LiDAR-IMU run fed a constant
// intensity must export it, one value per point (not an empty intensity set).
TEST(CloudMapper, ImuModePreservesIntensity)
{
  constexpr float kIntensity = 7.0f;

  // Same 180-deg-X extrinsic + gravity priming as the placement test: the
  // extrinsic selects the CPU (LiDAR-IMU) backend and the gravity samples let it
  // initialize its gravity-aligned state. The scans additionally carry intensity.
  slam::SensorTransform t_lidar_imu;
  t_lidar_imu.rotation_xyzw = {1.0, 0.0, 0.0, 0.0};
  t_lidar_imu.translation = {0.0, 0.0, 0.0};

  slam::CloudMapperConfig config;
  config.t_lidar_imu = t_lidar_imu;
  slam::CloudMapper mapper(config);

  constexpr std::int64_t kImuDtNs = 5'000'000;     // 200 Hz
  constexpr std::int64_t kScanDtNs = 100'000'000;  // 10 Hz
  const std::int64_t base = 1'000'000'000'000'000'000LL;

  // Prime 0.5 s of IMU so the backend can estimate its gravity-aligned state.
  std::int64_t imu_stamp = base;
  const std::int64_t first_scan = base + 500'000'000LL;
  while (imu_stamp < first_scan) {
    mapper.insert_imu(make_flipped_gravity_imu(imu_stamp));
    imu_stamp += kImuDtNs;
  }

  // 120 scans @ 10 Hz with IMU filling each inter-scan interval and a sample
  // exactly at the scan time; every scan carries the same constant intensity.
  for (int i = 0; i < 120; ++i) {
    const std::int64_t scan_stamp = first_scan + static_cast<std::int64_t>(i) * kScanDtNs;
    while (imu_stamp < scan_stamp) {
      mapper.insert_imu(make_flipped_gravity_imu(imu_stamp));
      imu_stamp += kImuDtNs;
    }
    mapper.insert_imu(make_flipped_gravity_imu(scan_stamp));
    mapper.insert(make_room_scan_with_intensity(scan_stamp, kIntensity));
  }

  const slam::CloudMap map = mapper.finish();

  ASSERT_FALSE(map.points.empty());
  // Intensities must survive the IMU pipeline: present, one-per-point, and equal
  // to the constant fed in (a constant is invariant under per-voxel averaging).
  ASSERT_EQ(map.intensities.size(), map.points.size());
  for (const float v : map.intensities) {
    EXPECT_NEAR(v, kIntensity, 1e-3f);
  }
}

// A GNSS fix already projected to the local metric frame.
slam::GnssPoint make_gnss_point(std::int64_t t_ns, double x, double y, double z)
{
  slam::GnssPoint p;
  p.stamp_ns = t_ns;
  p.position = {x, y, z};
  return p;
}

// GNSS off (the default): insert_gnss must be a pure no-op — no constraints, and
// an otherwise-normal map + trajectory. Guards the enable_gnss gate on the ingest
// side. (The alignment math itself is unit-tested in gnss_alignment_test, which
// needs no GLIM and no sensor motion.)
TEST(CloudMapper, GnssDisabledIgnoresFixes)
{
  slam::CloudMapper mapper;                    // enable_gnss defaults to false
  constexpr std::int64_t kDtNs = 100'000'000;  // 10 Hz
  std::int64_t stamp = 1'000'000'000'000'000'000LL;
  for (int i = 0; i < 120; ++i) {
    mapper.insert(make_room_scan(stamp));
    mapper.insert_gnss(make_gnss_point(stamp, static_cast<double>(i) * 1.0, 0.0, 0.0));
    stamp += kDtNs;
  }

  const slam::CloudMap map = mapper.finish();
  EXPECT_EQ(map.gnss_factor_count, 0u);
  ASSERT_FALSE(map.points.empty());
  ASSERT_FALSE(map.trajectory.empty());
}

// GNSS on, but the sensor is stationary, so the SLAM baseline stays well under
// the default 10 m gate: build_gnss_factors must add nothing, and enabling GNSS
// must not destabilize a run that yields no constraints (finite, non-empty map +
// trajectory). Exercises the finish() path where gnss_factor_count == 0 and the
// injection callback is never registered.
TEST(CloudMapper, GnssInsufficientBaselineAddsNoConstraints)
{
  slam::CloudMapperConfig config;
  config.enable_gnss = true;  // default gnss_min_baseline = 10 m
  slam::CloudMapper mapper(config);

  constexpr std::int64_t kDtNs = 100'000'000;  // 10 Hz
  std::int64_t stamp = 1'000'000'000'000'000'000LL;
  for (int i = 0; i < 120; ++i) {
    mapper.insert(make_room_scan(stamp));
    // GNSS spans the whole scan range so the submaps are time-covered; only the
    // baseline gate stops a constraint from being created.
    mapper.insert_gnss(make_gnss_point(stamp, static_cast<double>(i) * 0.5, 0.0, 0.0));
    stamp += kDtNs;
  }

  const slam::CloudMap map = mapper.finish();
  EXPECT_EQ(map.gnss_factor_count, 0u);
  ASSERT_FALSE(map.points.empty());
  for (const auto & p : map.points) {
    ASSERT_TRUE(std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]));
  }
  ASSERT_FALSE(map.trajectory.empty());
}

// NOTE on the missing ">=2 submaps -> factors actually injected" case:
// GNSS factors require at least two GNSS-covered submaps with a baseline over
// gnss_min_baseline. GLIM's stock SubMapping only closes a submap after
// ~max_num_keyframes keyframes, which in practice needs >10 m of travel — more
// than this 10 m synthetic room allows before the sensor exits it and scan
// matching degrades. A stationary or short-motion scene (all this fixture can
// produce reliably) yields a single submap, so build_gnss_factors always stops
// at the ids.size() < 2 gate here (verified: one submap for 200 scans, moving or
// not). The factor-injection path is therefore covered as follows instead:
//   - the world<-GNSS alignment + prior targets: gnss_alignment_test (exact,
//     GLIM-free);
//   - the GTSAM factor type / callback wiring: compile-verified (identical to
//     glim_ext's proven gnss_global usage) and exercised end to end on real bags
//     with genuine multi-submap motion.
// Forcing a second submap here would require an unrealistically long, flaky
// synthetic trajectory, so it is deliberately left to real-bag validation.

// Feed a deterministic LiDAR-only room sequence (120 scans @ 10 Hz) into `mapper`.
// Identical input for any mapper, so two mappers differing only in the feed path
// (serial vs pipeline) must produce identical results at num_threads = 1.
void feed_room_only(slam::CloudMapper & mapper)
{
  constexpr std::int64_t kDtNs = 100'000'000;  // 10 Hz
  std::int64_t stamp = 1'000'000'000'000'000'000LL;
  for (int i = 0; i < 120; ++i) {
    mapper.insert(make_room_scan(stamp));
    stamp += kDtNs;
  }
}

// Feed a deterministic LiDAR-IMU room sequence (200 Hz IMU interleaved with 10 Hz
// scans, 180-deg-X extrinsic) into `mapper` — mirrors the ImuMode fixtures so the
// pipeline's IMU/scan interleaving through the queue is exercised.
void feed_room_imu(slam::CloudMapper & mapper)
{
  constexpr std::int64_t kImuDtNs = 5'000'000;     // 200 Hz
  constexpr std::int64_t kScanDtNs = 100'000'000;  // 10 Hz
  const std::int64_t base = 1'000'000'000'000'000'000LL;

  std::int64_t imu_stamp = base;
  const std::int64_t first_scan = base + 500'000'000LL;
  while (imu_stamp < first_scan) {
    mapper.insert_imu(make_flipped_gravity_imu(imu_stamp));
    imu_stamp += kImuDtNs;
  }
  for (int i = 0; i < 120; ++i) {
    const std::int64_t scan_stamp = first_scan + static_cast<std::int64_t>(i) * kScanDtNs;
    while (imu_stamp < scan_stamp) {
      mapper.insert_imu(make_flipped_gravity_imu(imu_stamp));
      imu_stamp += kImuDtNs;
    }
    mapper.insert_imu(make_flipped_gravity_imu(scan_stamp));
    mapper.insert(make_room_scan(scan_stamp));
  }
}

// Assert two CloudMaps are identical: the pipeline only overlaps stages, so at
// num_threads = 1 (GLIM's bit-reproducible path) it must reproduce the serial
// trajectory and map exactly (modulo float ULPs).
void expect_maps_identical(const slam::CloudMap & a, const slam::CloudMap & b)
{
  ASSERT_EQ(a.trajectory.size(), b.trajectory.size());
  for (std::size_t i = 0; i < a.trajectory.size(); ++i) {
    const auto & pa = a.trajectory[i];
    const auto & pb = b.trajectory[i];
    EXPECT_EQ(pa.timestamp_ns, pb.timestamp_ns) << "pose " << i;
    EXPECT_DOUBLE_EQ(pa.tx, pb.tx) << "pose " << i;
    EXPECT_DOUBLE_EQ(pa.ty, pb.ty) << "pose " << i;
    EXPECT_DOUBLE_EQ(pa.tz, pb.tz) << "pose " << i;
    EXPECT_DOUBLE_EQ(pa.qx, pb.qx) << "pose " << i;
    EXPECT_DOUBLE_EQ(pa.qy, pb.qy) << "pose " << i;
    EXPECT_DOUBLE_EQ(pa.qz, pb.qz) << "pose " << i;
    EXPECT_DOUBLE_EQ(pa.qw, pb.qw) << "pose " << i;
  }

  ASSERT_EQ(a.points.size(), b.points.size());
  for (std::size_t i = 0; i < a.points.size(); ++i) {
    EXPECT_FLOAT_EQ(a.points[i][0], b.points[i][0]) << "point " << i;
    EXPECT_FLOAT_EQ(a.points[i][1], b.points[i][1]) << "point " << i;
    EXPECT_FLOAT_EQ(a.points[i][2], b.points[i][2]) << "point " << i;
  }

  ASSERT_EQ(a.intensities.size(), b.intensities.size());
  for (std::size_t i = 0; i < a.intensities.size(); ++i) {
    EXPECT_FLOAT_EQ(a.intensities[i], b.intensities[i]) << "intensity " << i;
  }
}

// Assert two trajectories agree to within a tolerance: same poses + timestamps,
// translations within `trans_tol` m and quaternion components within `rot_tol`.
// Used for the LiDAR-IMU backend, whose GTSAM optimizer is NOT bit-reproducible
// run-to-run (two serial runs already drift at the ~1e-15 level), so exact parity
// would be a GLIM-determinism test, not a pipeline-ordering test. A genuine
// ordering bug in the queue (IMU/scan mis-interleaved) diverges by cm–m, far above
// these tolerances.
void expect_trajectories_close(
  const slam::CloudMap & a, const slam::CloudMap & b, double trans_tol, double rot_tol)
{
  ASSERT_EQ(a.trajectory.size(), b.trajectory.size());
  for (std::size_t i = 0; i < a.trajectory.size(); ++i) {
    const auto & pa = a.trajectory[i];
    const auto & pb = b.trajectory[i];
    EXPECT_EQ(pa.timestamp_ns, pb.timestamp_ns) << "pose " << i;
    EXPECT_NEAR(pa.tx, pb.tx, trans_tol) << "pose " << i;
    EXPECT_NEAR(pa.ty, pb.ty, trans_tol) << "pose " << i;
    EXPECT_NEAR(pa.tz, pb.tz, trans_tol) << "pose " << i;
    EXPECT_NEAR(pa.qx, pb.qx, rot_tol) << "pose " << i;
    EXPECT_NEAR(pa.qy, pb.qy, rot_tol) << "pose " << i;
    EXPECT_NEAR(pa.qz, pb.qz, rot_tol) << "pose " << i;
    EXPECT_NEAR(pa.qw, pb.qw, rot_tol) << "pose " << i;
  }
}

// The default pipeline must be bit-identical to the serial path (CloudMapperConfig
// disable_pipeline = true): the consumer processes events in the same bag order, so
// odometry/sub/global see identical input. Pinned to num_threads = 1 (GLIM's
// reproducibility-guaranteed path) so any difference is attributable to the pipeline,
// not GLIM's own multithreaded non-determinism.
TEST(CloudMapper, PipelineMatchesSerialLidarOnly)
{
  slam::CloudMapperConfig serial_config;
  serial_config.num_threads = 1;
  serial_config.disable_pipeline = true;
  slam::CloudMapper serial_mapper(serial_config);
  feed_room_only(serial_mapper);
  const slam::CloudMap serial_map = serial_mapper.finish();

  slam::CloudMapperConfig pipeline_config;
  pipeline_config.num_threads = 1;
  pipeline_config.disable_pipeline = false;
  slam::CloudMapper pipeline_mapper(pipeline_config);
  feed_room_only(pipeline_mapper);
  const slam::CloudMap pipeline_map = pipeline_mapper.finish();

  ASSERT_FALSE(serial_map.points.empty());
  ASSERT_FALSE(serial_map.trajectory.empty());
  expect_maps_identical(serial_map, pipeline_map);
}

// Parity check with the LiDAR-IMU backend, which routes IMU samples and scans
// through the queue interleaved — verifies the pipeline preserves the IMU/scan
// ordering each preintegrator requires. Compared with a tolerance (not exact):
// GLIM's IMU optimizer is not bit-reproducible run-to-run (two serial runs already
// drift ~1e-15), so this asserts the pipeline does not DIVERGE from serial, which
// it would by cm–m if the queue mis-ordered IMU and scans.
TEST(CloudMapper, PipelineMatchesSerialImu)
{
  slam::SensorTransform t_lidar_imu;
  t_lidar_imu.rotation_xyzw = {1.0, 0.0, 0.0, 0.0};
  t_lidar_imu.translation = {0.0, 0.0, 0.0};

  slam::CloudMapperConfig serial_config;
  serial_config.num_threads = 1;
  serial_config.disable_pipeline = true;
  serial_config.t_lidar_imu = t_lidar_imu;
  slam::CloudMapper serial_mapper(serial_config);
  feed_room_imu(serial_mapper);
  const slam::CloudMap serial_map = serial_mapper.finish();

  slam::CloudMapperConfig pipeline_config;
  pipeline_config.num_threads = 1;
  pipeline_config.disable_pipeline = false;
  pipeline_config.t_lidar_imu = t_lidar_imu;
  slam::CloudMapper pipeline_mapper(pipeline_config);
  feed_room_imu(pipeline_mapper);
  const slam::CloudMap pipeline_map = pipeline_mapper.finish();

  ASSERT_FALSE(serial_map.points.empty());
  ASSERT_FALSE(serial_map.trajectory.empty());
  // 1 mm / 1e-3 quat: ~1000x above GLIM's run-to-run noise, far below any real
  // ordering bug.
  expect_trajectories_close(serial_map, pipeline_map, 1e-3, 1e-3);
  // The map must still be produced and sane (point geometry tracks the poses, so a
  // tolerance-equal trajectory yields a near-equal map; assert non-empty + finite).
  ASSERT_FALSE(pipeline_map.points.empty());
  for (const auto & p : pipeline_map.points) {
    ASSERT_TRUE(std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]));
  }
}

// finish() with no inserts must not deadlock or start a consumer (the pipeline is
// lazily started on first insert) and returns an empty map either way.
TEST(CloudMapper, PipelineFinishWithoutInsertIsEmpty)
{
  slam::CloudMapperConfig config;
  config.num_threads = 1;
  slam::CloudMapper mapper(config);
  const slam::CloudMap map = mapper.finish();
  EXPECT_TRUE(map.trajectory.empty());
  EXPECT_TRUE(map.points.empty());
}

slam::VisualObservation make_visual_observation(
  std::int64_t stamp_ns, std::int32_t camera_id, std::uint64_t track_id, double x, double y)
{
  slam::VisualObservation obs;
  obs.camera_id = camera_id;
  obs.track_id = track_id;
  obs.stamp_ns = stamp_ns;
  obs.x = x;
  obs.y = y;
  return obs;
}

// insert_visual_observations() must be a pure no-op when config.visual_cameras
// is empty (the default): no factors, no tracked ids, and an otherwise-normal
// map. Mirrors GnssDisabledIgnoresFixes for the visual ingest gate. The CPU
// mapping pipeline is not run-to-run reproducible even for identical input (see
// the module comment), so this compares the point count against a separate
// no-observation baseline run with a loose tolerance instead of exact equality.
TEST(CloudMapper, VisualObservationsIgnoredWithoutCameras)
{
  slam::CloudMapper mapper;                    // visual_cameras defaults to empty
  constexpr std::int64_t kDtNs = 100'000'000;  // 10 Hz
  std::int64_t stamp = 1'000'000'000'000'000'000LL;
  for (int i = 0; i < 120; ++i) {
    mapper.insert(make_room_scan(stamp));
    const auto obs = make_visual_observation(stamp, 0, static_cast<std::uint64_t>(i), 0.01, 0.02);
    mapper.insert_visual_observations(std::array{obs});
    stamp += kDtNs;
  }
  const slam::CloudMap map = mapper.finish();
  EXPECT_EQ(map.visual_factor_count, 0);
  EXPECT_EQ(map.visual_track_count, 0);
  ASSERT_FALSE(map.points.empty());

  slam::CloudMapper baseline_mapper;  // identical feed, no visual observations at all
  feed_room_only(baseline_mapper);
  const slam::CloudMap baseline_map = baseline_mapper.finish();
  ASSERT_FALSE(baseline_map.points.empty());

  const auto count = static_cast<double>(map.points.size());
  const auto baseline_count = static_cast<double>(baseline_map.points.size());
  EXPECT_NEAR(count, baseline_count, baseline_count * 0.2);
}

// One configured camera (identity extrinsic): insert_visual_observations must
// accumulate observations, and finish() must report the number of distinct
// track ids received, independent of whether any factor was built from them —
// this stationary run yields a single submap, so nothing here is co-visible and
// visual_factor_count stays 0 (VisualObservationsProduceFactors covers the
// factor path).
TEST(CloudMapper, VisualObservationCountIsReported)
{
  slam::CloudMapperConfig config;
  config.visual_cameras.push_back(slam::SensorTransform{});  // identity extrinsic
  slam::CloudMapper mapper(config);

  constexpr std::int64_t kDtNs = 100'000'000;  // 10 Hz
  std::int64_t stamp = 1'000'000'000'000'000'000LL;
  for (int i = 0; i < 120; ++i) {
    mapper.insert(make_room_scan(stamp));
    const auto track_id = static_cast<std::uint64_t>(i % 3);  // 3 distinct tracks
    const auto obs = make_visual_observation(stamp, 0, track_id, 0.01, 0.02);
    mapper.insert_visual_observations(std::array{obs});
    stamp += kDtNs;
  }

  const slam::CloudMap map = mapper.finish();
  EXPECT_EQ(map.visual_track_count, 3);
  ASSERT_FALSE(map.points.empty());
}

// Track ids are unique only within a camera: each VisualFrontend numbers its own
// tracks from 0. Two cameras reusing the same ids are 6 distinct tracks, not 3,
// so the count must key on (camera_id, track_id).
TEST(CloudMapper, VisualTrackCountSeparatesCamerasReusingTrackIds)
{
  slam::CloudMapperConfig config;
  config.visual_cameras.push_back(slam::SensorTransform{});
  config.visual_cameras.push_back(slam::SensorTransform{});
  slam::CloudMapper mapper(config);

  constexpr std::int64_t kDtNs = 100'000'000;  // 10 Hz
  std::int64_t stamp = 1'000'000'000'000'000'000LL;
  for (int i = 0; i < 120; ++i) {
    mapper.insert(make_room_scan(stamp));
    const auto track_id = static_cast<std::uint64_t>(i % 3);  // 3 ids, reused by both cameras
    mapper.insert_visual_observations(
      std::array{
        make_visual_observation(stamp, 0, track_id, 0.01, 0.02),
        make_visual_observation(stamp, 1, track_id, 0.03, 0.04)});
    stamp += kDtNs;
  }

  const slam::CloudMap map = mapper.finish();
  EXPECT_EQ(map.visual_track_count, 6);
  ASSERT_FALSE(map.points.empty());
}

// ---------------------------------------------------------------------------
// Co-visibility scene for the visual factor injection.
//
// A rig-projection factor ties the submaps that saw one landmark together, so
// it needs two things the fixtures above deliberately avoid, and the scene
// below arranges both:
//
//   * More than one submap. GLIM finalizes a submap once it holds
//     submap_max_keyframes keyframes, and it only cuts a keyframe when a scan's
//     overlap with the last one drops below 0.8. The room is fully visible from
//     every point inside it, so that overlap never drops and the stock 15
//     yields exactly ONE submap however long the sequence (the same reason the
//     GNSS NOTE above gives for its missing case). submap_max_keyframes = 1
//     closes one submap per odometry frame instead.
//   * Parallax. Triangulating a landmark needs a baseline between the views, so
//     the sensor slides along +y — perpendicular to the line of sight of the
//     camera watching the +x wall, which is where the parallax per metre
//     travelled is largest.
//
// The room stays fixed in the world and the sensor starts at the world origin,
// which is also where GLIM anchors its estimate, so the ground-truth poses the
// observations are projected through are the poses SLAM converges to.
// ---------------------------------------------------------------------------

// 60 scans at 10 Hz. The LiDAR-only (CT) backend hands sub mapping only the
// frames its 5 s fixed-lag smoother has marginalized — it has no
// end-of-sequence flush, unlike the LiDAR-IMU backend — so a 6 s run passes on
// only its first ten frames or so, which at one keyframe per submap is a handful
// of submaps. That is plenty of co-visibility while keeping the global graph's
// submap matching (fully connected here, since every submap sees this whole
// room) small enough for the test to stay fast.
constexpr std::int64_t kVisualBaseStamp = 1'000'000'000'000'000'000LL;
constexpr std::int64_t kVisualScanDtNs = 100'000'000;  // 10 Hz
constexpr int kVisualScanCount = 60;
constexpr double kVisualStepY = 0.05;  // sensor slide per scan [m]
constexpr double kWallX = 5.0;         // the room's +x wall, the camera's target

// The room fixture seen from a sensor displaced by `sensor_y`: subtracting the
// displacement from the sensor-frame coordinates keeps the room's world points
// exactly where make_room_scan puts them.
slam::LidarScan make_room_scan_shifted(std::int64_t stamp_ns, double sensor_y)
{
  slam::LidarScan scan = make_room_scan(stamp_ns);
  for (auto & p : scan.points) {
    p[1] -= sensor_y;
  }
  return scan;
}

// T_lidar_cam for a forward-looking optical frame (z forward, x right, y down)
// on the LiDAR body frame (x forward, y left, z up): body x = optical z, body
// y = -optical x, body z = -optical y. Same convention as visual_factors_test.
Eigen::Isometry3d forward_camera_pose()
{
  Eigen::Isometry3d extrinsic = Eigen::Isometry3d::Identity();
  extrinsic.linear() << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0;
  return extrinsic;
}

// The same extrinsic as the Eigen-free POD CloudMapperConfig carries.
slam::SensorTransform forward_camera_extrinsic()
{
  const Eigen::Quaterniond q(forward_camera_pose().rotation());
  slam::SensorTransform extrinsic;
  extrinsic.rotation_xyzw = {q.x(), q.y(), q.z(), q.w()};
  extrinsic.translation = {0.0, 0.0, 0.0};
  return extrinsic;
}

// Ten landmarks on the +x wall, one per track. None sits on either optical axis:
// a point dead ahead of a sideways-sliding camera keeps the same image position
// and so carries no parallax to triangulate from.
std::vector<Eigen::Vector3d> wall_landmarks()
{
  std::vector<Eigen::Vector3d> landmarks;
  for (int i = 0; i < 10; ++i) {
    const double y = -1.8 + 0.4 * static_cast<double>(i);  // -1.8 .. 1.8, never 0
    const double z = (i % 2 == 0) ? 0.5 : 1.2;
    landmarks.emplace_back(kWallX, y, z);
  }
  return landmarks;
}

// One forward-looking camera, one submap per odometry frame, and a measurement
// sigma loose enough (1e-2 normalized ~ 5 cm at the wall's 5 m range, and the
// triangulation outlier gate is 3x that) to absorb the SLAM estimate's
// deviation from the ground-truth poses the observations are projected through.
slam::CloudMapperConfig make_visual_config()
{
  slam::CloudMapperConfig config;
  config.visual_cameras.push_back(forward_camera_extrinsic());
  config.visual_obs_sigma = 1e-2;
  config.visual_gate_distance = 1.0;
  config.submap_max_keyframes = 1;
  // Keep every observation. Only the run's opening scans reach a finalized
  // submap (see kVisualScanCount), and thinning each track's 60 observations to
  // the default 16 BEFORE they are associated would leave only about three of
  // them inside a submap span — the bare minimum a factor needs.
  config.visual_max_obs_per_track = 0;
  return config;
}

// Feed the sliding-sensor room sequence, plus one observation per landmark per
// scan: every landmark becomes a track seen from every submap of the run.
// Observations carry the scan stamps verbatim, which is what the one-frame
// submaps' frame stamps are (GLIM passes the frame timestamp through untouched),
// so each one lands inside a submap's span rather than in a gap between them.
// With `with_gnss` the same run also gets a GNSS fix per scan.
void feed_room_with_visual_tracks(slam::CloudMapper & mapper, bool with_gnss)
{
  const Eigen::Isometry3d T_lidar_cam = forward_camera_pose();
  const std::vector<Eigen::Vector3d> landmarks = wall_landmarks();

  for (int i = 0; i < kVisualScanCount; ++i) {
    const std::int64_t stamp = kVisualBaseStamp + static_cast<std::int64_t>(i) * kVisualScanDtNs;
    const double sensor_y = kVisualStepY * static_cast<double>(i);
    mapper.insert(make_room_scan_shifted(stamp, sensor_y));
    if (with_gnss) {
      mapper.insert_gnss(make_gnss_point(stamp, 0.0, sensor_y, 0.0));
    }

    Eigen::Isometry3d T_world_lidar = Eigen::Isometry3d::Identity();
    T_world_lidar.translation() = Eigen::Vector3d(0.0, sensor_y, 0.0);
    const Eigen::Isometry3d T_cam_world = (T_world_lidar * T_lidar_cam).inverse();

    std::vector<slam::VisualObservation> batch;
    batch.reserve(landmarks.size());
    for (std::size_t track = 0; track < landmarks.size(); ++track) {
      const Eigen::Vector3d p_cam = T_cam_world * landmarks[track];
      ASSERT_GT(p_cam.z(), 0.0) << "landmark " << track << " is behind the camera";
      batch.push_back(
        make_visual_observation(stamp, 0, track, p_cam.x() / p_cam.z(), p_cam.y() / p_cam.z()));
    }
    mapper.insert_visual_observations(batch);
  }
}

// The end-to-end claim of the visual path: observations consistent with the
// LiDAR trajectory turn into rig-projection factors in the global graph, and
// the run still produces a sane map.
TEST(CloudMapper, VisualObservationsProduceFactors)
{
  slam::CloudMapper mapper(make_visual_config());
  feed_room_with_visual_tracks(mapper, /*with_gnss=*/false);

  const slam::CloudMap map = mapper.finish();

  EXPECT_EQ(map.visual_track_count, 10);
  // A track yields at most one factor, and at least one must survive
  // triangulation and the LiDAR-support gate. Not == 10: a track is dropped
  // when its observations all land in one submap, and which frames reach a
  // finalized submap is GLIM's scheduling decision, not this test's.
  EXPECT_GE(map.visual_factor_count, 1);
  EXPECT_LE(map.visual_factor_count, 10);

  ASSERT_FALSE(map.points.empty());
  ASSERT_FALSE(map.trajectory.empty());
  for (const auto & p : map.points) {
    ASSERT_TRUE(std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]));
  }
}

// GNSS priors and visual factors share one pending-factor vector and one
// on_smoother_update slot, so a regression in that plumbing shows up as one kind
// silencing the other. Here GNSS is enabled and its fixes cover every submap,
// but the baseline between them is sub-metre — far under the 10 m gate — so it
// contributes nothing, and the visual factors must still reach the graph.
TEST(CloudMapper, GnssAndVisualCoexist)
{
  slam::CloudMapperConfig config = make_visual_config();
  config.enable_gnss = true;
  slam::CloudMapper mapper(config);
  feed_room_with_visual_tracks(mapper, /*with_gnss=*/true);

  const slam::CloudMap map = mapper.finish();

  EXPECT_EQ(map.gnss_factor_count, 0u);
  EXPECT_EQ(map.visual_track_count, 10);
  EXPECT_GE(map.visual_factor_count, 1);
  EXPECT_LE(map.visual_factor_count, 10);
  ASSERT_FALSE(map.points.empty());
  ASSERT_FALSE(map.trajectory.empty());
}

}  // namespace
