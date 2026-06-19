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

#include <gtest/gtest.h>

#include <algorithm>
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
// Mirrors cloud_odometry_test's scene.
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

}  // namespace
