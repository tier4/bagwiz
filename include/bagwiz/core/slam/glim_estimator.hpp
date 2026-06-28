// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__GLIM_ESTIMATOR_HPP_
#define BAGWIZ__CORE__SLAM__GLIM_ESTIMATOR_HPP_

// INTERNAL — this header pulls in GLIM / Eigen. Include it ONLY from a
// BAGWIZ_WITH_SLAM translation unit (cloud_mapper.cpp), never from a GLIM-free
// public header. It turns the GLIM-free extrinsic POD into the right GLIM
// backend (LiDAR-only CT, LiDAR-IMU CPU, or — with --backend gpu — LiDAR-IMU GPU).

#include "bagwiz/core/slam/sensor_transform.hpp"

#include <Eigen/Geometry>
#include <glim/odometry/odometry_estimation_base.hpp>
#include <glim/odometry/odometry_estimation_cpu.hpp>
#include <glim/odometry/odometry_estimation_ct.hpp>
#ifdef BAGWIZ_WITH_SLAM_CUDA
#include <glim/odometry/odometry_estimation_gpu.hpp>
#endif
#include <glim/util/logging.hpp>

#include <spdlog/spdlog.h>

#include <memory>
#include <optional>
#include <stdexcept>

namespace bagwiz::core::slam::detail
{

// Convert the GLIM-free extrinsic POD into the Eigen transform GLIM expects for
// its T_lidar_imu parameter (p_lidar = T_lidar_imu * p_imu).
inline Eigen::Isometry3d to_isometry(const SensorTransform & t)
{
  Eigen::Quaterniond q(
    t.rotation_xyzw[3], t.rotation_xyzw[0], t.rotation_xyzw[1], t.rotation_xyzw[2]);  // w, x, y, z
  // normalize() on a (near-)zero quaternion yields NaN silently; reject it so a
  // malformed extrinsic fails loudly instead of feeding GLIM a NaN rotation.
  if (q.norm() < 1e-9) {
    throw std::runtime_error("SensorTransform rotation quaternion has near-zero norm");
  }
  q.normalize();
  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.linear() = q.toRotationMatrix();
  iso.translation() = Eigen::Vector3d(t.translation[0], t.translation[1], t.translation[2]);
  return iso;
}

// Build the odometry backend, returned as the common base so callers stay
// backend-agnostic (insert_frame / insert_imu / get_remaining_frames are virtual).
// A non-positive num_threads falls back to the default (4).
//
// Selection: with an extrinsic -> LiDAR-IMU (GPU when use_gpu, else CPU); without
// one -> LiDAR-only CT (GLIM has no GPU LiDAR-only backend, so use_gpu changes
// nothing here — the command layer logs a notice and GPU acceleration applies to
// sub/global mapping instead).
// use_gpu in a non-CUDA build is a hard error (the caller pre-flights it, but a
// direct API caller must also fail loudly rather than silently run on the CPU).
inline std::unique_ptr<glim::OdometryEstimationBase> make_odometry_estimator(
  const std::optional<SensorTransform> & t_lidar_imu, int num_threads = 4, bool use_gpu = false)
{
  const int threads = num_threads > 0 ? num_threads : 4;

  if (use_gpu) {
#ifdef BAGWIZ_WITH_SLAM_CUDA
    if (t_lidar_imu.has_value()) {
      glim::OdometryEstimationGPUParams params;
      params.T_lidar_imu = to_isometry(*t_lidar_imu);
      params.num_threads = threads;
      return std::make_unique<glim::OdometryEstimationGPU>(params);
    }
    // --backend gpu without --imu: GLIM has no GPU LiDAR-only odometry, so odometry
    // stays on CT (GPU acceleration still applies to sub/global mapping). The command
    // layer (map_slam.cpp) prints this notice; here we just fall through to CT.
#else
    throw std::runtime_error(
      "--backend gpu requested but this binary was built without CUDA "
      "(rebuild with -DBAGWIZ_WITH_SLAM_CUDA=ON / `pixi run build-slam-gpu`).");
#endif
  }

  if (t_lidar_imu.has_value()) {
    glim::OdometryEstimationCPUParams params;
    params.T_lidar_imu = to_isometry(*t_lidar_imu);
    params.num_threads = threads;
    return std::make_unique<glim::OdometryEstimationCPU>(params);
  }

  glim::OdometryEstimationCTParams params;
  params.num_threads = threads;
  return std::make_unique<glim::OdometryEstimationCT>(params);
}

// RAII silence of GLIM's default logger for the guard's lifetime, restored on
// destruction (so a throwing GLIM constructor cannot leave the shared logger
// muted for the rest of the process). GLIM emits ~50 "config not found / using
// default" lines while reading params at construction; we drive it with no
// config dir on purpose, so we mute exactly that startup chatter.
class ScopedLoggerSilence
{
public:
  ScopedLoggerSilence() : logger_(glim::get_default_logger()), previous_(logger_->level())
  {
    logger_->set_level(spdlog::level::off);
  }
  ~ScopedLoggerSilence() { logger_->set_level(previous_); }
  ScopedLoggerSilence(const ScopedLoggerSilence &) = delete;
  ScopedLoggerSilence & operator=(const ScopedLoggerSilence &) = delete;
  ScopedLoggerSilence(ScopedLoggerSilence &&) = delete;
  ScopedLoggerSilence & operator=(ScopedLoggerSilence &&) = delete;

private:
  std::shared_ptr<spdlog::logger> logger_;
  spdlog::level::level_enum previous_;
};

}  // namespace bagwiz::core::slam::detail

#endif  // BAGWIZ__CORE__SLAM__GLIM_ESTIMATOR_HPP_
