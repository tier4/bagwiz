// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__GLIM_ESTIMATOR_HPP_
#define BAGWIZ__CORE__SLAM__GLIM_ESTIMATOR_HPP_

// INTERNAL — this header pulls in GLIM / Eigen. Include it ONLY from the
// BAGWIZ_WITH_SLAM translation units (cloud_odometry.cpp / cloud_mapper.cpp),
// never from a GLIM-free public header. It factors out the one piece both the
// odometry and the mapping wrappers share: turning the GLIM-free extrinsic POD
// into the right GLIM backend (LiDAR-only CT vs LiDAR-IMU CPU).

#include "bagwiz/core/slam/sensor_transform.hpp"

#include <Eigen/Geometry>
#include <glim/odometry/odometry_estimation_base.hpp>
#include <glim/odometry/odometry_estimation_cpu.hpp>
#include <glim/odometry/odometry_estimation_ct.hpp>
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

// Build the odometry backend: LiDAR-IMU CPU when an extrinsic is given (with that
// T_lidar_imu), else LiDAR-only CT. Returned as the common base so callers stay
// backend-agnostic (insert_frame / insert_imu / get_remaining_frames are virtual).
inline std::unique_ptr<glim::OdometryEstimationBase> make_odometry_estimator(
  const std::optional<SensorTransform> & t_lidar_imu)
{
  if (t_lidar_imu.has_value()) {
    glim::OdometryEstimationCPUParams params;
    params.T_lidar_imu = to_isometry(*t_lidar_imu);
    return std::make_unique<glim::OdometryEstimationCPU>(params);
  }
  return std::make_unique<glim::OdometryEstimationCT>();
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
