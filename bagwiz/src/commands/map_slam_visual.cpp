// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "map_slam_visual.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/image/packed_raster.hpp"

#include <chrono>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace bagwiz::commands
{
namespace
{
// Images in flight per camera. The frontend's per-frame work (decode + KLT)
// runs at roughly camera frame rate, so a handful of buffered frames is enough
// to absorb jitter without holding a large multiple of a JPEG payload per
// camera.
constexpr std::size_t kQueueCapacityWeight = 8;
constexpr std::size_t kItemWeight = 1;

using Clock = std::chrono::steady_clock;

std::int64_t ns_since(Clock::time_point start)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - start).count();
}

}  // namespace

VisualFeed::VisualFeed(
  std::span<const std::string> image_topics, std::span<const core::image::CameraInfo> cameras,
  int max_features, core::slam::CloudMapper & mapper, const char * logger)
: image_topics_(image_topics.begin(), image_topics.end()), mapper_(mapper), logger_(logger)
{
  const std::size_t count = image_topics_.size();
  images_.assign(count, 0);
  failures_.assign(count, 0);
  wait_ns_.assign(count, 0);
  decode_ns_.assign(count, 0);
  track_ns_.assign(count, 0);
  frontends_.reserve(count);
  queues_.reserve(count);
  workers_.reserve(count);
  for (std::size_t cam = 0; cam < count; ++cam) {
    core::slam::VisualFrontendConfig config;
    // The observation's camera_id indexes CloudMapperConfig::visual_cameras,
    // which the caller built in this same order.
    config.camera_id = static_cast<int>(cam);
    config.camera = cameras[cam];
    config.max_features = max_features;
    frontends_.push_back(std::make_unique<core::slam::VisualFrontend>(std::move(config)));
    queues_.push_back(
      std::make_unique<core::slam::FrameFeedQueue<VisualWorkItem>>(kQueueCapacityWeight));
  }
  // Started only once every frontend and queue exists, so no worker can reach
  // for a slot still being constructed.
  for (std::size_t cam = 0; cam < count; ++cam) {
    workers_.emplace_back([this, cam]() { run_worker(cam); });
  }
}

VisualFeed::~VisualFeed()
{
  if (finished_) {
    return;
  }
  // Aborted read (a bag error, or an early return): the observations are moot,
  // so abandon the queued images rather than draining them — but the workers
  // must still be joined, since destroying a joinable thread terminates the
  // process.
  for (auto & queue : queues_) {
    queue->cancel();
  }
  for (auto & worker : workers_) {
    worker.join();
  }
}

void VisualFeed::push(
  std::size_t cam, std::int64_t stamp_ns, std::string type, std::vector<std::byte> payload)
{
  const auto t = std::chrono::steady_clock::now();
  const bool pushed =
    queues_[cam]->push(VisualWorkItem{stamp_ns, std::move(type), std::move(payload)}, kItemWeight);
  wait_ns_[cam] += ns_since(t);
  if (pushed) {
    ++images_[cam];
  }
}

void VisualFeed::finish()
{
  if (finished_) {
    return;
  }
  finished_ = true;
  for (auto & queue : queues_) {
    queue->close();
  }
  for (auto & worker : workers_) {
    worker.join();
  }
  for (std::size_t cam = 0; cam < image_topics_.size(); ++cam) {
    BAGWIZ_LOG_INFO(
      logger_,
      "Visual tracking: fed %" PRId64 " image(s) from '%s' (%" PRId64 " failed to decode or track)",
      images_[cam], image_topics_[cam].c_str(), failures_[cam]);
    const auto sec = [](std::int64_t ns) { return static_cast<double>(ns) * 1e-9; };
    const core::slam::VisualFrontendStats & fs = frontends_[cam]->stats();
    BAGWIZ_LOG_INFO(
      logger_,
      "Visual timing '%s': decode %.1fs, track %.1fs (gray %.1fs, resize %.1fs, klt %.1fs+%.1fs, "
      "detect %.1fs/%" PRId64 ", emit %.1fs), reader waited %.1fs",
      image_topics_[cam].c_str(), sec(decode_ns_[cam]), sec(track_ns_[cam]), sec(fs.gray_ns),
      sec(fs.resize_ns), sec(fs.klt_forward_ns), sec(fs.klt_backward_ns), sec(fs.detect_ns),
      fs.detect_calls, sec(fs.emit_ns), sec(wait_ns_[cam]));
  }
}

void VisualFeed::run_worker(std::size_t cam)
{
  std::int64_t failures = 0;
  // Surface the first failure per topic at WARN — a wrong CameraInfo size or an
  // undecodable codec would otherwise vanish into a counter — then only count
  // the rest, since a systematic failure spams one line per image otherwise.
  const auto count_failure = [&](const char * what) {
    if (failures == 0) {
      BAGWIZ_LOG_WARN(
        logger_,
        "Tracking an image on '%s' failed: %s (further failures on this topic are only "
        "counted)",
        image_topics_[cam].c_str(), what);
    }
    ++failures;
  };

  std::int64_t decode_ns = 0;
  std::int64_t track_ns = 0;
  VisualWorkItem item;
  while (queues_[cam]->pop(item)) {
    try {
      auto t = std::chrono::steady_clock::now();
      auto decoded = core::image::to_packed_raster(item.type, item.payload);
      decode_ns += ns_since(t);
      if (!decoded.ok()) {
        count_failure(decoded.error.c_str());
        continue;
      }
      const auto & raster = *decoded.raster;
      t = std::chrono::steady_clock::now();
      const auto observations =
        frontends_[cam]->track(item.stamp_ns, raster.bgr, raster.width, raster.height);
      track_ns += ns_since(t);
      mapper_.insert_visual_observations(observations);
    } catch (const std::exception & e) {
      // A decode or frontend throw must not escape this thread (that
      // terminates the process); the run continues with whatever the other
      // frames yield.
      count_failure(e.what());
    }
  }
  failures_[cam] = failures;
  decode_ns_[cam] = decode_ns;
  track_ns_[cam] = track_ns;
}

}  // namespace bagwiz::commands
