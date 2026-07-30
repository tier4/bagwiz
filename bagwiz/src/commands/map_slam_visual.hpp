// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__MAP_SLAM_VISUAL_HPP_
#define COMMANDS__MAP_SLAM_VISUAL_HPP_

#include "bagwiz/core/image/camera_info.hpp"
#include "bagwiz/core/slam/cloud_mapper.hpp"
#include "bagwiz/core/slam/frame_feed_queue.hpp"
#include "bagwiz/core/slam/visual_frontend.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <thread>
#include <vector>

// The `map slam --cam` visual feed, split out of map_slam.cpp so the camera
// worker pipeline is not tangled with the bag-reading loop it hangs off.
// CLI-internal: this header lives with the command sources and is not
// installed.
namespace bagwiz::commands
{

// One camera image on its way to a visual worker: the UNDECODED message plus
// the capture stamp the reader already read off its header.
struct VisualWorkItem
{
  std::int64_t stamp_ns = 0;
  std::string type;
  std::vector<std::byte> payload;
};

// One VisualFrontend + bounded queue + worker thread per --cam camera, driven
// from the single-threaded bag read loop.
//
// push() hands one undecoded image to its camera's worker, which decodes it,
// tracks its features and inserts the observations into the mapper
// (CloudMapper::insert_visual_observations is thread-safe). Keeping the decode
// and the tracking off the reader thread matters: that thread also feeds GLIM's
// odometry, and a JPEG decode plus a KLT pass per camera frame would otherwise
// serialize behind it. A frontend is touched only by its own worker and sees
// its camera's frames in bag order, so the emitted tracks match a serial run's;
// the queue bound is the backpressure that stops a slow camera from ballooning
// memory.
//
// finish() closes the queues and joins the workers. It MUST run before
// CloudMapper::finish(), which turns the buffered observations into factors.
// The destructor cancels and joins instead, so an aborted read cannot leave a
// worker blocked in pop().
class VisualFeed
{
public:
  // One camera per `image_topics` entry, with `cameras` parallel to it
  // (`cameras[i]` is `image_topics[i]`'s full-resolution CameraInfo). The index
  // is the VisualObservation::camera_id stamped into every observation, i.e.
  // the row of CloudMapperConfig::visual_cameras the mapper resolves it
  // against, so both must be built in --cam listing order. `mapper` and
  // `logger` must outlive this object.
  VisualFeed(
    std::span<const std::string> image_topics, std::span<const core::image::CameraInfo> cameras,
    int max_features, core::slam::CloudMapper & mapper, const char * logger);
  ~VisualFeed();

  VisualFeed(const VisualFeed &) = delete;
  VisualFeed & operator=(const VisualFeed &) = delete;
  VisualFeed(VisualFeed &&) = delete;
  VisualFeed & operator=(VisualFeed &&) = delete;

  // Enqueue one image for camera `cam`, blocking while that camera's queue is
  // full. `stamp_ns` is the capture stamp read off the message header on the
  // reader side (the frontend needs its frames in stamp order, and the reader
  // is the only place that ordering is guaranteed).
  void push(
    std::size_t cam, std::int64_t stamp_ns, std::string type, std::vector<std::byte> payload);

  // Stop accepting work, let every worker drain its queue, and log each
  // camera's fed/failed image counts. Idempotent.
  void finish();

private:
  void run_worker(std::size_t cam);

  std::vector<std::string> image_topics_;
  core::slam::CloudMapper & mapper_;
  const char * logger_;
  std::vector<std::unique_ptr<core::slam::VisualFrontend>> frontends_;
  std::vector<std::unique_ptr<core::slam::FrameFeedQueue<VisualWorkItem>>> queues_;
  std::vector<std::thread> workers_;
  // Per camera: images enqueued (reader thread only) and images that failed to
  // decode or track (its own worker only, published when that worker joins).
  std::vector<std::int64_t> images_;
  std::vector<std::int64_t> failures_;
  bool finished_ = false;
};

}  // namespace bagwiz::commands

#endif  // COMMANDS__MAP_SLAM_VISUAL_HPP_
