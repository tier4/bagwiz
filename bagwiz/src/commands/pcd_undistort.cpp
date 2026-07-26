// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/pcd_undistort.hpp"

#include "bagwiz/core/bag/rewrite.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/pointcloud/deskew.hpp"
#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/core/tf/trajectory.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/bag_open.hpp"
#include "pcd_undistort_common.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <tf2/buffer_core.hpp>

#include <geometry_msgs/msg/transform.hpp>

#include <chrono>
#include <cinttypes>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <span>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bagwiz::commands
{
namespace
{

constexpr const char * kLogger = "bagwiz.cmd.pcd";
constexpr std::chrono::hours kTfBufferCacheTime{24 * 365};
constexpr const char * kDefaultRefFrame = "map";
constexpr const char * kDefaultOfFrame = "base_link";

// One PointCloud2 message handed off from the bag reader to a worker thread.
struct DeskewJob
{
  std::size_t seq;
  std::string topic;
  std::int64_t timestamp_ns;
  std::vector<std::byte> payload;
  std::optional<geometry_msgs::msg::Transform> extrinsic;
};

// One output message waiting in the in-order completion map for the collector.
struct OutputItem
{
  std::string topic;
  std::int64_t timestamp_ns;
  std::vector<std::byte> payload;
  std::optional<std::string> error;
};

// Rendezvous slot for a copy-through message written without a payload copy.
// When nothing is in flight, the reader hands the collector the borrowed
// payload span (valid until the reader's next next() call) instead of copying
// it into the completion map; the reader blocks until the collector sets
// `done`, so the span never outlives its backing store.
struct DirectWrite
{
  std::string topic;
  std::int64_t timestamp_ns;
  std::span<const std::byte> payload;
  bool done = false;
};

// Shared state for the parallel reader / worker pool / collector pipeline.
struct ParallelContext
{
  std::mutex mutex;
  std::condition_variable cv;
  std::queue<DeskewJob> job_queue;
  std::map<std::size_t, OutputItem> completed;
  std::optional<DirectWrite> direct;
  std::size_t next_output_seq = 0;
  std::size_t total_submitted = 0;
  std::size_t in_flight = 0;
  std::size_t max_in_flight = 0;
  bool stop = false;
  bool reader_done = false;
};

// Parse, deskew, and serialize a single cloud.  Runs on a worker thread and
// only touches local state plus the read-only trajectory span.
OutputItem process_deskew_job(DeskewJob job, std::span<const core::TrajectoryPose> trajectory)
{
  OutputItem item;
  item.topic = job.topic;
  item.timestamp_ns = job.timestamp_ns;

  try {
    auto parsed = core::pointcloud::parse_pointcloud2(job.payload);
    if (!parsed.ok()) {
      BAGWIZ_LOG_WARN(
        kLogger, "pcd undistort: skipping undecodable cloud on '%s': %s", job.topic.c_str(),
        parsed.error.c_str());
      item.payload = std::move(job.payload);
    } else {
      const std::int64_t t_ref = parsed.cloud->timestamp_ns;
      auto res = core::pointcloud::deskew_pointcloud2(
        std::move(*parsed.cloud), t_ref, trajectory, job.extrinsic);
      if (!res.ok()) {
        item.error = res.error;
      } else {
        if (res.points_deskewed == 0 && res.points_total > 0) {
          BAGWIZ_LOG_WARN(
            kLogger,
            "pcd undistort: cloud on '%s' had nothing deskewed of %" PRIu64
            " point(s) (no_time=%" PRIu64 ", no_pose=%" PRIu64 ", nonfinite=%" PRIu64
            "); passed through un-deskewed",
            job.topic.c_str(), res.points_total, res.points_no_time, res.points_no_pose,
            res.points_nonfinite);
        }
        item.payload = core::pointcloud::serialize_pointcloud2(*res.cloud);
      }
    }
  } catch (const std::exception & e) {
    item.error = std::string("exception: ") + e.what();
  }
  return item;
}

// Parallel version of Pass 2.  One reader thread, one collector thread that
// alone calls writer.write(), and a fixed-size std::jthread worker pool that
// deskews PointCloud2 messages.  Non-pcd messages bypass the job queue and go
// straight into the in-order completion map.  Output message order is identical
// to the synchronous path.
int run_parallel_undistort_pass(
  io::BagWriter & writer, const io::BagReader & topic_reader,
  const std::filesystem::path & input_path, const std::unordered_set<std::string> & pcd_set,
  const ExtrinsicMap & extrinsics, std::span<const core::TrajectoryPose> trajectory,
  int num_threads, std::uint64_t & total_clouds)
{
  for (const auto & t : topic_reader.topics()) {
    writer.declare_topic(t);
  }

  std::unique_ptr<io::BagReader> rd;
  try {
    rd = io::open_read(input_path);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Failed to reopen %s: %s", input_path.c_str(), e.what());
    return 1;
  }
  rd->populate_schemas();

  ParallelContext ctx;
  ctx.max_in_flight = static_cast<std::size_t>(num_threads) * 3;

  int collector_status = 0;

  auto worker = [&]() {
    while (true) {
      DeskewJob job;
      {
        std::unique_lock lock(ctx.mutex);
        ctx.cv.wait(lock, [&] { return ctx.stop || !ctx.job_queue.empty(); });
        if (ctx.stop && ctx.job_queue.empty()) {
          return;
        }
        job = std::move(ctx.job_queue.front());
        ctx.job_queue.pop();
      }

      const std::size_t seq = job.seq;
      OutputItem item = process_deskew_job(std::move(job), trajectory);

      {
        std::lock_guard lock(ctx.mutex);
        ctx.completed.emplace(seq, std::move(item));
      }
      ctx.cv.notify_all();
    }
  };

  auto collector = [&]() {
    try {
      while (true) {
        OutputItem item;
        bool wrote_direct = false;
        {
          std::unique_lock lock(ctx.mutex);
          ctx.cv.wait(lock, [&] {
            auto it = ctx.completed.find(ctx.next_output_seq);
            // A served (done) rendezvous stays visible until the reader
            // resets it; it must not be served again.
            const bool direct_pending = ctx.direct.has_value() && !ctx.direct->done;
            return direct_pending || (it != ctx.completed.end()) ||
                   (ctx.reader_done && ctx.next_output_seq == ctx.total_submitted);
          });

          if (ctx.direct.has_value() && !ctx.direct->done) {
            // Serve the rendezvous: the reader is blocked until `done`, so the
            // borrowed span is still valid. writer.write stays on this thread
            // (the BagWriter single-thread contract).
            const std::string topic = ctx.direct->topic;
            const std::int64_t timestamp_ns = ctx.direct->timestamp_ns;
            const std::span<const std::byte> payload = ctx.direct->payload;
            lock.unlock();
            writer.write(topic, timestamp_ns, payload);
            lock.lock();
            ctx.direct->done = true;
            wrote_direct = true;
          } else if (ctx.reader_done && ctx.next_output_seq == ctx.total_submitted) {
            break;
          } else {
            auto it = ctx.completed.find(ctx.next_output_seq);
            if (it == ctx.completed.end()) {
              continue;
            }
            item = std::move(it->second);
            ctx.completed.erase(it);
            ++ctx.next_output_seq;
            --ctx.in_flight;
          }
        }
        ctx.cv.notify_all();
        if (wrote_direct) {
          continue;
        }

        if (item.error.has_value()) {
          BAGWIZ_LOG_ERROR(
            kLogger, "pcd undistort: deskew failed on '%s': %s", item.topic.c_str(),
            item.error->c_str());
          collector_status = 1;
          {
            std::lock_guard lock(ctx.mutex);
            ctx.stop = true;
          }
          ctx.cv.notify_all();
          try {
            writer.close();
          } catch (...) {
            // A writer close error is secondary to the deskew error already reported.
          }
          return;
        }

        writer.write(item.topic, item.timestamp_ns, item.payload);
      }

      if (!io::close_writer_or_log(writer, kLogger)) {
        collector_status = 1;
      }
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "pcd undistort: collector error: %s", e.what());
      collector_status = 1;
      {
        std::lock_guard lock(ctx.mutex);
        ctx.stop = true;
      }
      ctx.cv.notify_all();
    }
  };

  std::vector<std::jthread> workers;
  workers.reserve(num_threads);
  for (int i = 0; i < num_threads; ++i) {
    workers.emplace_back(worker);
  }
  std::jthread collector_thread(collector);

  io::RawMessage raw;
  try {
    while (rd->next(raw)) {
      const std::string & name = raw.topic->name;
      const bool is_pcd = pcd_set.count(name) != 0;

      if (is_pcd) {
        DeskewJob job;
        job.topic = name;
        job.timestamp_ns = raw.timestamp_ns;
        job.payload.assign(raw.payload.begin(), raw.payload.end());
        job.extrinsic = extrinsics.at(name);

        std::unique_lock lock(ctx.mutex);
        ctx.cv.wait(lock, [&] { return ctx.in_flight < ctx.max_in_flight || ctx.stop; });
        if (ctx.stop) {
          break;
        }
        job.seq = ctx.total_submitted++;
        ++ctx.in_flight;
        ctx.job_queue.push(std::move(job));
        ++total_clouds;
        lock.unlock();
        ctx.cv.notify_all();
      } else {
        // Copy-through message. When the pipeline has drained (nothing in
        // flight — which implies the queue and completion map are empty and
        // every submitted item has been popped by the collector), this message
        // is exactly next in output order: hand the collector the borrowed
        // payload span for an immediate write instead of copying it into the
        // completion map. The reader blocks until the write completes, so the
        // span stays valid. in_flight is only ever incremented by this reader
        // thread, so the check is race-free.
        bool used_fast_path = false;
        {
          std::unique_lock lock(ctx.mutex);
          if (!ctx.stop && ctx.in_flight == 0) {
            ctx.direct = DirectWrite{name, raw.timestamp_ns, raw.payload, false};
            ctx.cv.notify_all();
            ctx.cv.wait(lock, [&] { return ctx.direct->done || ctx.stop; });
            const bool wrote = ctx.direct->done;
            ctx.direct.reset();
            used_fast_path = true;
            if (!wrote) {
              break;  // stop requested (collector error)
            }
          }
        }
        if (used_fast_path) {
          continue;
        }

        OutputItem item;
        item.topic = name;
        item.timestamp_ns = raw.timestamp_ns;
        item.payload.assign(raw.payload.begin(), raw.payload.end());

        std::unique_lock lock(ctx.mutex);
        ctx.cv.wait(lock, [&] { return ctx.in_flight < ctx.max_in_flight || ctx.stop; });
        if (ctx.stop) {
          break;
        }
        const std::size_t seq = ctx.total_submitted++;
        ++ctx.in_flight;
        ctx.completed.emplace(seq, std::move(item));
        lock.unlock();
        ctx.cv.notify_all();
      }
    }
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "pcd undistort: read error: %s", e.what());
    {
      std::lock_guard lock(ctx.mutex);
      ctx.reader_done = true;
      ctx.stop = true;
    }
    ctx.cv.notify_all();
    for (auto & t : workers) {
      t.join();
    }
    ctx.cv.notify_all();
    collector_thread.join();
    return 1;
  }

  {
    std::lock_guard lock(ctx.mutex);
    ctx.reader_done = true;
    ctx.stop = true;
  }
  ctx.cv.notify_all();

  for (auto & t : workers) {
    t.join();
  }

  ctx.cv.notify_all();
  collector_thread.join();

  return collector_status;
}

// Synchronous version of Pass 2 (num_threads <= 1): same declare + reopen +
// stream shape as run_parallel_undistort_pass, but deskews each cloud inline
// on the reader thread. Output message order is trivially the bag's order.
int run_sync_undistort_pass(
  io::BagWriter & writer, const io::BagReader & topic_reader,
  const std::filesystem::path & input_path, const std::unordered_set<std::string> & pcd_set,
  const ExtrinsicMap & extrinsics, std::span<const core::TrajectoryPose> trajectory,
  std::uint64_t & total_clouds)
{
  for (const auto & t : topic_reader.topics()) {
    writer.declare_topic(t);
  }

  std::unique_ptr<io::BagReader> rd;
  try {
    rd = io::open_read(input_path);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Failed to reopen %s: %s", input_path.c_str(), e.what());
    return 1;
  }
  rd->populate_schemas();

  io::RawMessage raw;
  while (rd->next(raw)) {
    const std::string & name = raw.topic->name;
    if (pcd_set.count(name) == 0) {
      writer.write(name, raw.timestamp_ns, raw.payload);
      continue;
    }
    auto parsed = core::pointcloud::parse_pointcloud2(raw.payload);
    if (!parsed.ok()) {
      BAGWIZ_LOG_WARN(
        kLogger, "pcd undistort: skipping undecodable cloud on '%s': %s", name.c_str(),
        parsed.error.c_str());
      writer.write(name, raw.timestamp_ns, raw.payload);
      continue;
    }
    const std::int64_t t_ref = parsed.cloud->timestamp_ns;  // header.stamp
    auto res = core::pointcloud::deskew_pointcloud2(
      std::move(*parsed.cloud), t_ref, trajectory, extrinsics.at(name));
    if (!res.ok()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "pcd undistort: deskew failed on '%s': %s", name.c_str(), res.error.c_str());
      return 1;
    }
    // The upfront per-topic check only guarantees the FIRST cloud on this
    // topic had a usable time field; a heterogeneous stream could still
    // hand deskew_pointcloud2 a later cloud that ends up moving nothing
    // (no usable time/pose/finite xyz on any point). ok() is still true —
    // the cloud passes through verbatim by design — but that must not be
    // silent, or a bug upstream of this topic could go unnoticed.
    if (res.points_deskewed == 0 && res.points_total > 0) {
      BAGWIZ_LOG_WARN(
        kLogger,
        "pcd undistort: cloud on '%s' had nothing deskewed of %" PRIu64
        " point(s) (no_time=%" PRIu64 ", no_pose=%" PRIu64 ", nonfinite=%" PRIu64
        "); passed through un-deskewed",
        name.c_str(), res.points_total, res.points_no_time, res.points_no_pose,
        res.points_nonfinite);
    }
    const auto payload = core::pointcloud::serialize_pointcloud2(*res.cloud);
    writer.write(name, raw.timestamp_ns, payload);
    ++total_clouds;
  }

  if (!io::close_writer_or_log(writer, kLogger)) {
    return 1;
  }
  return 0;
}

// Run Pass 2 through the shared -o vs in-place rewrite dispatch, picking the
// sync or parallel pass by thread count. Unlike the other rewrite commands,
// pcd undistort keeps the storage default (zstd) for MCAP compression rather
// than forcing "none".
int dispatch_undistort_pass(
  const PcdUndistortArgs & args, const io::BagReader & topic_reader,
  const std::unordered_set<std::string> & pcd_set, const ExtrinsicMap & extrinsics,
  std::span<const core::TrajectoryPose> trajectory, int num_threads, std::uint64_t & total_clouds)
{
  core::BagRewriteOptions rewrite_opts;
  rewrite_opts.logger = kLogger;
  rewrite_opts.format_unknown_error =
    "pcd undistort: could not detect storage format of input bag '%s'.";
  rewrite_opts.pass_failed_error = "pcd undistort: pass failed; aborting in-place swap";
  rewrite_opts.inherit_output_format = true;
  rewrite_opts.disable_mcap_compression = false;
  return core::run_bag_rewrite(
    args.input_path, args.output_path, args.overwrite, rewrite_opts,
    [&](const io::WriterFactory & factory) {
      auto writer = io::open_write_or_log(factory, kLogger);
      if (!writer) {
        return 1;
      }
      if (num_threads <= 1) {
        return run_sync_undistort_pass(
          *writer, topic_reader, args.input_path, pcd_set, extrinsics, trajectory, total_clouds);
      }
      return run_parallel_undistort_pass(
        *writer, topic_reader, args.input_path, pcd_set, extrinsics, trajectory, num_threads,
        total_clouds);
    });
}

}  // namespace

int run_pcd_undistort(const PcdUndistortArgs & args)
{
  // ---- validate arguments + topics ------------------------------------------
  if (args.pcd_topics.empty()) {
    BAGWIZ_LOG_ERROR(kLogger, "pcd undistort: --pcd needs at least 1 topic");
    return 1;
  }
  const std::string ref = args.ref_frame.value_or(kDefaultRefFrame);
  const std::string of = args.of_frame.value_or(kDefaultOfFrame);

  auto reader = io::open_read_or_log(args.input_path, kLogger);
  if (!reader) {
    return 1;
  }
  reader->populate_schemas();
  const io::TopicInfo * pose_ti =
    validate_undistort_topics(*reader, args.pose_topic, args.pcd_topics, args.input_path, kLogger);
  if (pose_ti == nullptr) {
    return 1;
  }

  // ---- Pass 1: static TF + the --of -> --ref trajectory ---------------------
  tf2::BufferCore buffer{kTfBufferCacheTime};
  auto built = build_sorted_of_ref_trajectory(args.input_path, *pose_ti, ref, of, buffer, kLogger);
  if (!built.ok()) {
    return 1;
  }
  const std::vector<core::TrajectoryPose> trajectory = std::move(built.trajectory);

  // ---- peek + validate each --pcd topic's first cloud, then extrinsics ------
  const auto pcd_state = peek_pcd_topic_states(args.input_path, args.pcd_topics, kLogger);
  if (!pcd_state.has_value() || !validate_pcd_topic_states(args.pcd_topics, *pcd_state, kLogger)) {
    return 1;
  }
  const auto extrinsics = resolve_pcd_extrinsics(buffer, of, args.pcd_topics, *pcd_state, kLogger);
  if (!extrinsics.has_value()) {
    return 1;
  }

  // ---- Pass 2: copy-through + deskew (-o vs in-place shared dispatch) -------
  const std::unordered_set<std::string> pcd_set(args.pcd_topics.begin(), args.pcd_topics.end());
  std::uint64_t total_clouds = 0;
  const int num_threads =
    resolve_num_threads(args.threads.value_or(8), std::thread::hardware_concurrency());
  const int status = dispatch_undistort_pass(
    args, *reader, pcd_set, *extrinsics, trajectory, num_threads, total_clouds);
  if (status != 0) {
    return status;
  }

  BAGWIZ_LOG_INFO(
    kLogger, "pcd undistort: deskewed %" PRIu64 " cloud(s) across %zu topic(s)", total_clouds,
    args.pcd_topics.size());
  return 0;
}

}  // namespace bagwiz::commands
