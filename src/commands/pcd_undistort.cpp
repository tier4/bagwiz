// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/pcd_undistort.hpp"

#include "bagwiz/core/bag_inplace.hpp"
#include "bagwiz/core/cdr_walker/value.hpp"
#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/logging.hpp"
#include "bagwiz/core/output_path.hpp"
#include "bagwiz/core/pointcloud/deskew.hpp"
#include "bagwiz/core/pointcloud/point_time.hpp"
#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/core/tf_buffer_loader.hpp"
#include "bagwiz/core/tf_chain.hpp"
#include "bagwiz/core/tf_value_extract.hpp"
#include "bagwiz/core/trajectory.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <tf2/buffer_core.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <algorithm>
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
#include <stdexcept>
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
constexpr const char * kPointCloud2Type = "sensor_msgs/msg/PointCloud2";
constexpr const char * kTfMessageType = "tf2_msgs/msg/TFMessage";
constexpr const char * kOdometryType = "nav_msgs/msg/Odometry";
constexpr const char * kPoseStampedType = "geometry_msgs/msg/PoseStamped";
constexpr const char * kPoseWithCovarianceStampedType =
  "geometry_msgs/msg/PoseWithCovarianceStamped";
constexpr std::chrono::hours kTfBufferCacheTime{24 * 365};
constexpr const char * kDefaultFromFrame = "map";
constexpr const char * kDefaultToFrame = "base_link";

bool is_supported_pose_topic_type(const std::string & type)
{
  return type == kTfMessageType || type == kOdometryType || type == kPoseStampedType ||
         type == kPoseWithCovarianceStampedType;
}

// Outcome of building the --from -> --to trajectory in Pass 1.
struct TrajectoryBuildResult
{
  std::vector<core::TrajectoryPose> trajectory;
  std::string error;  // empty on success

  [[nodiscard]] bool ok() const { return error.empty() && !trajectory.empty(); }
};

// One edge decoded from a TFMessage pose topic, kept alongside the dynamic
// buffer so the sample stamps actually published on `pose_topic` can be told
// apart from edges that only live in tf_static.
struct TfEdge
{
  std::string frame_id;
  std::string child_frame_id;
  std::int64_t stamp_ns = 0;
};

std::string edge_key(const std::string & parent, const std::string & child)
{
  return parent + '\0' + child;
}

// TFMessage pose topic: feed every transform it carries into `buffer` as
// dynamic edges (tf_static is already loaded there), resolve the --from ->
// --to chain, and sample it at every stamp the chain's edges are actually
// published on `pose_topic` — mirrors `traj dump`'s TFMessage path
// (traj.cpp's run_dump_tf_message), but scoped to this one topic since pcd
// undistort never reads a bag's dynamic /tf.
TrajectoryBuildResult build_trajectory_from_tf_message(
  const std::filesystem::path & input_path, const io::TopicInfo & pose_ti, const std::string & from,
  const std::string & to, tf2::BufferCore & buffer)
{
  TrajectoryBuildResult out;
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input_path);
  } catch (const std::exception & e) {
    out.error = std::string("failed to reopen bag for pose topic: ") + e.what();
    return out;
  }
  reader->populate_schemas();
  io::ReadFilter filter;
  filter.topics = {pose_ti.name};
  reader->set_filter(filter);

  auto open = core::decoder::open_decoder(pose_ti);
  if (!open.ok()) {
    out.error = "could not open decoder for pose topic '" + pose_ti.name + "': " + open.error;
    return out;
  }

  std::vector<TfEdge> input_edges;
  io::RawMessage raw;
  try {
    while (reader->next(raw)) {
      if (raw.topic->name != pose_ti.name) {
        continue;
      }
      const auto decoded = open.decoder->decode(raw.payload);
      if (!decoded.ok()) {
        out.error = "failed to decode message on '" + pose_ti.name + "': " + decoded.error;
        return out;
      }
      for (const auto & t : core::extract_tf_message(*decoded.value)) {
        buffer.setTransform(t, "bagwiz", /*is_static=*/false);
        const std::int64_t ns = static_cast<std::int64_t>(t.header.stamp.sec) * 1'000'000'000LL +
                                static_cast<std::int64_t>(t.header.stamp.nanosec);
        input_edges.push_back({t.header.frame_id, t.child_frame_id, ns});
      }
    }
  } catch (const std::exception & e) {
    out.error = "error reading pose topic '" + pose_ti.name + "': " + e.what();
    return out;
  }

  if (input_edges.empty()) {
    out.error = "pose topic '" + pose_ti.name + "' carried no TransformStamped entries";
    return out;
  }

  // Resolve the chain using the first published edge's stamp: parent/child
  // linkage in tf2 is fixed for a frame's whole life, so any populated stamp
  // works — this just needs to land inside what was just set above.
  const tf2::TimePoint resolve_tp{std::chrono::nanoseconds(input_edges.front().stamp_ns)};
  const auto chain = core::resolve_chain(buffer, from, to, resolve_tp);
  if (chain.empty()) {
    out.error = "no TF path from --from '" + from + "' to --to '" + to + "' (checked '" +
                pose_ti.name + "' + the bag's static TF)";
    return out;
  }
  const auto path_edges = core::chain_to_edges(buffer, chain, resolve_tp);
  std::unordered_set<std::string> path_edge_set;
  path_edge_set.reserve(path_edges.size());
  for (const auto & e : path_edges) {
    path_edge_set.insert(edge_key(e.first, e.second));
  }

  std::vector<std::int64_t> sample_stamps;
  for (const auto & ie : input_edges) {
    if (path_edge_set.count(edge_key(ie.frame_id, ie.child_frame_id)) != 0) {
      sample_stamps.push_back(ie.stamp_ns);
    }
  }
  if (sample_stamps.empty()) {
    out.error = "--from '" + from + "' -> --to '" + to +
                "' resolves via static TF, but none of the edges on that path are published on "
                "pose topic '" +
                pose_ti.name + "'";
    return out;
  }
  std::sort(sample_stamps.begin(), sample_stamps.end());
  sample_stamps.erase(std::unique(sample_stamps.begin(), sample_stamps.end()), sample_stamps.end());

  std::int64_t skipped = 0;
  std::string last_skip_reason;
  for (const std::int64_t ns : sample_stamps) {
    const tf2::TimePoint tp{std::chrono::nanoseconds(ns)};
    try {
      const auto tf = buffer.lookupTransform(from, to, tp);
      core::TrajectoryPose p;
      p.timestamp_ns = ns;
      p.tx = tf.transform.translation.x;
      p.ty = tf.transform.translation.y;
      p.tz = tf.transform.translation.z;
      p.qx = tf.transform.rotation.x;
      p.qy = tf.transform.rotation.y;
      p.qz = tf.transform.rotation.z;
      p.qw = tf.transform.rotation.w;
      out.trajectory.push_back(p);
    } catch (const tf2::TransformException & e) {
      ++skipped;
      last_skip_reason = e.what();
    }
  }
  if (out.trajectory.empty()) {
    out.error = "all " + std::to_string(sample_stamps.size()) +
                " sample stamp(s) failed to resolve via lookupTransform; last reason: " +
                (last_skip_reason.empty() ? "(none)" : last_skip_reason);
  }
  return out;
}

// The 3 pose-shaped topic types that carry their own body pose directly
// (as opposed to TFMessage, which is a set of independent edges).
enum class PoseComposeKind { kOdometry, kPoseStamped, kPoseWithCovarianceStamped };

PoseComposeKind pose_compose_kind(const std::string & type)
{
  if (type == kOdometryType) {
    return PoseComposeKind::kOdometry;
  }
  if (type == kPoseStampedType) {
    return PoseComposeKind::kPoseStamped;
  }
  return PoseComposeKind::kPoseWithCovarianceStamped;  // caller validated the type
}

// One decoded sample from a pose / odometry input topic (mirrors traj.cpp's
// PoseSample; kept local since pcd undistort does not share a translation
// unit with traj.cpp). `child_frame` is only set for Odometry.
struct PoseSample
{
  geometry_msgs::msg::PoseStamped pose;
  std::string child_frame;
};

bool decode_pose_sample(
  PoseComposeKind kind, const core::cdr_walker::Value & value, PoseSample & out)
{
  switch (kind) {
    case PoseComposeKind::kPoseStamped: {
      const auto ps = core::extract_pose_stamped_message(value);
      if (!ps.has_value()) {
        return false;
      }
      out.pose = *ps;
      return true;
    }
    case PoseComposeKind::kPoseWithCovarianceStamped: {
      const auto pwc = core::extract_pose_with_covariance_stamped_message(value);
      if (!pwc.has_value()) {
        return false;
      }
      out.pose.header = pwc->header;
      out.pose.pose = pwc->pose.pose;
      return true;
    }
    case PoseComposeKind::kOdometry: {
      const auto odom = core::extract_odometry_message(value);
      if (!odom.has_value()) {
        return false;
      }
      out.pose.header = odom->header;
      out.pose.pose = odom->pose.pose;
      out.child_frame = odom->child_frame_id;
      return true;
    }
  }
  return false;
}

// Odometry / PoseStamped / PoseWithCovarianceStamped pose topic: for each
// message, compose T_from_to = T_from_header * T_header_body * T_body_to,
// bridging into --from / --to via the bag's static TF when the message's own
// frames do not already match. Mirrors traj.cpp's run_dump_pose_topic
// composition (traj.cpp:732-757), except an unresolvable bridge is fatal here
// rather than a per-sample skip: pcd undistort's TF is static-only, so a
// failure is a configuration problem, not transient sensor noise.
TrajectoryBuildResult build_trajectory_from_pose_topic(
  const std::filesystem::path & input_path, const io::TopicInfo & pose_ti, PoseComposeKind kind,
  const std::string & from, const std::string & to, tf2::BufferCore & buffer)
{
  TrajectoryBuildResult out;
  const bool is_odom = (kind == PoseComposeKind::kOdometry);

  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input_path);
  } catch (const std::exception & e) {
    out.error = std::string("failed to reopen bag for pose topic: ") + e.what();
    return out;
  }
  reader->populate_schemas();
  io::ReadFilter filter;
  filter.topics = {pose_ti.name};
  reader->set_filter(filter);

  auto open = core::decoder::open_decoder(pose_ti);
  if (!open.ok()) {
    out.error = "could not open decoder for pose topic '" + pose_ti.name + "': " + open.error;
    return out;
  }

  io::RawMessage raw;
  try {
    while (reader->next(raw)) {
      if (raw.topic->name != pose_ti.name) {
        continue;
      }
      const auto decoded = open.decoder->decode(raw.payload);
      if (!decoded.ok()) {
        out.error = "failed to decode message on '" + pose_ti.name + "': " + decoded.error;
        return out;
      }
      PoseSample sample;
      if (!decode_pose_sample(kind, *decoded.value, sample)) {
        continue;  // unparsable sample; tolerated like traj dump's skip
      }
      if (sample.pose.header.frame_id.empty()) {
        out.error = "pose topic '" + pose_ti.name + "': message has empty header.frame_id";
        return out;
      }
      if (is_odom && sample.child_frame.empty()) {
        out.error = "pose topic '" + pose_ti.name + "': Odometry message has empty child_frame_id";
        return out;
      }

      const std::string & header_frame = sample.pose.header.frame_id;
      const std::int64_t ns =
        static_cast<std::int64_t>(sample.pose.header.stamp.sec) * 1'000'000'000LL +
        static_cast<std::int64_t>(sample.pose.header.stamp.nanosec);

      std::optional<geometry_msgs::msg::Transform> from_header;
      if (from != header_frame) {
        if (!core::missing_frames(buffer, from, header_frame).empty()) {
          out.error = "--from '" + from + "' has no static TF chain to pose topic '" +
                      pose_ti.name + "'s frame '" + header_frame + "'";
          return out;
        }
        try {
          from_header = buffer.lookupTransform(from, header_frame, tf2::TimePointZero).transform;
        } catch (const std::exception & e) {
          out.error =
            "--from '" + from + "' -> '" + header_frame + "' TF lookup failed: " + e.what();
          return out;
        }
      }
      std::optional<geometry_msgs::msg::Transform> body_to;
      if (is_odom && to != sample.child_frame) {
        if (!core::missing_frames(buffer, sample.child_frame, to).empty()) {
          out.error = "--to '" + to + "' has no static TF chain from Odometry child frame '" +
                      sample.child_frame + "'";
          return out;
        }
        try {
          body_to = buffer.lookupTransform(sample.child_frame, to, tf2::TimePointZero).transform;
        } catch (const std::exception & e) {
          out.error =
            "'" + sample.child_frame + "' -> --to '" + to + "' TF lookup failed: " + e.what();
          return out;
        }
      }

      const auto composed = core::compose_trajectory_pose(from_header, sample.pose.pose, body_to);
      core::TrajectoryPose p;
      p.timestamp_ns = ns;
      p.tx = composed.position.x;
      p.ty = composed.position.y;
      p.tz = composed.position.z;
      p.qx = composed.orientation.x;
      p.qy = composed.orientation.y;
      p.qz = composed.orientation.z;
      p.qw = composed.orientation.w;
      out.trajectory.push_back(p);
    }
  } catch (const std::exception & e) {
    out.error = "error reading pose topic '" + pose_ti.name + "': " + e.what();
    return out;
  }

  if (out.trajectory.empty()) {
    out.error = "no poses decoded from pose topic '" + pose_ti.name + "'";
  }
  return out;
}

// find_point_time_field only reads `.fields`, so a header-only peek (no point
// data copy) is enough to tell whether a --pcd topic has a usable per-point
// time field. A field that is present by name but whose declared offset runs
// past `point_step` is treated the same as an absent field: deskew_pointcloud2
// applies the identical bounds check (its own `fits()` guard) and silently
// falls back to "no usable time" rather than erroring, which would otherwise
// let an out-of-bounds field slip past this upfront, required-time-field
// check and get passed through un-deskewed with no warning.
bool cloud_has_usable_point_time(
  const std::vector<core::pointcloud::PointField> & fields, std::uint32_t point_step)
{
  core::pointcloud::PointCloud2 shim;
  shim.fields = fields;
  const auto field = core::pointcloud::find_point_time_field(shim);
  if (!field.has_value()) {
    return false;
  }
  return static_cast<std::size_t>(field->offset) +
           core::pointcloud::datatype_size(field->datatype) <=
         point_step;
}

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

// Shared state for the parallel reader / worker pool / collector pipeline.
struct ParallelContext
{
  std::mutex mutex;
  std::condition_variable cv;
  std::queue<DeskewJob> job_queue;
  std::map<std::size_t, OutputItem> completed;
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
      auto res =
        core::pointcloud::deskew_pointcloud2(*parsed.cloud, t_ref, trajectory, job.extrinsic);
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
  const std::unordered_map<std::string, std::optional<geometry_msgs::msg::Transform>> & extrinsics,
  std::span<const core::TrajectoryPose> trajectory, int num_threads, std::uint64_t & total_clouds)
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
        {
          std::unique_lock lock(ctx.mutex);
          ctx.cv.wait(lock, [&] {
            auto it = ctx.completed.find(ctx.next_output_seq);
            return (it != ctx.completed.end()) ||
                   (ctx.reader_done && ctx.next_output_seq == ctx.total_submitted);
          });

          if (ctx.reader_done && ctx.next_output_seq == ctx.total_submitted) {
            break;
          }

          auto it = ctx.completed.find(ctx.next_output_seq);
          if (it == ctx.completed.end()) {
            continue;
          }
          item = std::move(it->second);
          ctx.completed.erase(it);
          ++ctx.next_output_seq;
          --ctx.in_flight;
        }
        ctx.cv.notify_all();

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

      try {
        writer.close();
      } catch (const std::exception & e) {
        BAGWIZ_LOG_ERROR(kLogger, "Writer close() failed: %s", e.what());
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

}  // namespace

int run_pcd_undistort(const PcdUndistortArgs & args)
{
  // ---- validate arguments -------------------------------------------------
  if (args.pcd_topics.empty()) {
    BAGWIZ_LOG_ERROR(kLogger, "pcd undistort: --pcd needs at least 1 topic");
    return 1;
  }
  const std::string from = args.from_frame.value_or(kDefaultFromFrame);
  const std::string to = args.to_frame.value_or(kDefaultToFrame);

  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(args.input_path);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", args.input_path.c_str(), e.what());
    return 1;
  }
  reader->populate_schemas();

  const io::TopicInfo * pose_ti = nullptr;
  std::unordered_map<std::string, const io::TopicInfo *> pcd_info;
  for (const auto & t : reader->topics()) {
    if (t.name == args.pose_topic) {
      pose_ti = &t;
    }
    if (
      std::find(args.pcd_topics.begin(), args.pcd_topics.end(), t.name) != args.pcd_topics.end()) {
      pcd_info.emplace(t.name, &t);
    }
  }
  if (pose_ti == nullptr) {
    BAGWIZ_LOG_ERROR(
      kLogger, "Topic '%s' is not present in %s", args.pose_topic.c_str(), args.input_path.c_str());
    return 1;
  }
  if (!is_supported_pose_topic_type(pose_ti->type)) {
    BAGWIZ_LOG_ERROR(
      kLogger, "Topic '%s' has unsupported type '%s'. Supported: %s, %s, %s, %s.",
      args.pose_topic.c_str(), pose_ti->type.c_str(), kTfMessageType, kOdometryType,
      kPoseStampedType, kPoseWithCovarianceStampedType);
    return 1;
  }
  for (const auto & topic : args.pcd_topics) {
    const auto it = pcd_info.find(topic);
    if (it == pcd_info.end()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Topic '%s' is not present in %s", topic.c_str(), args.input_path.c_str());
      return 1;
    }
    if (it->second->type != kPointCloud2Type) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Topic '%s' is %s, expected %s", topic.c_str(), it->second->type.c_str(),
        kPointCloud2Type);
      return 1;
    }
  }

  // ---- Pass 1: build the --from -> --to trajectory ------------------------
  tf2::BufferCore buffer{kTfBufferCacheTime};
  if (const auto error = core::load_static_tf_buffer(args.input_path, buffer); error.has_value()) {
    // load_static_tf_buffer is a shared, caller-neutral helper (it names no
    // command's flags), so its detail is always safe to forward here.
    BAGWIZ_LOG_ERROR(
      kLogger,
      "pcd undistort: could not load the bag's static TF (needed to resolve --from '%s' / --to "
      "'%s' and any --pcd topic's sensor extrinsic); detail: %s",
      from.c_str(), to.c_str(), error->c_str());
    return 1;
  }

  TrajectoryBuildResult built =
    (pose_ti->type == kTfMessageType)
      ? build_trajectory_from_tf_message(args.input_path, *pose_ti, from, to, buffer)
      : build_trajectory_from_pose_topic(
          args.input_path, *pose_ti, pose_compose_kind(pose_ti->type), from, to, buffer);
  if (!built.ok()) {
    BAGWIZ_LOG_ERROR(
      kLogger, "pcd undistort: could not resolve --from '%s' -> --to '%s' from pose topic '%s': %s",
      from.c_str(), to.c_str(), args.pose_topic.c_str(), built.error.c_str());
    return 1;
  }
  std::vector<core::TrajectoryPose> trajectory = std::move(built.trajectory);
  std::sort(trajectory.begin(), trajectory.end(), [](const auto & a, const auto & b) {
    return a.timestamp_ns < b.timestamp_ns;
  });

  // ---- peek each --pcd topic's first cloud (frame_id + time field) --------
  struct PcdTopicState
  {
    std::string frame_id;
    bool has_time = false;
  };
  std::unordered_map<std::string, PcdTopicState> pcd_state;
  {
    std::unique_ptr<io::BagReader> preader;
    try {
      preader = io::open_read(args.input_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to reopen %s: %s", args.input_path.c_str(), e.what());
      return 1;
    }
    io::ReadFilter filter;
    filter.topics = args.pcd_topics;
    preader->set_filter(filter);
    std::unordered_set<std::string> pending(args.pcd_topics.begin(), args.pcd_topics.end());
    io::RawMessage raw;
    try {
      while (!pending.empty() && preader->next(raw)) {
        const auto it = pending.find(raw.topic->name);
        if (it == pending.end()) {
          continue;  // already peeked this topic's first message
        }
        const auto header = core::pointcloud::parse_pointcloud2_header(raw.payload);
        if (!header.ok()) {
          BAGWIZ_LOG_ERROR(
            kLogger, "pcd undistort: could not parse the first message on --pcd topic '%s': %s",
            raw.topic->name.c_str(), header.error.c_str());
          return 1;
        }
        PcdTopicState st;
        st.frame_id = header.header->frame_id;
        st.has_time = cloud_has_usable_point_time(header.header->fields, header.header->point_step);
        pcd_state.emplace(raw.topic->name, st);
        pending.erase(it);
      }
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "read error peeking --pcd topics: %s", e.what());
      return 1;
    }
  }
  for (const auto & topic : args.pcd_topics) {
    const auto it = pcd_state.find(topic);
    if (it == pcd_state.end()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "pcd undistort: --pcd topic '%s' has no decodable PointCloud2 message",
        topic.c_str());
      return 1;
    }
    if (!it->second.has_time) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "pcd undistort: --pcd topic '%s' has no per-point time field (checked t / time / "
        "time_stamp / timestamp); pcd undistort requires per-point time to deskew",
        topic.c_str());
      return 1;
    }
  }

  // ---- resolve each --pcd topic's extrinsic E = T_to_cloud -----------------
  std::unordered_map<std::string, std::optional<geometry_msgs::msg::Transform>> extrinsics;
  for (const auto & topic : args.pcd_topics) {
    const std::string & frame_id = pcd_state.at(topic).frame_id;
    std::optional<geometry_msgs::msg::Transform> extrinsic;
    if (frame_id != to) {
      if (!core::missing_frames(buffer, to, frame_id).empty()) {
        BAGWIZ_LOG_ERROR(
          kLogger, "pcd undistort: --to '%s' has no static TF chain to --pcd topic '%s' frame '%s'",
          to.c_str(), topic.c_str(), frame_id.c_str());
        return 1;
      }
      try {
        extrinsic = buffer.lookupTransform(to, frame_id, tf2::TimePointZero).transform;
      } catch (const std::exception & e) {
        BAGWIZ_LOG_ERROR(
          kLogger, "pcd undistort: --to '%s' -> --pcd topic '%s' frame '%s' TF lookup failed: %s",
          to.c_str(), topic.c_str(), frame_id.c_str(), e.what());
        return 1;
      }
    }
    extrinsics.emplace(topic, extrinsic);
  }

  // ---- Pass 2: copy-through + deskew ---------------------------------------
  const std::unordered_set<std::string> pcd_set(args.pcd_topics.begin(), args.pcd_topics.end());
  std::uint64_t total_clouds = 0;

  const unsigned int hardware = std::thread::hardware_concurrency();
  const int requested = args.threads.value_or(0);
  int num_threads = (requested <= 0) ? static_cast<int>(hardware ? hardware : 1) : requested;
  if (hardware > 0 && num_threads > static_cast<int>(hardware)) {
    num_threads = static_cast<int>(hardware);
  }

  const auto execute_pass = [&](io::BagWriter & writer) -> int {
    for (const auto & t : reader->topics()) {
      writer.declare_topic(t);
    }

    std::unique_ptr<io::BagReader> rd;
    try {
      rd = io::open_read(args.input_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to reopen %s: %s", args.input_path.c_str(), e.what());
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
      auto res =
        core::pointcloud::deskew_pointcloud2(*parsed.cloud, t_ref, trajectory, extrinsics.at(name));
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

    try {
      writer.close();
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Writer close() failed: %s", e.what());
      return 1;
    }
    return 0;
  };

  // ---- dispatch: -o (new bag) vs in-place ----------------------------------
  int status = 0;
  if (args.output_path.has_value()) {
    if (const auto r = core::prepare_output_path(*args.output_path, args.overwrite); !r.ok) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", r.error.c_str());
      return 1;
    }
    std::unique_ptr<io::BagWriter> writer;
    try {
      const auto copts = io::create_options_inheriting_format(args.input_path, *args.output_path);
      writer = io::open_write(*args.output_path, copts);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open output writer: %s", e.what());
      return 1;
    }
    if (num_threads <= 1) {
      status = execute_pass(*writer);
    } else {
      status = run_parallel_undistort_pass(
        *writer, *reader, args.input_path, pcd_set, extrinsics, trajectory, num_threads,
        total_clouds);
    }
  } else {
    const auto inplace_copts = io::create_options_preserving_storage(args.input_path);
    if (inplace_copts.format == io::Format::Auto) {
      BAGWIZ_LOG_ERROR(
        kLogger, "pcd undistort: could not detect storage format of input bag '%s'.",
        args.input_path.c_str());
      return 1;
    }
    try {
      core::write_bag_inplace(args.input_path, [&](const std::filesystem::path & tmp) {
        auto writer = io::open_write(tmp, inplace_copts);
        if (num_threads <= 1) {
          status = execute_pass(*writer);
        } else {
          status = run_parallel_undistort_pass(
            *writer, *reader, args.input_path, pcd_set, extrinsics, trajectory, num_threads,
            total_clouds);
        }
        if (status != 0) {
          throw std::runtime_error("pcd undistort: pass failed; aborting in-place swap");
        }
      });
    } catch (const std::exception & e) {
      // cppcheck-suppress knownConditionTrueFalse  // status is assigned inside the lambda above
      if (status != 0) {
        return status;
      }
      BAGWIZ_LOG_ERROR(kLogger, "In-place swap failed: %s", e.what());
      return 1;
    }
  }
  if (status != 0) {
    return status;
  }

  BAGWIZ_LOG_INFO(
    kLogger, "pcd undistort: deskewed %" PRIu64 " cloud(s) across %zu topic(s)", total_clouds,
    args.pcd_topics.size());
  return 0;
}

}  // namespace bagwiz::commands
