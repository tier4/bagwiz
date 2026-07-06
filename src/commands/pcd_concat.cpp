// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/pcd_concat.hpp"

#include "bagwiz/core/bag_inplace.hpp"
#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/duration_parse.hpp"
#include "bagwiz/core/logging.hpp"
#include "bagwiz/core/output_path.hpp"
#include "bagwiz/core/pointcloud/cloud_concat.hpp"
#include "bagwiz/core/pointcloud/cloud_transform.hpp"
#include "bagwiz/core/pointcloud/concat_sync.hpp"
#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/core/tf_chain.hpp"
#include "bagwiz/core/tf_value_extract.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <tf2/buffer_core.hpp>
#include <tf2/time.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
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
constexpr std::string_view kTfStaticSuffix = "tf_static";
constexpr std::chrono::hours kTfBufferCacheTime{24 * 365};
constexpr std::int64_t kDefaultToleranceNs = 50'000'000;  // 50 ms fallback
constexpr const char * kDefaultFrame = "base_link";       // --frame default

bool is_static_tf_topic(std::string_view topic_name)
{
  if (topic_name.size() < kTfStaticSuffix.size()) {
    return false;
  }
  return topic_name.compare(
           topic_name.size() - kTfStaticSuffix.size(), kTfStaticSuffix.size(), kTfStaticSuffix) ==
         0;
}

// geometry_msgs quaternion (x,y,z,w) + translation -> RigidTransform (row-major
// rotation matrix). p_target = R * p_source + t.
core::pointcloud::RigidTransform to_rigid(const geometry_msgs::msg::TransformStamped & ts)
{
  const double x = ts.transform.rotation.x;
  const double y = ts.transform.rotation.y;
  const double z = ts.transform.rotation.z;
  const double w = ts.transform.rotation.w;
  core::pointcloud::RigidTransform out;
  out.rotation = {1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y),
                  2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
                  2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)};
  out.translation = {
    ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z};
  return out;
}

// Build a tf2 buffer from every static TF topic in the bag. (Ported from
// map_slam; promote to a shared core helper when `pcd undistort` also needs it.)
bool build_static_tf_buffer(
  const std::string & input_path, tf2::BufferCore & buffer, std::string & error)
{
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input_path);
  } catch (const std::exception & e) {
    error = std::string("failed to reopen bag for static TF: ") + e.what();
    return false;
  }

  std::vector<std::string> static_topics;
  for (const auto & t : reader->topics()) {
    if (t.type == kTfMessageType && is_static_tf_topic(t.name)) {
      static_topics.push_back(t.name);
    }
  }
  if (static_topics.empty()) {
    error =
      "bag has no static TF topic (…tf_static); cannot resolve the LiDAR extrinsics to --frame";
    return false;
  }

  io::ReadFilter filter;
  filter.topics = static_topics;
  reader->set_filter(filter);

  std::unordered_map<std::string, std::unique_ptr<core::decoder::Decoder>> decoders;
  for (const auto & info : reader->topics()) {
    if (info.type != kTfMessageType || !is_static_tf_topic(info.name)) {
      continue;
    }
    auto open = core::decoder::open_decoder(info);
    if (!open.ok()) {
      error = "could not open decoder for '" + info.name + "': " + open.error;
      return false;
    }
    decoders.emplace(info.name, std::move(open.decoder));
  }

  io::RawMessage raw;
  try {
    while (reader->next(raw)) {
      const auto it = decoders.find(raw.topic->name);
      if (it == decoders.end()) {
        continue;
      }
      const auto decoded = it->second->decode(raw.payload);
      if (!decoded.ok()) {
        error = "failed to decode static TF on '" + raw.topic->name + "': " + decoded.error;
        return false;
      }
      for (const auto & t : core::extract_tf_message(*decoded.value)) {
        buffer.setTransform(t, "bagwiz", true);
      }
    }
  } catch (const std::exception & e) {
    error = std::string("error reading static TF: ") + e.what();
    return false;
  }
  return true;
}

// One input pcd topic's resolved state.
struct TopicState
{
  std::string name;
  std::string frame_id;
  core::pointcloud::RigidTransform extrinsic;  // target(--frame) <- frame_id
  std::int64_t offset_ns = 0;                  // --stamp-offset
  std::vector<std::int64_t> stamps_ns;         // Pass-A collected header stamps
};

std::int64_t median_period_ns(const std::vector<std::int64_t> & stamps)
{
  if (stamps.size() < 2) {
    return 0;
  }
  std::vector<std::int64_t> deltas;
  deltas.reserve(stamps.size() - 1);
  for (std::size_t i = 1; i < stamps.size(); ++i) {
    deltas.push_back(stamps[i] - stamps[i - 1]);
  }
  std::sort(deltas.begin(), deltas.end());
  return deltas[deltas.size() / 2];
}

}  // namespace

int run_pcd_concat(const PcdConcatArgs & args)
{
  // ---- validate arguments -------------------------------------------------
  if (args.input_topics.size() < 2) {
    BAGWIZ_LOG_ERROR(kLogger, "pcd concat: --input-topics needs at least 2 topics");
    return 1;
  }
  // --frame defaults to base_link. When it is not given and the default cannot
  // reach every --input-topics frame via static TF, --frame becomes required
  // (enforced during extrinsic resolution below).
  const bool frame_explicit = args.frame.has_value();
  const std::string target_frame = frame_explicit ? *args.frame : std::string(kDefaultFrame);
  {
    std::unordered_set<std::string> seen;
    for (const auto & t : args.input_topics) {
      if (!seen.insert(t).second) {
        BAGWIZ_LOG_ERROR(kLogger, "pcd concat: duplicate topic in --input-topics: '%s'", t.c_str());
        return 1;
      }
    }
  }

  // The first --input-topics topic is always the reference that drives the output
  // rate and per-message timestamps.
  const std::size_t ref_idx = 0;

  const std::size_t num_topics = args.input_topics.size();
  std::unordered_map<std::string, std::size_t> topic_index;
  for (std::size_t i = 0; i < num_topics; ++i) {
    topic_index[args.input_topics[i]] = i;
  }

  // parse --stamp-offset entries
  std::vector<std::int64_t> offsets(num_topics, 0);
  for (const auto & entry : args.stamp_offsets) {
    const auto eq = entry.find('=');
    if (eq == std::string::npos) {
      BAGWIZ_LOG_ERROR(
        kLogger, "pcd concat: --stamp-offset must be topic=value (got '%s')", entry.c_str());
      return 1;
    }
    const std::string topic = entry.substr(0, eq);
    const std::string value = entry.substr(eq + 1);
    const auto it = topic_index.find(topic);
    if (it == topic_index.end()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "pcd concat: --stamp-offset topic '%s' is not in --input-topics", topic.c_str());
      return 1;
    }
    const auto ns = core::parse_duration_ns(value);
    if (!ns.has_value()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "pcd concat: could not parse --stamp-offset value '%s' (e.g. 50ms, -500ns, 0.05s)",
        value.c_str());
      return 1;
    }
    offsets[it->second] = *ns;
  }

  // parse --tolerance (number + optional unit ns/us/ms/s, no unit = ms) if given
  std::optional<std::int64_t> tolerance_override;
  if (args.tolerance.has_value()) {
    const auto ns = core::parse_duration_ns(*args.tolerance);
    if (!ns.has_value() || *ns < 0) {
      BAGWIZ_LOG_ERROR(
        kLogger, "pcd concat: could not parse --tolerance '%s' (e.g. 50ms, 0.05s, 500us)",
        args.tolerance->c_str());
      return 1;
    }
    tolerance_override = *ns;
  }

  // ---- open reader, validate topics ---------------------------------------
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(args.input_path);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", args.input_path.c_str(), e.what());
    return 1;
  }
  reader->populate_schemas();

  // Resolve output-topic existence and every input topic's TopicInfo in a single
  // pass over the bag's topics, using the topic_index built above.
  std::vector<const io::TopicInfo *> info_by_index(num_topics, nullptr);
  bool output_topic_exists = false;
  for (const auto & t : reader->topics()) {
    if (t.name == args.output_topic) {
      output_topic_exists = true;
    }
    if (const auto it = topic_index.find(t.name); it != topic_index.end()) {
      info_by_index[it->second] = &t;
    }
  }
  for (std::size_t i = 0; i < num_topics; ++i) {
    const io::TopicInfo * info = info_by_index[i];
    if (info == nullptr) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Topic '%s' is not present in %s", args.input_topics[i].c_str(),
        args.input_path.c_str());
      return 1;
    }
    if (info->type != kPointCloud2Type) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Topic '%s' is %s, expected %s", args.input_topics[i].c_str(), info->type.c_str(),
        kPointCloud2Type);
      return 1;
    }
  }
  const io::TopicInfo * ref_info = info_by_index[ref_idx];

  if (output_topic_exists && !args.force) {
    BAGWIZ_LOG_ERROR(
      kLogger, "Output topic '%s' already exists in %s; pass --force to replace it",
      args.output_topic.c_str(), args.input_path.c_str());
    return 1;
  }

  // Per-topic diagnostics surfaced in the end-of-run summary.
  std::vector<std::int64_t> header_fail(num_topics, 0);  // undecodable header in Pass A
  std::vector<char> non_monotonic(num_topics, 0);        // header stamps went backwards

  // ---- Pass A: collect per-topic header stamps + first frame_id -----------
  std::vector<TopicState> topics(num_topics);
  for (std::size_t i = 0; i < num_topics; ++i) {
    topics[i].name = args.input_topics[i];
    topics[i].offset_ns = offsets[i];
  }
  {
    std::unique_ptr<io::BagReader> sreader;
    try {
      sreader = io::open_read(args.input_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to reopen %s: %s", args.input_path.c_str(), e.what());
      return 1;
    }
    io::ReadFilter filter;
    filter.topics = args.input_topics;
    sreader->set_filter(filter);
    io::RawMessage raw;
    try {
      while (sreader->next(raw)) {
        const auto it = topic_index.find(raw.topic->name);
        if (it == topic_index.end()) {
          continue;
        }
        TopicState & ts = topics[it->second];
        const auto header = core::pointcloud::parse_pointcloud2_header(raw.payload);
        if (header.ok()) {
          if (!ts.stamps_ns.empty() && header.header->timestamp_ns < ts.stamps_ns.back()) {
            non_monotonic[it->second] = 1;
          }
          ts.stamps_ns.push_back(header.header->timestamp_ns);
          if (ts.frame_id.empty()) {
            ts.frame_id = header.header->frame_id;
          }
        } else {
          // keep index alignment with Pass B by recording the bag stamp
          ts.stamps_ns.push_back(raw.timestamp_ns);
          ++header_fail[it->second];
        }
      }
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "read error collecting stamps: %s", e.what());
      return 1;
    }
  }
  for (std::size_t i = 0; i < num_topics; ++i) {
    if (topics[i].stamps_ns.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Topic '%s' has no decodable PointCloud2 messages", topics[i].name.c_str());
      return 1;
    }
    if (topics[i].frame_id.empty()) {
      BAGWIZ_LOG_ERROR(kLogger, "Could not read a frame_id from '%s'", topics[i].name.c_str());
      return 1;
    }
  }

  // ---- resolve extrinsics (target = --frame) ------------------------------
  {
    tf2::BufferCore buffer{kTfBufferCacheTime};
    bool need_tf = false;
    for (const auto & ts : topics) {
      if (ts.frame_id != target_frame) {
        need_tf = true;
      }
    }
    if (need_tf) {
      std::string error;
      if (!build_static_tf_buffer(args.input_path, buffer, error)) {
        BAGWIZ_LOG_ERROR(kLogger, "%s", error.c_str());
        return 1;
      }
    }
    for (auto & ts : topics) {
      if (ts.frame_id == target_frame) {
        ts.extrinsic = core::pointcloud::RigidTransform{};  // identity
        continue;
      }
      // Resolve target_frame <- ts.frame_id from the static TF. If --frame was
      // not given and a topic cannot reach the default (base_link), the default
      // does not span every input, so --frame is required.
      std::string reach_error;
      const auto missing = core::missing_frames(buffer, target_frame, ts.frame_id);
      if (!missing.empty()) {
        std::string names;
        for (std::size_t i = 0; i < missing.size(); ++i) {
          names += (i ? ", " : "") + missing[i];
        }
        reach_error = "frame(s) not present in the bag's static TF tree: " + names;
      } else {
        try {
          ts.extrinsic =
            to_rigid(buffer.lookupTransform(target_frame, ts.frame_id, tf2::TimePointZero));
        } catch (const std::exception & e) {
          reach_error =
            "no static TF chain from '" + target_frame + "' to '" + ts.frame_id + "': " + e.what();
        }
      }
      if (!reach_error.empty()) {
        if (frame_explicit) {
          BAGWIZ_LOG_ERROR(kLogger, "%s", reach_error.c_str());
        } else {
          BAGWIZ_LOG_ERROR(
            kLogger,
            "pcd concat: --frame is required — the default frame '%s' is not reachable from '%s' "
            "(frame '%s'); pass --frame explicitly [%s]",
            target_frame.c_str(), ts.name.c_str(), ts.frame_id.c_str(), reach_error.c_str());
        }
        return 1;
      }
    }
  }

  // ---- tolerance + plan ---------------------------------------------------
  std::int64_t tolerance_ns = kDefaultToleranceNs;
  if (tolerance_override.has_value()) {
    tolerance_ns = *tolerance_override;
  } else {
    const std::int64_t period = median_period_ns(topics[ref_idx].stamps_ns);
    if (period > 0) {
      tolerance_ns = period / 2;
    }
  }

  std::vector<core::pointcloud::SyncTopic> sync_topics(num_topics);
  for (std::size_t i = 0; i < num_topics; ++i) {
    sync_topics[i].stamps_ns = topics[i].stamps_ns;
    sync_topics[i].offset_ns = topics[i].offset_ns;
  }
  const auto groups = core::pointcloud::plan_sync(sync_topics, ref_idx, tolerance_ns);

  // (topic i, msg index) -> groups referencing it; and per-group remaining picks.
  std::vector<std::vector<std::vector<std::size_t>>> refs(num_topics);
  for (std::size_t i = 0; i < num_topics; ++i) {
    refs[i].resize(topics[i].stamps_ns.size());
  }
  std::vector<std::size_t> group_remaining(groups.size(), 0);
  for (std::size_t g = 0; g < groups.size(); ++g) {
    for (std::size_t t = 0; t < num_topics; ++t) {
      if (groups[g].picks[t].has_value()) {
        refs[t][*groups[g].picks[t]].push_back(g);
        ++group_remaining[g];
      }
    }
  }

  // ---- output writer factory + streaming build ----------------------------
  // Topics suppressed from copy-through: dropped pcd inputs, and a pre-existing
  // output topic being replaced (--force).
  std::unordered_set<std::string> suppress;
  if (args.drop_inputs) {
    for (const auto & t : args.input_topics) {
      suppress.insert(t);
    }
  }
  if (output_topic_exists) {
    suppress.insert(args.output_topic);
  }

  io::TopicInfo out_topic = *ref_info;
  out_topic.name = args.output_topic;

  // Cache of transformed clouds keyed by (topic index, msg index), with a
  // refcount so an entry is freed once no pending group needs it.
  struct Cached
  {
    core::pointcloud::PointCloud2 cloud;
    std::int64_t header_stamp_ns = 0;
    std::size_t refcount = 0;
  };
  std::unordered_map<std::uint64_t, Cached> cache;
  const auto key = [](std::size_t t, std::size_t i) -> std::uint64_t {
    return (static_cast<std::uint64_t>(t) << 40) | static_cast<std::uint64_t>(i);
  };

  std::vector<char> fired(groups.size(), 0);
  std::int64_t written_groups = 0;
  std::int64_t partial_groups = 0;
  std::vector<std::int64_t> matched(num_topics, 0);
  std::vector<std::int64_t> parse_fail(num_topics, 0);
  std::vector<std::int64_t> transform_fail(num_topics, 0);

  const auto execute_pass = [&](io::BagWriter & writer) -> int {
    // declare surviving input topics + the new output topic
    for (const auto & t : reader->topics()) {
      if (suppress.count(t.name) != 0 || t.name == args.output_topic) {
        continue;
      }
      writer.declare_topic(t);
    }
    writer.declare_topic(out_topic);

    std::unique_ptr<io::BagReader> rd;
    try {
      rd = io::open_read(args.input_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to reopen %s: %s", args.input_path.c_str(), e.what());
      return 1;
    }
    rd->populate_schemas();

    std::vector<std::size_t> seen(num_topics, 0);
    io::RawMessage raw;
    while (rd->next(raw)) {
      const std::string & name = raw.topic->name;
      const auto ti = topic_index.find(name);

      // copy-through unless suppressed
      if (suppress.count(name) == 0 && name != args.output_topic) {
        writer.write(name, raw.timestamp_ns, raw.payload);
      }

      if (ti == topic_index.end()) {
        continue;  // not a pcd input topic
      }
      const std::size_t t = ti->second;
      const std::size_t idx = seen[t]++;
      if (idx >= refs[t].size() || refs[t][idx].empty()) {
        continue;  // this message is not picked by any group
      }

      // parse + transform + stash
      auto parsed = core::pointcloud::parse_pointcloud2(raw.payload);
      if (parsed.ok()) {
        auto cloud = std::move(*parsed.cloud);
        const auto tr = core::pointcloud::transform_cloud_xyz(cloud, topics[t].extrinsic);
        if (tr.ok) {
          Cached c;
          c.header_stamp_ns = cloud.timestamp_ns;
          c.refcount = refs[t][idx].size();
          c.cloud = std::move(cloud);
          cache.emplace(key(t, idx), std::move(c));
        } else {
          ++transform_fail[t];
        }
      } else {
        ++parse_fail[t];
      }

      // notify referencing groups; fire the ones now complete
      for (const std::size_t g : refs[t][idx]) {
        if (group_remaining[g] > 0) {
          --group_remaining[g];
        }
        if (group_remaining[g] != 0 || fired[g] != 0) {
          continue;
        }
        fired[g] = 1;
        // assemble inputs in --input-topics order from cached picks
        std::vector<core::pointcloud::ConcatInput> inputs;
        for (std::size_t k = 0; k < num_topics; ++k) {
          if (!groups[g].picks[k].has_value()) {
            continue;
          }
          const auto ci = cache.find(key(k, *groups[g].picks[k]));
          if (ci == cache.end()) {
            continue;  // parse/transform failed -> partial
          }
          inputs.push_back({&ci->second.cloud, ci->second.header_stamp_ns});
          ++matched[k];
        }
        if (inputs.size() < num_topics) {
          ++partial_groups;
        }
        if (!inputs.empty()) {
          const auto merged =
            core::pointcloud::concat_clouds(inputs, groups[g].output_stamp_ns, target_frame);
          if (!merged.ok()) {
            BAGWIZ_LOG_ERROR(kLogger, "concat failed: %s", merged.error.c_str());
            return 1;
          }
          const auto payload = core::pointcloud::serialize_pointcloud2(*merged.cloud);
          writer.write(args.output_topic, groups[g].output_stamp_ns, payload);
          ++written_groups;
        }
        // release cached picks whose refcount hits zero
        for (std::size_t k = 0; k < num_topics; ++k) {
          if (!groups[g].picks[k].has_value()) {
            continue;
          }
          const auto ci = cache.find(key(k, *groups[g].picks[k]));
          if (ci != cache.end() && --ci->second.refcount == 0) {
            cache.erase(ci);
          }
        }
      }
    }

    try {
      writer.close();
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Writer close() failed: %s", e.what());
      return 1;
    }
    return 0;
  };

  // ---- dispatch: -o (new bag) vs in-place ---------------------------------
  int status = 0;
  if (args.output_path.has_value()) {
    if (const auto r = core::prepare_output_path(*args.output_path, args.overwrite); !r.ok) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", r.error.c_str());
      return 1;
    }
    const auto input = args.input_path;
    const auto output = *args.output_path;
    std::unique_ptr<io::BagWriter> writer;
    try {
      auto copts = io::create_options_inheriting_format(input, output);
      copts.mcap_compression = "none";
      writer = io::open_write(output, copts);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open output writer: %s", e.what());
      return 1;
    }
    status = execute_pass(*writer);
  } else {
    const auto inplace_copts = io::create_options_preserving_storage(args.input_path);
    if (inplace_copts.format == io::Format::Auto) {
      BAGWIZ_LOG_ERROR(
        kLogger, "pcd concat: could not detect storage format of input bag '%s'.",
        args.input_path.c_str());
      return 1;
    }
    try {
      core::write_bag_inplace(args.input_path, [&](const std::filesystem::path & tmp) {
        auto copts = inplace_copts;
        copts.mcap_compression = "none";
        auto writer = io::open_write(tmp, copts);
        status = execute_pass(*writer);
        if (status != 0) {
          throw std::runtime_error("pcd concat: pass failed; aborting in-place swap");
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
    kLogger,
    "pcd concat: wrote %" PRId64 " concatenated message(s) to '%s' (%" PRId64
    " partial, tolerance %.3f ms)",
    written_groups, args.output_topic.c_str(), partial_groups,
    static_cast<double>(tolerance_ns) / 1e6);
  for (std::size_t i = 0; i < num_topics; ++i) {
    BAGWIZ_LOG_INFO(
      kLogger, "  %s: matched %" PRId64 " (frame '%s', offset %.3f ms)", topics[i].name.c_str(),
      matched[i], topics[i].frame_id.c_str(), static_cast<double>(topics[i].offset_ns) / 1e6);
  }
  for (std::size_t i = 0; i < num_topics; ++i) {
    if (
      header_fail[i] != 0 || parse_fail[i] != 0 || transform_fail[i] != 0 ||
      non_monotonic[i] != 0) {
      BAGWIZ_LOG_WARN(
        kLogger,
        "  %s: %" PRId64 " undecodable header(s) [bag time used for matching], %" PRId64
        " parse + %" PRId64 " transform failure(s) dropped from concat%s",
        topics[i].name.c_str(), header_fail[i], parse_fail[i], transform_fail[i],
        non_monotonic[i] != 0 ? "; header stamps are not monotonic (matching may be wrong)" : "");
    }
  }
  return 0;
}

}  // namespace bagwiz::commands
