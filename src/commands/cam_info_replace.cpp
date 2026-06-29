// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/cam_info_replace.hpp"

#include "bagwiz/core/bag_inplace.hpp"
#include "bagwiz/core/image/camera_calibration_yaml.hpp"
#include "bagwiz/core/introspection_loader.hpp"
#include "bagwiz/core/logging.hpp"
#include "bagwiz/core/output_path.hpp"
#include "bagwiz/core/pipeline/backend_select.hpp"
#include "bagwiz/core/pipeline/rewrite_backend.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <sensor_msgs/msg/camera_info.hpp>

#include <rcutils/allocator.h>
#include <rcutils/error_handling.h>
#include <rmw/rmw.h>
#include <rmw/serialized_message.h>
#include <rmw/types.h>

#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{

namespace img = bagwiz::core::image;

constexpr const char * kLogger = "bagwiz.cmd.cam-info.replace";
constexpr const char * kCameraInfoType = "sensor_msgs/msg/CameraInfo";

// RAII wrapper around an rmw_serialized_message_t that owns its buffer (the
// destination of rmw_serialize). Mirrors the helper in ros2_yaml_to_cdr.cpp.
class OwnedSerializedMessage
{
public:
  OwnedSerializedMessage()
  {
    rcutils_allocator_t alloc = rcutils_get_default_allocator();
    if (rmw_serialized_message_init(&msg_, 0, &alloc) != RMW_RET_OK) {
      throw std::runtime_error("rmw_serialized_message_init failed");
    }
  }
  ~OwnedSerializedMessage() { rmw_serialized_message_fini(&msg_); }

  OwnedSerializedMessage(const OwnedSerializedMessage &) = delete;
  OwnedSerializedMessage & operator=(const OwnedSerializedMessage &) = delete;
  OwnedSerializedMessage(OwnedSerializedMessage &&) = delete;
  OwnedSerializedMessage & operator=(OwnedSerializedMessage &&) = delete;

  rmw_serialized_message_t & get() noexcept { return msg_; }

private:
  rmw_serialized_message_t msg_ = rmw_get_zero_initialized_serialized_message();
};

// Most recent rmw error as a string, then reset so it does not leak into a
// later call. Returns a placeholder when no error state is set.
std::string take_rmw_error()
{
  const rcutils_error_state_t * s = rcutils_get_error_state();
  std::string err = (s != nullptr) ? s->message : "(no error message)";
  rcutils_reset_error();
  return err;
}

// Rewrites the calibration on one or more CameraInfo topics. Non-target topics
// pass through verbatim. For a target topic, the original CDR is deserialized
// into a typed sensor_msgs/msg/CameraInfo (so the header, binning, and roi
// survive untouched), the calibration fields are overwritten from the (single,
// shared) YAML, and the message is re-serialized. The typesupport pointer is
// shared and read-only; transform() runs on the single producer thread, so the
// scratch typed message it builds is never shared across threads.
class CamInfoReplaceProcessor : public core::pipeline::Processor
{
public:
  CamInfoReplaceProcessor(
    const std::vector<std::string> & topics, const rosidl_message_type_support_t * typesupport,
    const img::CameraCalibration & calibration, const std::optional<std::string> & frame_id)
  : targets_(topics.begin(), topics.end()),
    typesupport_(typesupport),
    calibration_(calibration),
    frame_id_(frame_id)
  {
    for (const auto & topic : topics) {
      rewritten_.emplace(topic, 0);
    }
  }

  // Number of messages rewritten on `topic` over the pass (0 for a topic that
  // carried none, or one not targeted). Read after the pipeline run completes.
  [[nodiscard]] std::uint64_t rewritten_count(const std::string & topic) const
  {
    const auto it = rewritten_.find(topic);
    return it == rewritten_.end() ? 0 : it->second;
  }

  [[nodiscard]] core::pipeline::Emit route(const std::string & in_topic) const override
  {
    return core::pipeline::Emit{true, in_topic};
  }

  [[nodiscard]] bool transforms() const override { return true; }

  [[nodiscard]] core::pipeline::TransformAction transform(
    const std::string & in_topic, std::span<const std::byte> in,
    std::vector<std::byte> & out) const override
  {
    if (targets_.find(in_topic) == targets_.end()) {
      return core::pipeline::TransformAction::kPassthrough;
    }

    // Wrap the reader's payload as an rmw_serialized_message_t without taking
    // ownership of it (no _fini on this view; the bytes belong to the reader).
    rmw_serialized_message_t in_view = rmw_get_zero_initialized_serialized_message();
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast) rmw API is non-const but only reads.
    in_view.buffer = const_cast<std::uint8_t *>(reinterpret_cast<const std::uint8_t *>(in.data()));
    in_view.buffer_length = in.size();
    in_view.buffer_capacity = in.size();
    in_view.allocator = rcutils_get_default_allocator();

    sensor_msgs::msg::CameraInfo msg;
    if (rmw_deserialize(&in_view, typesupport_, &msg) != RMW_RET_OK) {
      throw std::runtime_error(
        "failed to deserialize a message on '" + in_topic + "' as " + kCameraInfoType + ": " +
        take_rmw_error());
    }

    // Overwrite only the calibration fields; header (stamp + frame_id),
    // binning_x/y, and roi are left as deserialized.
    msg.height = calibration_.height;
    msg.width = calibration_.width;
    msg.distortion_model = calibration_.distortion_model;
    msg.d.assign(calibration_.d.begin(), calibration_.d.end());
    for (std::size_t i = 0; i < 9; ++i) {
      msg.k[i] = calibration_.k[i];
    }
    for (std::size_t i = 0; i < 9; ++i) {
      msg.r[i] = calibration_.r[i];
    }
    for (std::size_t i = 0; i < 12; ++i) {
      msg.p[i] = calibration_.p[i];
    }
    if (frame_id_.has_value()) {
      msg.header.frame_id = *frame_id_;
    }

    OwnedSerializedMessage serialized;
    if (rmw_serialize(&msg, typesupport_, &serialized.get()) != RMW_RET_OK) {
      throw std::runtime_error(
        "failed to re-serialize a CameraInfo message on '" + in_topic + "': " + take_rmw_error());
    }
    const auto * sm = &serialized.get();
    out.resize(sm->buffer_length);
    if (sm->buffer_length > 0 && sm->buffer != nullptr) {
      std::memcpy(out.data(), sm->buffer, sm->buffer_length);
    }
    ++rewritten_.at(in_topic);
    return core::pipeline::TransformAction::kWrite;
  }

private:
  std::unordered_set<std::string> targets_;
  const rosidl_message_type_support_t * typesupport_;
  const img::CameraCalibration & calibration_;
  const std::optional<std::string> & frame_id_;
  // Per-target tally of rewritten messages. Mutable because transform() is const
  // per the Processor contract, which also guarantees transform() runs on the
  // single producer thread — so this is the sole writer and needs no
  // synchronization. Keys are fixed at construction (one per target topic);
  // transform() only updates existing values via at(), never inserts.
  mutable std::unordered_map<std::string, std::uint64_t> rewritten_;
};

// One full pass: open `input_path`, declare every topic verbatim (the topic
// type is unchanged), then stream-copy — rewriting the calibration on each topic
// in `target_topics` and forwarding everything else untouched. The writer factory
// is parameterised so the in-place path can hand in a tmp location.
int execute_pass(
  const std::filesystem::path & input_path, const std::vector<std::string> & target_topics,
  const rosidl_message_type_support_t * typesupport, const img::CameraCalibration & calibration,
  const std::optional<std::string> & frame_id,
  const std::function<std::unique_ptr<io::BagWriter>()> & open_writer)
{
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input_path);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", input_path.c_str(), e.what());
    return 1;
  }
  reader->populate_schemas();

  const std::vector<io::TopicInfo> topics(reader->topics().begin(), reader->topics().end());

  std::unique_ptr<io::BagWriter> writer;
  try {
    writer = open_writer();
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Failed to open output writer: %s", e.what());
    return 1;
  }

  for (const auto & t : topics) {
    try {
      writer->declare_topic(t);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "declare_topic failed for '%s': %s", t.name.c_str(), e.what());
      return 1;
    }
  }

  CamInfoReplaceProcessor processor(target_topics, typesupport, calibration, frame_id);
  core::pipeline::RewriteCounts counts;
  try {
    auto backend = core::pipeline::make_backend(core::pipeline::BackendKind::Pipelined);
    counts =
      core::pipeline::run_pipeline(*reader, *writer, processor, *backend, "cam-info replace");
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "cam-info replace read/write failed: %s", e.what());
    return 1;
  }

  try {
    writer->close();
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "writer->close failed: %s", e.what());
    return 1;
  }

  const std::uint64_t forwarded = counts.copied - counts.transformed;
  std::string topic_list;
  for (const auto & t : target_topics) {
    if (!topic_list.empty()) {
      topic_list += ", ";
    }
    topic_list += t;
  }
  BAGWIZ_LOG_INFO(
    kLogger,
    "cam-info replace: rewrote calibration on %" PRIu64
    " message(s) across %zu topic(s) [%s], "
    "copied %" PRIu64 " other message(s) verbatim.",
    counts.transformed, target_topics.size(), topic_list.c_str(), forwarded);
  // Surface any target that carried no messages so a typo'd-but-existing or
  // empty topic does not silently no-op when the others were rewritten.
  for (const auto & t : target_topics) {
    if (processor.rewritten_count(t) == 0) {
      BAGWIZ_LOG_WARN(
        kLogger, "Topic '%s' carried no messages; its calibration was left unchanged.", t.c_str());
    }
  }
  return 0;
}

}  // namespace

int run_cam_info_replace(const CamInfoReplaceArgs & args)
{
  // 1. Inspect the bag and confirm every requested <topic> exists and is a
  //    CameraInfo topic. The CLI guarantees args.topics is non-empty (->required).
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(args.input_path);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", args.input_path.c_str(), e.what());
    return 1;
  }

  // Index the bag's topics by name and collect its CameraInfo topics (for the
  // error listing). The TopicInfo pointers stay valid until `reader` is reset
  // below, which is after this validation block completes.
  std::unordered_map<std::string, const io::TopicInfo *> topics_by_name;
  std::string camera_info_topics;
  for (const auto & t : reader->topics()) {
    topics_by_name.emplace(t.name, &t);
    if (t.type == kCameraInfoType) {
      if (!camera_info_topics.empty()) {
        camera_info_topics += ", ";
      }
      camera_info_topics += t.name;
    }
  }

  // Validate each requested topic, deduplicating while preserving command-line
  // order. Collect all failures before bailing so one run reports every bad
  // topic, and stop before the bag is touched if any failed.
  std::vector<std::string> target_topics;
  std::unordered_set<std::string> seen;
  bool all_valid = true;
  for (const auto & topic : args.topics) {
    if (!seen.insert(topic).second) {
      continue;  // duplicate on the command line; validated on its first occurrence
    }
    const auto it = topics_by_name.find(topic);
    if (it == topics_by_name.end()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Topic '%s' is not present in %s.", topic.c_str(), args.input_path.c_str());
      all_valid = false;
      continue;
    }
    if (it->second->type != kCameraInfoType) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Topic '%s' has type '%s', expected '%s'.", topic.c_str(),
        it->second->type.c_str(), kCameraInfoType);
      all_valid = false;
      continue;
    }
    target_topics.push_back(topic);
  }
  if (!all_valid) {
    BAGWIZ_LOG_ERROR(
      kLogger, "Available %s topic(s): %s", kCameraInfoType,
      camera_info_topics.empty() ? "(none)" : camera_info_topics.c_str());
    return 1;
  }

  // Release the inspection reader before opening the read/write passes.
  reader.reset();

  // 2. Parse the camera_calibration YAML.
  const auto parsed = img::parse_camera_calibration_yaml(args.yaml_path);
  if (!parsed.ok()) {
    BAGWIZ_LOG_ERROR(
      kLogger, "Could not load calibration from '%s': %s", args.yaml_path.c_str(),
      parsed.error.c_str());
    return 1;
  }
  const img::CameraCalibration & calibration = *parsed.calibration;

  // 3. Load the introspection typesupport used for both deserialize and
  //    serialize of the CameraInfo payloads.
  const auto intro = core::load_introspection(kCameraInfoType);
  if (!intro.ok()) {
    BAGWIZ_LOG_ERROR(
      kLogger, "Could not load introspection typesupport for %s: %s", kCameraInfoType,
      intro.error.c_str());
    return 1;
  }

  // 4a. Explicit -o: write a fresh bag, leaving <input> untouched.
  if (args.output_path.has_value()) {
    if (const auto r = core::prepare_output_path(*args.output_path, args.overwrite); !r.ok) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", r.error.c_str());
      return 1;
    }
    auto make_writer = [&]() {
      io::CreateOptions copts;
      copts.format = io::Format::Auto;
      copts.layout = io::Layout::Auto;
      copts.mcap_compression = "none";
      return io::open_write(*args.output_path, copts);
    };
    return execute_pass(
      args.input_path, target_topics, intro.typesupport, calibration, args.frame_id, make_writer);
  }

  // 4b. In-place: pin format/layout to <input>'s identity (the tmp suffix that
  //     write_bag_inplace uses cannot be interpreted by Auto resolution).
  const auto inplace_copts = io::create_options_preserving_storage(args.input_path);
  if (inplace_copts.format == io::Format::Auto) {
    BAGWIZ_LOG_ERROR(
      kLogger, "Could not detect storage format of input bag '%s'.",
      args.input_path.string().c_str());
    return 1;
  }
  auto make_inplace_writer = [inplace_copts](const std::filesystem::path & tmp) {
    auto copts = inplace_copts;
    copts.mcap_compression = "none";
    return io::open_write(tmp, copts);
  };

  int pass_status = 0;
  try {
    core::write_bag_inplace(args.input_path, [&](const std::filesystem::path & tmp) {
      pass_status = execute_pass(
        args.input_path, target_topics, intro.typesupport, calibration, args.frame_id,
        [&]() { return make_inplace_writer(tmp); });
      if (pass_status != 0) {
        throw std::runtime_error("cam-info replace: pass failed; aborting in-place swap");
      }
    });
  } catch (const std::exception & e) {
    // cppcheck-suppress knownConditionTrueFalse  // assigned inside the lambda above
    if (pass_status != 0) {
      return pass_status;
    }
    BAGWIZ_LOG_ERROR(kLogger, "In-place swap failed: %s", e.what());
    return 1;
  }
  return 0;
}

}  // namespace bagwiz::commands
