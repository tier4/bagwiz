// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__TF__TF_MESSAGE_WIRE_HPP_
#define BAGWIZ__CORE__TF__TF_MESSAGE_WIRE_HPP_

#include "bagwiz/io/bag_io.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cstddef>
#include <memory>
#include <span>
#include <string_view>
#include <vector>

// Wire-level helpers for tf2_msgs/msg/TFMessage: the canonical schema
// text, TopicInfo synthesis for declaring a *new* TF topic in a
// destination bag, and CDR serialisation of a TFMessage payload. Used
// when writing TF topics into a bag (e.g. by `bagwiz traj join`).
namespace bagwiz::core
{

// Concatenated ros2msg schema text for `tf2_msgs/msg/TFMessage`,
// including every transitively-referenced geometry_msgs / std_msgs /
// builtin_interfaces type. The encoding matches what rosbag2's MCAP
// backend records in `Schema.data` for this type, so a writer
// declaring a new TF topic can hand this string straight to
// `io::TopicInfo::schema_text` and downstream readers / decoders pick
// it up without needing the rosidl introspection typesupport.
extern const char * const kTfMessageWireSchema;

// Build an `io::TopicInfo` suitable for `BagWriter::declare_topic`
// when introducing a brand-new `tf2_msgs/msg/TFMessage` topic into a
// destination bag. Returned fields:
//
//   name                  = topic_name
//   type                  = "tf2_msgs/msg/TFMessage"
//   serialization_format  = "cdr"
//   schema_encoding       = "ros2msg"
//   schema_text           = kTfMessageWireSchema
//   offered_qos_profiles  = ""
//   type_description_hash = ""
//
// Callers that are re-declaring a topic that already exists in the
// destination bag should pass through that existing TopicInfo instead
// of synthesising a new one, so QoS / type-description-hash metadata
// is preserved on round-trip.
io::TopicInfo make_tf_message_topic_info(std::string_view topic_name);

// Encode a single tf2_msgs/msg/TFMessage carrying `transforms` into
// the on-wire CDR payload. Uses the introspection typesupport (dlopen'd
// at runtime from libtf2_msgs__rosidl_typesupport_introspection_cpp.so)
// + rmw_serialize. Throws std::runtime_error when the typesupport
// cannot be loaded or rmw_serialize fails.
std::vector<std::byte> serialize_tf_message(
  std::span<const geometry_msgs::msg::TransformStamped> transforms);

// Stateful serializer for repeated tf2_msgs/msg/TFMessage payloads.
//
// This class serializes each TFMessage payload directly to CDR via
// cdr_walker::CdrWriter, so callers that serialize many TF messages (e.g.
// `bagwiz traj join`) avoid the introspection typesupport / rmw_serialize
// path and its per-message dlopen/dlsym overhead. It is NOT thread-safe;
// use one instance per thread.
class TfMessageSerializer
{
public:
  TfMessageSerializer();
  ~TfMessageSerializer();

  TfMessageSerializer(const TfMessageSerializer &) = delete;
  TfMessageSerializer & operator=(const TfMessageSerializer &) = delete;
  TfMessageSerializer(TfMessageSerializer &&) noexcept;
  TfMessageSerializer & operator=(TfMessageSerializer &&) noexcept;

  // Serialize `transform` as a TFMessage containing exactly one
  // TransformStamped. Writes the CDR bytes into `out`, resizing it to the
  // exact payload length. The internal state is reused across calls.
  void serialize_one(
    const geometry_msgs::msg::TransformStamped & transform, std::vector<std::byte> & out);

  // Serialize `transforms` as a TFMessage carrying N TransformStamped entries.
  // Writes the CDR bytes into `out`, resizing it to the exact payload length.
  void serialize_many(
    std::span<const geometry_msgs::msg::TransformStamped> transforms, std::vector<std::byte> & out);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__TF__TF_MESSAGE_WIRE_HPP_
