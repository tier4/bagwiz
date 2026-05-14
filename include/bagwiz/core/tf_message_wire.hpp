// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__TF_MESSAGE_WIRE_HPP_
#define BAGWIZ__CORE__TF_MESSAGE_WIRE_HPP_

#include "bagwiz/io/bag_io.hpp"

#include <string_view>

// Wire-level helpers for tf2_msgs/msg/TFMessage. Sits next to
// tf_static_injector.hpp (which provides `serialize_tf_message`), but
// the responsibilities are deliberately split: this header carries the
// constants and TopicInfo synthesis used when declaring a *new* TF
// topic in a destination bag (e.g. by `bagwiz traj join`), while
// tf_static_injector keeps the bag-scanning helpers specific to
// `bagwiz tf inject-static`.
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

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__TF_MESSAGE_WIRE_HPP_
