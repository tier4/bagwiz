// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tf_message_wire.hpp"

#include "bagwiz/core/introspection_loader.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <tf2_msgs/msg/tf_message.hpp>

#include <rcutils/allocator.h>
#include <rmw/rmw.h>
#include <rmw/serialized_message.h>

#include <cstddef>
#include <cstring>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace bagwiz::core
{

namespace
{

constexpr const char * kTfMessageType = "tf2_msgs/msg/TFMessage";

// RAII wrapper around rmw_serialized_message_t. Mirrors the
// SerializedMessageRmw helper in ros2_yaml_to_cdr.cpp; kept local here
// to avoid coupling the two modules through a private header.
class SerializedMessageRmw
{
public:
  explicit SerializedMessageRmw(std::size_t capacity)
  {
    rcutils_allocator_t alloc = rcutils_get_default_allocator();
    if (rmw_serialized_message_init(&msg_, capacity, &alloc) != RMW_RET_OK) {
      throw std::runtime_error("rmw_serialized_message_init failed");
    }
  }
  ~SerializedMessageRmw() { rmw_serialized_message_fini(&msg_); }

  SerializedMessageRmw(const SerializedMessageRmw &) = delete;
  SerializedMessageRmw & operator=(const SerializedMessageRmw &) = delete;
  SerializedMessageRmw(SerializedMessageRmw &&) = delete;
  SerializedMessageRmw & operator=(SerializedMessageRmw &&) = delete;

  rmw_serialized_message_t & get() noexcept { return msg_; }

private:
  rmw_serialized_message_t msg_ = rmw_get_zero_initialized_serialized_message();
};

}  // namespace

// Same field/separator layout that rosbag2's writer records into
// MCAP Schema records for tf2_msgs/msg/TFMessage. Keeping the bytes
// identical means decoders that compare schema_text textually still
// recognise topics written via this helper.
const char * const kTfMessageWireSchema =
  "geometry_msgs/TransformStamped[] transforms\n"
  "================================================================================\n"
  "MSG: geometry_msgs/TransformStamped\n"
  "std_msgs/Header header\n"
  "string child_frame_id\n"
  "Transform transform\n"
  "================================================================================\n"
  "MSG: std_msgs/Header\n"
  "builtin_interfaces/Time stamp\n"
  "string frame_id\n"
  "================================================================================\n"
  "MSG: geometry_msgs/Transform\n"
  "Vector3 translation\n"
  "Quaternion rotation\n"
  "================================================================================\n"
  "MSG: geometry_msgs/Vector3\n"
  "float64 x\n"
  "float64 y\n"
  "float64 z\n"
  "================================================================================\n"
  "MSG: geometry_msgs/Quaternion\n"
  "float64 x\n"
  "float64 y\n"
  "float64 z\n"
  "float64 w\n";

io::TopicInfo make_tf_message_topic_info(std::string_view topic_name)
{
  io::TopicInfo info;
  info.name.assign(topic_name.begin(), topic_name.end());
  info.type = "tf2_msgs/msg/TFMessage";
  info.serialization_format = "cdr";
  info.schema_encoding = "ros2msg";
  info.schema_text = kTfMessageWireSchema;
  return info;
}

std::vector<std::byte> serialize_tf_message(
  std::span<const geometry_msgs::msg::TransformStamped> transforms)
{
  auto intro = load_introspection(kTfMessageType);
  if (!intro.ok()) {
    throw std::runtime_error(
      std::string("could not load introspection typesupport for ") + kTfMessageType + ": " +
      intro.error);
  }

  // The introspection typesupport and rosidl_generator_cpp emit
  // structurally identical layouts for the same IDL, so we can build a
  // standard C++ TFMessage and hand its address to rmw_serialize.
  tf2_msgs::msg::TFMessage msg;
  msg.transforms.assign(transforms.begin(), transforms.end());

  SerializedMessageRmw serialized(0);
  const rmw_ret_t rc = rmw_serialize(&msg, intro.typesupport, &serialized.get());
  if (rc != RMW_RET_OK) {
    const rcutils_error_state_t * s = rcutils_get_error_state();
    std::string err = "rmw_serialize failed: ";
    err += (s != nullptr) ? s->message : "(no error message)";
    rcutils_reset_error();
    throw std::runtime_error(err);
  }

  const auto * sm = &serialized.get();
  std::vector<std::byte> out;
  out.resize(sm->buffer_length);
  if (sm->buffer_length > 0 && sm->buffer != nullptr) {
    std::memcpy(out.data(), sm->buffer, sm->buffer_length);
  }
  return out;
}

}  // namespace bagwiz::core
