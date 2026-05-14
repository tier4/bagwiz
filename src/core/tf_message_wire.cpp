// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tf_message_wire.hpp"

#include "bagwiz/io/bag_io.hpp"

#include <string_view>

namespace bagwiz::core
{

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

}  // namespace bagwiz::core
