// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__TF__TF_TOPICS_HPP_
#define BAGWIZ__CORE__TF__TF_TOPICS_HPP_

#include "bagwiz/io/bag_io.hpp"

#include <string>
#include <string_view>
#include <vector>

namespace bagwiz::core
{

// True when a TF topic's name marks it static (ends with "tf_static", e.g.
// "/tf_static"). Static topics use transient_local durability and carry
// one-shot, time-independent transforms; everything else carrying TFMessage is
// treated as dynamic.
bool is_static_tf_topic(std::string_view topic_name);

// A tf2_msgs/msg/TFMessage topic in the bag plus the static flag used to
// populate a tf2 buffer with the correct static/dynamic storage.
struct TfTopic
{
  std::string name;
  bool is_static = false;
};

// Every tf2_msgs/msg/TFMessage topic the bag carries, in the reader's topic
// order, each tagged with its static flag (see is_static_tf_topic).
std::vector<TfTopic> collect_tf_topics(const io::BagReader & reader);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__TF__TF_TOPICS_HPP_
