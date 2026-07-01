// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__TF_BUFFER_LOADER_HPP_
#define BAGWIZ__CORE__TF_BUFFER_LOADER_HPP_

#include <tf2/buffer_core.hpp>

#include <filesystem>
#include <optional>
#include <string>

namespace bagwiz::core
{

// Load every tf2_msgs/msg/TFMessage topic from `input` into `buffer`.
// Returns std::nullopt on success, or an error string on failure.
[[nodiscard]] std::optional<std::string> load_tf_buffer(
  const std::filesystem::path & input, tf2::BufferCore & buffer);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__TF_BUFFER_LOADER_HPP_
