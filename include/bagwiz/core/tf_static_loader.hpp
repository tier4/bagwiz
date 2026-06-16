// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__TF_STATIC_LOADER_HPP_
#define BAGWIZ__CORE__TF_STATIC_LOADER_HPP_

#include <tf2/buffer_core.hpp>

#include <filesystem>
#include <memory>
#include <string>

namespace bagwiz::core
{

struct TfStaticLoadResult {
  bool ok = false;
  std::string error;
  std::unique_ptr<tf2::BufferCore> buffer;
};

TfStaticLoadResult load_static_tf(const std::filesystem::path & input);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__TF_STATIC_LOADER_HPP_
