#pragma once

#include "bagwiz/io/bag_io.hpp"

#include <tf2/buffer_core.h>

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
