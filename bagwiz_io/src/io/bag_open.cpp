// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/bag_open.hpp"

#include "bagwiz/core/base/logging.hpp"

#include <exception>
#include <memory>

namespace bagwiz::io
{

std::unique_ptr<BagReader> open_read_or_log(const std::filesystem::path & path, const char * logger)
{
  try {
    return open_read(path);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(logger, "Failed to open %s: %s", path.c_str(), e.what());
    return nullptr;
  }
}

std::unique_ptr<BagWriter> open_write_or_log(const WriterFactory & factory, const char * logger)
{
  try {
    return factory();
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(logger, "Failed to open output writer: %s", e.what());
    return nullptr;
  }
}

bool close_writer_or_log(BagWriter & writer, const char * logger)
{
  try {
    writer.close();
    return true;
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(logger, "Writer close() failed: %s", e.what());
    return false;
  }
}

}  // namespace bagwiz::io
