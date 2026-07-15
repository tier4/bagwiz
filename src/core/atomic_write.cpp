// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/atomic_write.hpp"

#include <exception>
#include <filesystem>
#include <fstream>
#include <ios>
#include <string>
#include <system_error>

namespace bagwiz::core
{

bool write_file_atomically(
  const std::filesystem::path & path, const std::string & contents, std::string & error)
{
  const std::filesystem::path tmp = path.parent_path() / (path.filename().string() + ".bagwiz.tmp");
  try {
    {
      std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
      if (!out) {
        error = "could not open '" + tmp.string() + "' for writing";
        return false;
      }
      out << contents;
      out.flush();
      if (!out) {
        error = "failed while writing '" + tmp.string() + "'";
        return false;
      }
    }
    std::filesystem::rename(tmp, path);
  } catch (const std::exception & e) {
    std::error_code ignored;
    std::filesystem::remove(tmp, ignored);
    error = e.what();
    return false;
  }
  return true;
}

}  // namespace bagwiz::core
