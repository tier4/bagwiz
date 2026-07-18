// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__IO__BAG_OPEN_HPP_
#define BAGWIZ__IO__BAG_OPEN_HPP_

#include "bagwiz/io/bag_io.hpp"

#include <filesystem>
#include <functional>
#include <memory>

// try/catch + log wrappers around the bag open/close calls, covering the
// failure handling every rewrite-style command repeats: open the input
// reader, open the output writer through the injected factory, and close the
// writer at the end of the pass. Each helper logs one fixed message (the
// wording the commands have always emitted — tests and users may match it)
// and reports failure through its return value, so the caller's error path
// stays `if (!x) return 1;`.
namespace bagwiz::io
{

// Factory producing a fresh, open BagWriter. Rewrite-style commands inject
// this shape so the dispatch layer (e.g. an in-place rewrite) can decide the
// output path at call time.
using WriterFactory = std::function<std::unique_ptr<BagWriter>()>;

// Open `path` for reading via open_read(). On failure log exactly
// "Failed to open %s: %s" (path, error) to `logger` and return nullptr.
std::unique_ptr<BagReader> open_read_or_log(
  const std::filesystem::path & path, const char * logger);

// Invoke `factory` to open the output writer. On failure log exactly
// "Failed to open output writer: %s" (error) to `logger` and return nullptr.
std::unique_ptr<BagWriter> open_write_or_log(const WriterFactory & factory, const char * logger);

// Close `writer`. On failure log exactly "Writer close() failed: %s" (error)
// to `logger` and return false; returns true on success.
bool close_writer_or_log(BagWriter & writer, const char * logger);

}  // namespace bagwiz::io

#endif  // BAGWIZ__IO__BAG_OPEN_HPP_
