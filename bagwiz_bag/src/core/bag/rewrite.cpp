// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/bag/rewrite.hpp"

#include "bagwiz/core/bag/bag_inplace.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/base/output_path.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <exception>
#include <stdexcept>
#include <string>
#include <utility>

namespace bagwiz::core
{

int run_bag_rewrite(
  const std::filesystem::path & input_path,
  const std::optional<std::filesystem::path> & output_path, bool overwrite,
  const BagRewriteOptions & options, const BagRewritePass & pass)
{
  // -o mode: write a new bag whose storage follows the output path (its
  // extension picks a single-file backend; a directory inherits the input's
  // backend when inherit_output_format is set) and leave <input> untouched.
  if (output_path.has_value()) {
    if (const auto r = core::prepare_output_path(*output_path, overwrite); !r.ok) {
      BAGWIZ_LOG_ERROR(options.logger, "%s", r.error.c_str());
      return 1;
    }
    const auto input = input_path;
    const auto output = *output_path;
    const bool inherit = options.inherit_output_format;
    const bool no_compression = options.disable_mcap_compression;
    const io::WriterFactory make_writer = [input, output, inherit, no_compression]() {
      io::CreateOptions copts;
      if (inherit) {
        copts = io::create_options_inheriting_format(input, output);
      } else {
        copts.format = io::Format::Auto;
        copts.layout = io::Layout::Auto;
      }
      if (no_compression) {
        copts.mcap_compression = "none";
      }
      return io::open_write(output, copts);
    };
    return pass(make_writer);
  }

  // In-place mode: rewrite <input> atomically via a sibling tmp, preserving
  // its storage format and layout. The tmp path carries a synthetic suffix
  // that Format::Auto cannot interpret, so pin both explicitly.
  const auto inplace_copts = io::create_options_preserving_storage(input_path);
  if (inplace_copts.format == io::Format::Auto) {
    BAGWIZ_LOG_ERROR(options.logger, options.format_unknown_error, input_path.string().c_str());
    return 1;
  }
  const bool no_compression = options.disable_mcap_compression;
  auto make_inplace_writer = [inplace_copts, no_compression](const std::filesystem::path & tmp) {
    auto copts = inplace_copts;
    if (no_compression) {
      copts.mcap_compression = "none";
    }
    return io::open_write(tmp, copts);
  };

  // The pass reports command-level failures via its return value rather than
  // throwing, so capture the status and translate a non-zero exit into a
  // runtime_error to make write_bag_inplace abort the swap (leaving <input>
  // untouched).
  int pass_status = 0;
  try {
    core::write_bag_inplace(input_path, [&](const std::filesystem::path & tmp) {
      pass_status = pass([&]() { return make_inplace_writer(tmp); });
      if (pass_status != 0) {
        throw std::runtime_error(options.pass_failed_error);
      }
    });
  } catch (const std::exception & e) {
    // cppcheck-suppress knownConditionTrueFalse  // assigned inside the lambda above
    if (pass_status != 0) {
      return pass_status;  // the pass already logged the specific error
    }
    BAGWIZ_LOG_ERROR(options.logger, "In-place swap failed: %s", e.what());
    return 1;
  }
  return 0;
}

}  // namespace bagwiz::core
