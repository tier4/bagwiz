// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__BAG__REWRITE_HPP_
#define BAGWIZ__CORE__BAG__REWRITE_HPP_

#include "bagwiz/io/bag_open.hpp"

#include <filesystem>
#include <functional>
#include <optional>

// The shared "-o vs in-place" dispatch every rewrite-style command
// (topic drop/keep/rename, cam-info replace/recompute-p, convert msg geo,
// pcd concat/undistort, tf static cp, traj join) runs after validating its
// arguments:
//
//   * with -o: guard the output path (prepare_output_path), then run the
//     command's pass with a writer factory targeting the output path.
//   * without -o: rewrite <input> atomically via write_bag_inplace, running
//     the same pass against the sibling tmp path with the input's storage
//     format and layout pinned (the tmp suffix defeats Auto detection), and
//     abort the swap when the pass reports failure.
//
// Centralising the dispatch keeps the clobber policy, the Format::Auto guard,
// the mcap_compression override, and the pass-status-to-exception translation
// identical across commands. The pass itself stays with the command.
namespace bagwiz::core
{

// Knobs and message texts for run_bag_rewrite. The messages are parameters
// (not fixed strings) because each command has always emitted its own
// wording; they must stay byte-identical to what the command printed before
// the dispatch was shared.
struct BagRewriteOptions
{
  // Logger name used for every message the dispatch emits.
  const char * logger = nullptr;

  // Logged (ERROR) in in-place mode when the input's storage format cannot be
  // detected. printf-style format with exactly one "%s", filled with the
  // input path.
  const char * format_unknown_error = nullptr;

  // Text of the std::runtime_error thrown to abort the in-place swap when the
  // pass returns non-zero. Never printed: the catch path returns the pass's
  // exit code, since the pass has already logged the specific error.
  const char * pass_failed_error = nullptr;

  // -o mode only: compose the writer's CreateOptions via
  // io::create_options_inheriting_format(input, output), so a directory
  // output inherits the input's storage format while a .mcap/.db3 extension
  // still picks a single-file backend. When false, Format::Auto /
  // Layout::Auto is used and the factory resolves purely from the output
  // path's extension.
  bool inherit_output_format = false;

  // Force mcap_compression = "none" on the writer options (both modes).
  // Rewrite commands disable compression so a bag that is rewritten often
  // does not pay the (de)compression cost each time; commands that want the
  // storage default set this to false.
  bool disable_mcap_compression = true;
};

// The command's rewrite pass. Receives the writer factory chosen by the
// dispatch (output path in -o mode, sibling tmp path in-place) and returns a
// process exit code: 0 on success; non-zero on failure after logging the
// specific error itself.
using BagRewritePass = std::function<int(const io::WriterFactory & open_writer)>;

// Run the -o / in-place dispatch for a rewrite-style command.
//
// `input_path` is the bag read by the pass and the path rewritten in place
// when `output_path` is nullopt (for tf-static-cp-style commands it is the
// destination bag). `overwrite` only governs the -o branch's clobber policy;
// in-place mode always replaces <input>.
//
// Returns the pass's exit code, or 1 when the dispatch itself fails (output
// collision, undetectable input format, in-place swap error).
int run_bag_rewrite(
  const std::filesystem::path & input_path,
  const std::optional<std::filesystem::path> & output_path, bool overwrite,
  const BagRewriteOptions & options, const BagRewritePass & pass);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__BAG__REWRITE_HPP_
