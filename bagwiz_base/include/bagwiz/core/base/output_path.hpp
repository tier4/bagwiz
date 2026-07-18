// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__BASE__OUTPUT_PATH_HPP_
#define BAGWIZ__CORE__BASE__OUTPUT_PATH_HPP_

#include <filesystem>
#include <string>

// Shared "clobber" policy for every bagwiz subcommand that materialises a
// file or directory output. Centralising the check here means
// `bagwiz convert`, `bagwiz traj dump`, and `bagwiz traj join -o` all
// behave identically: any pre-existing entry at the chosen output path
// stops the run unless the user opts in to replacement with
// `--overwrite`. Subcommands call the helper once,
// immediately after argument parsing, before opening any writer.
namespace bagwiz::core
{

struct PrepareOutputResult
{
  bool ok = false;
  // Populated when ok is false. Phrased so the caller can log it verbatim
  // without prepending extra context.
  std::string error;
};

// Decide whether `path` is free to write into.
//
//   * `path` does not exist            -> ok=true; caller proceeds.
//   * `path` exists, overwrite=false   -> ok=false; error explains the
//                                         collision and points at --overwrite.
//   * `path` exists, overwrite=true    -> The existing entry is removed
//                                         (recursively for directories) so
//                                         the writer starts from a clean
//                                         slot. ok=true on success; ok=false
//                                         with a filesystem-error message
//                                         when removal fails (permission
//                                         denied, in-use file, ...).
//
// The function never throws; filesystem errors are reported through the
// result so the caller's error path stays a single shape.
PrepareOutputResult prepare_output_path(const std::filesystem::path & path, bool overwrite);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__BASE__OUTPUT_PATH_HPP_
