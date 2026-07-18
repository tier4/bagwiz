// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__WALK_SAVE_HPP_
#define COMMANDS__WALK_SAVE_HPP_

#include "bagwiz/core/tui/pager.hpp"

#include <cstddef>
#include <filesystem>
#include <span>
#include <string>
#include <string_view>

// Save-to-disk plumbing of `bagwiz walk`: the save-path resolution shared by
// the YAML and PNG save prompts, and the prompt/mkdir/write sequence itself
// (both save handlers run the same flow). The path resolution and file write
// are TTY-free and unit-tested; only the prompt needs the pager.
// CLI-internal: this header lives with the command sources and is not
// installed.
namespace bagwiz::commands
{

// ROS topic names use `/`; replace each `/` with `__` so path separators
// do not collide with underscores that appear inside topic name segments.
[[nodiscard]] std::string topic_for_filename(std::string_view topic);

// Resolve the user's answer at the save prompt: trailing blanks are ignored;
// an empty answer picks `cwd / default_filename`; an existing directory (or
// a path ending in a separator) picks the default name inside it; anything
// else is used as the target path verbatim.
[[nodiscard]] std::filesystem::path resolve_save_path(
  const std::string & line_from_stdin, const std::filesystem::path & cwd,
  const std::string & default_filename);

// Outcome of write_save_file(). `path` is the resolved target; `error` is
// empty on success and carries the failure status text otherwise.
struct WriteSaveResult
{
  std::filesystem::path path;
  std::string error;
};

// TTY-free core of the save flow: resolve the target from the user's input
// line, create the parent directory, and write `data`.
[[nodiscard]] WriteSaveResult write_save_file(
  const std::string & line_from_stdin, const std::filesystem::path & cwd,
  const std::string & default_base, std::span<const std::byte> data);

// Prompt for a save path via the pager's cooked-mode line input and write
// `data` to the resolved path. `prompt_label` is the leading prompt text
// (e.g. "Save YAML path"). On return `status` holds the outcome surfaced in
// the UI ("saved <path>", "(save cancelled)", or the failure reason);
// returns whether the file was written.
bool save_bytes_with_prompt(
  core::tui::ScrollablePager & pager, std::string_view prompt_label,
  const std::filesystem::path & cwd, const std::string & default_base,
  std::span<const std::byte> data, std::string & status);

}  // namespace bagwiz::commands

#endif  // COMMANDS__WALK_SAVE_HPP_
