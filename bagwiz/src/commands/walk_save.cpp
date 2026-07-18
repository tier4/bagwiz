// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "walk_save.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <fmt/core.h>

#include <fstream>
#include <istream>
#include <ostream>
#include <string>
#include <system_error>

namespace bagwiz::commands
{

std::string topic_for_filename(std::string_view topic)
{
  std::string out;
  out.reserve(topic.size() * 2);
  for (unsigned char uc : topic) {
    const char c = static_cast<char>(uc);
    if (c == '/') {
      out += "__";
    } else {
      out.push_back(c);
    }
  }
  return out;
}

std::filesystem::path resolve_save_path(
  const std::string & line_from_stdin, const std::filesystem::path & cwd,
  const std::string & default_filename)
{
  std::string trimmed = line_from_stdin;
  while (!trimmed.empty() && (trimmed.back() == ' ' || trimmed.back() == '\t')) {
    trimmed.pop_back();
  }
  if (trimmed.empty()) {
    return cwd / default_filename;
  }

  std::filesystem::path user_path(trimmed);
  std::error_code ec;
  if (std::filesystem::exists(user_path, ec) && std::filesystem::is_directory(user_path, ec)) {
    return user_path / default_filename;
  }
  const char last = trimmed.back();
  if (last == '/' || last == '\\') {
    return std::filesystem::path(trimmed) / default_filename;
  }
  return user_path;
}

WriteSaveResult write_save_file(
  const std::string & line_from_stdin, const std::filesystem::path & cwd,
  const std::string & default_base, std::span<const std::byte> data)
{
  WriteSaveResult result;
  result.path = resolve_save_path(line_from_stdin, cwd, default_base);
  std::error_code mk_ec;
  const auto parent = result.path.parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent, mk_ec);
    if (mk_ec) {
      result.error =
        fmt::format("could not create directory {}: {}", parent.string(), mk_ec.message());
      return result;
    }
  }
  std::ofstream of(result.path, std::ios::binary);
  if (!of) {
    result.error = fmt::format("could not open {} for writing", result.path.string());
    return result;
  }
  of.write(
    reinterpret_cast<const char *>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
      data.data()),
    static_cast<std::streamsize>(data.size()));
  if (!of.good()) {
    result.error = fmt::format("write failed: {}", result.path.string());
    return result;
  }
  return result;
}

bool save_bytes_with_prompt(
  core::tui::ScrollablePager & pager, std::string_view prompt_label,
  const std::filesystem::path & cwd, const std::string & default_base,
  std::span<const std::byte> data, std::string & status)
{
  const std::filesystem::path default_full = cwd / default_base;

  std::filesystem::path out_path;
  bool save_ok = false;
  std::string failure_status;
  pager.with_line_input([&](std::istream & in, std::ostream & out) {
    out << fmt::format("{} (Enter for {}):\n", prompt_label, default_full.string());
    out.flush();
    std::string line;
    if (!std::getline(in, line)) {
      failure_status = "(save cancelled)";
      return;
    }
    const auto result = write_save_file(line, cwd, default_base, data);
    out_path = result.path;
    if (!result.error.empty()) {
      failure_status = result.error;
      return;
    }
    save_ok = true;
  });
  if (save_ok) {
    status = fmt::format("saved {}", out_path.string());
  } else {
    status = failure_status;
  }
  return save_ok;
}

}  // namespace bagwiz::commands
