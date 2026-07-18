// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/msg_yaml/msg_definition_resolver.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bagwiz::core
{

namespace
{

// 80 `=` characters — the rosbag2 / mcap convention separating the
// top-level .msg body from each dependency's body.
constexpr const char * kSep =
  "================================================================================";

// ROS 2 .msg primitive type names. Anything matching here is a wire
// primitive that does not require recursive .msg resolution.
//
// Note: ROS 2 dropped the ROS 1 `time` and `duration` primitives in
// favour of `builtin_interfaces/Time` and `builtin_interfaces/Duration`,
// which appear as ordinary complex-type references in .msg text and
// resolve via the disk path like any other type.
const std::unordered_set<std::string> & primitive_types()
{
  static const std::unordered_set<std::string> kSet = {
    "bool",   "byte",  "char",   "int8",    "uint8",   "int16",  "uint16",  "int32",
    "uint32", "int64", "uint64", "float32", "float64", "string", "wstring",
  };
  return kSet;
}

// Split "pkg/msg/Type" (preferred) or "pkg/Type" (legacy) into the
// short canonical form ("pkg", "Type"). Returns false on malformed
// input.
bool split_ros2_type(std::string_view full, std::string & package, std::string & type)
{
  std::vector<std::string> parts;
  std::size_t start = 0;
  for (std::size_t i = 0; i < full.size(); ++i) {
    if (full[i] == '/') {
      parts.emplace_back(full.substr(start, i - start));
      start = i + 1;
    }
  }
  parts.emplace_back(full.substr(start));
  if (parts.size() == 3 && parts[1] == "msg") {
    package = parts[0];
    type = parts[2];
    return !package.empty() && !type.empty();
  }
  if (parts.size() == 2) {
    package = parts[0];
    type = parts[1];
    return !package.empty() && !type.empty();
  }
  return false;
}

bool read_file(const std::filesystem::path & path, std::string & out)
{
  std::ifstream f(path, std::ios::binary);
  if (!f.is_open()) {
    return false;
  }
  std::stringstream ss;
  ss << f.rdbuf();
  out = ss.str();
  return true;
}

// Locate `<pkg-share>/msg/<Type>.msg` via ament_index_cpp and read it.
// Returns false on package-not-found, file-not-found, or read errors.
bool load_msg_file(const std::string & package, const std::string & type, std::string & out)
{
  std::string share;
  try {
    share = ament_index_cpp::get_package_share_directory(package);
  } catch (...) {
    // Package not registered with the ament index — caller treats as
    // unresolvable, no special diagnosis here.
    return false;
  }
  const std::filesystem::path path = std::filesystem::path(share) / "msg" / (type + ".msg");
  return read_file(path, out);
}

// Extract the complex (non-primitive) type referenced by a single .msg
// line, applying the same-package convention. Returns "" for primitive
// fields, constants, blank lines, and comment-only lines.
//
// Recognised .msg syntactic shapes (see ROS 2 design):
//   <type> <name>                     — field
//   <type> <name> <default>           — field with default value
//   <type> <name> = <value>           — constant (skipped)
//   <type>[N] <name>                  — fixed-size array
//   <type>[] <name>                   — unbounded sequence
//   <type>[<=N] <name>                — bounded sequence
//
// We strip the array brackets to recover the element type, then check
// the primitive set. Same-package shorthand (`KeyValue values` inside
// `diagnostic_msgs/msg/DiagnosticStatus.msg`) is canonicalised against
// the parent package.
std::string extract_dep_type(std::string_view raw_line, const std::string & parent_package)
{
  std::string line(raw_line);

  // Strip inline comments and trailing whitespace; trim leading whitespace.
  if (auto hash = line.find('#'); hash != std::string::npos) {
    line.resize(hash);
  }
  while (!line.empty() && (line.back() == ' ' || line.back() == '\t' || line.back() == '\r' ||
                           line.back() == '\n')) {
    line.pop_back();
  }
  std::size_t lead = 0;
  while (lead < line.size() && (line[lead] == ' ' || line[lead] == '\t')) {
    ++lead;
  }
  if (lead >= line.size()) {
    return {};
  }
  const std::string body = line.substr(lead);

  // First token = type expression (with optional [...] array suffix).
  std::size_t sp = 0;
  while (sp < body.size() && body[sp] != ' ' && body[sp] != '\t') {
    ++sp;
  }
  if (sp == body.size()) {
    return {};  // single-token line, not a field
  }
  std::string type_token = body.substr(0, sp);

  // Strip array brackets to recover element type.
  if (auto br = type_token.find('['); br != std::string::npos) {
    type_token.resize(br);
  }
  if (type_token.empty()) {
    return {};
  }

  const std::string rest = body.substr(sp);
  const bool has_eq = rest.find('=') != std::string::npos;

  // Constants are always typed with primitives in practice; if a
  // primitive line carries '=', it's a constant declaration we don't
  // need to follow as a dep.
  const bool is_primitive = primitive_types().count(type_token) > 0;
  if (is_primitive && has_eq) {
    return {};
  }
  if (is_primitive) {
    return {};
  }

  // Same-package shorthand resolves against the parent's package.
  if (type_token.find('/') == std::string::npos) {
    return parent_package + "/" + type_token;
  }
  return type_token;
}

// Per-process cache of one .msg file's parsed contents.
struct CacheEntry
{
  bool ok = false;
  std::string body;                      // raw .msg text (with comments preserved)
  std::vector<std::string> direct_deps;  // canonicalised "pkg/Type" form, deduped
};

std::mutex & cache_mutex()
{
  static std::mutex m;
  return m;
}

std::unordered_map<std::string, CacheEntry> & cache()
{
  static std::unordered_map<std::string, CacheEntry> c;
  return c;
}

// Load and parse one type. Cached on success and on persistent failure
// (so repeated misses don't keep hitting the filesystem). `key` must be
// the canonical short form "pkg/Type".
const CacheEntry & load_one(const std::string & key)
{
  {
    std::lock_guard<std::mutex> lk(cache_mutex());
    auto it = cache().find(key);
    if (it != cache().end()) {
      return it->second;
    }
  }

  CacheEntry entry;
  std::string package;
  std::string type;
  if (!split_ros2_type(key, package, type)) {
    std::lock_guard<std::mutex> lk(cache_mutex());
    return cache()[key] = entry;
  }
  if (!load_msg_file(package, type, entry.body)) {
    std::lock_guard<std::mutex> lk(cache_mutex());
    return cache()[key] = entry;
  }

  // Walk the body line by line collecting unique direct deps.
  std::unordered_set<std::string> seen;
  std::stringstream ss(entry.body);
  std::string line;
  while (std::getline(ss, line)) {
    auto dep = extract_dep_type(line, package);
    if (dep.empty()) {
      continue;
    }
    if (seen.insert(dep).second) {
      entry.direct_deps.push_back(std::move(dep));
    }
  }
  entry.ok = true;

  std::lock_guard<std::mutex> lk(cache_mutex());
  return cache()[key] = std::move(entry);
}

}  // namespace

// cppcheck-suppress passedByValue
// std::string_view is the lightweight view type and is idiomatically
// passed by value; cppcheck's heuristic flags all view-by-value uses.
ResolvedMessageDefinition resolve_message_definition(std::string_view ros2_type)
{
  // Canonicalise input. Accept either the fully-qualified `pkg/msg/Type`
  // form (rosbag2's preferred shape) or the legacy `pkg/Type` form.
  std::string package;
  std::string type;
  if (!split_ros2_type(ros2_type, package, type)) {
    return {};
  }
  const std::string root_key = package + "/" + type;

  const auto & root = load_one(root_key);
  if (!root.ok) {
    return {};
  }

  // Flatten transitive deps into a deduped, deterministic order.
  // DFS-by-stack is fine here because the rosbag2 mcap parser doesn't
  // require topological ordering — only uniqueness.
  std::vector<std::string> deps_flat;
  std::unordered_set<std::string> seen{root_key};
  std::vector<std::string> stack(root.direct_deps.rbegin(), root.direct_deps.rend());

  while (!stack.empty()) {
    auto cur = std::move(stack.back());
    stack.pop_back();
    if (!seen.insert(cur).second) {
      continue;
    }
    const auto & ce = load_one(cur);
    if (!ce.ok) {
      // A required dep is missing — fail the whole resolution rather
      // than emit a partial / inconsistent definition.
      return {};
    }
    deps_flat.push_back(cur);
    for (auto it = ce.direct_deps.rbegin(); it != ce.direct_deps.rend(); ++it) {
      if (seen.find(*it) == seen.end()) {
        stack.push_back(*it);
      }
    }
  }

  // Assemble the rosbag2-style concatenated text.
  std::string text = root.body;
  if (!text.empty() && text.back() != '\n') {
    text.push_back('\n');
  }
  for (const auto & dep : deps_flat) {
    const auto & ce = load_one(dep);
    text.append(kSep);
    text.push_back('\n');
    text.append("MSG: ");
    text.append(dep);
    text.push_back('\n');
    text.append(ce.body);
    if (text.empty() || text.back() != '\n') {
      text.push_back('\n');
    }
  }

  ResolvedMessageDefinition out;
  out.text = std::move(text);
  out.encoding = "ros2msg";
  return out;
}

}  // namespace bagwiz::core
