// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tf/tf_static_tree_yaml.hpp"

#include "bagwiz/core/tf/tf_forest_check.hpp"
#include "bagwiz/core/tf/tf_transform_format.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <iomanip>
#include <set>
#include <span>
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

// Significant digits for every emitted number. This file's job is a readable,
// hand-editable config, so it deliberately does not round-trip the doubles
// bit-for-bit: the shortest exact form would render a calibration's -0.650337 as
// -0.65033700000000005 and, worse, expose the 1-4 ULP that the quaternion -> RPY
// conversion adds, turning -0.002701 into -0.0027009999999999795.
//
// 14 is where that noise stops showing. A double carries ~15.95 decimal digits,
// and a few ULP of error consume the last ~1.5 of them, so 14 is the precision
// that actually survives this command's conversion — not a value tuned to one
// bag. The cost is ~1e-14 relative error: on a translation in metres that is
// hundredths of a picometre, far below what any calibration resolves.
// `bagwiz tf static calc --json` is the full-precision view.
constexpr int kSignificantDigits = 14;

// Absolute floor below which an emitted ANGLE is treated as zero, in radians.
//
// kSignificantDigits folds noise that scales with the value, but a component
// whose true value is 0 gets absolute error instead, which no relative precision
// removes. Recovering RPY from a quaternion cannot represent an exact zero next
// to a right angle: the camera_optical rotation (roll = -pi/2, yaw = -pi/2) comes
// back with pitch = -5.55e-17 rather than 0, so a config written by `dump`, read
// by a publisher, and dumped again would not match itself.
//
// 1e-12 rad sits three orders above the ~1e-15 worst case for a few ULP of a
// right angle, and six below the microradian that is the finest any real
// extrinsic calibration resolves — so it can only ever erase noise. It is
// deliberately NOT applied to translations, which reach the emitter straight from
// the bag and never pass through this conversion.
constexpr double kAngleZeroFloor = 1e-12;

// The transforms sharing one parent frame, in first-seen order. Pointers alias
// the caller's span, which outlives the emit call.
struct ParentGroup
{
  std::string parent;
  std::vector<const geometry_msgs::msg::TransformStamped *> children;
};

constexpr char to_lower_ascii(char c)
{
  return (c >= 'A' && c <= 'Z') ? static_cast<char>(c - 'A' + 'a') : c;
}

// True when a YAML reader would resolve `name` as a bool or null rather than a
// string. YAML 1.1 readers (PyYAML, and so every ROS launch/param loader that
// goes through it) treat all of these as non-strings, which would turn a frame
// named `no` into `false`, so such a name has to be quoted.
bool is_yaml_keyword(std::string_view name)
{
  static constexpr std::array<std::string_view, 9> kKeywords{"y",     "n",  "yes", "no",  "true",
                                                             "false", "on", "off", "null"};
  std::string lower;
  lower.reserve(name.size());
  for (const char c : name) {
    lower += to_lower_ascii(c);
  }
  return std::find(kKeywords.begin(), kKeywords.end(), lower) != kKeywords.end();
}

// Characters allowed in an unquoted frame id. Deliberately narrower than YAML's
// plain-scalar rules: ':' is excluded because readers disagree on whether
// `a:b` is one scalar or a nested key, and every real ROS frame id
// ("base_link", "camera0/camera_link") fits without it.
constexpr bool is_plain_body_char(char c)
{
  return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' ||
         c == '/' || c == '.' || c == '-';
}

bool is_plain_safe(std::string_view name)
{
  if (name.empty()) {
    return false;
  }
  // Requiring a letter or '_' first also rules out anything a reader would
  // resolve as a number, a date, or a YAML tag/anchor sigil.
  const char first = name.front();
  const bool alpha = (first >= 'a' && first <= 'z') || (first >= 'A' && first <= 'Z');
  if (!alpha && first != '_') {
    return false;
  }
  if (!std::all_of(name.begin(), name.end(), is_plain_body_char)) {
    return false;
  }
  return !is_yaml_keyword(name);
}

// Double-quoted YAML scalar with escapes. Unlike the single-quoted style this
// can represent any byte string, including the control characters a
// pathological frame id could carry.
std::string quote_double(std::string_view name)
{
  static constexpr std::string_view kHex = "0123456789abcdef";
  std::string out = "\"";
  for (const char c : name) {
    const auto byte = static_cast<unsigned char>(c);
    switch (c) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        if (byte < 0x20 || byte == 0x7f) {
          out += "\\x";
          out += kHex[(byte >> 4U) & 0xFU];
          out += kHex[byte & 0xFU];
        } else {
          // >= 0x80 passes through: UTF-8 is valid inside a quoted scalar.
          out += c;
        }
        break;
    }
  }
  out += '"';
  return out;
}

// A frame id rendered as a YAML mapping key, quoted only when it has to be so
// ordinary output stays as readable as a hand-written config.
std::string yaml_key(std::string_view name)
{
  return is_plain_safe(name) ? std::string(name) : quote_double(name);
}

std::string format_double(double v)
{
  // Non-finite values cannot come from a sane calibration, but a corrupt bag
  // can carry them. `nan` / `inf` as written by ostream are plain scalars a
  // YAML reader resolves to the *strings* "nan"/"inf", so the file would fail
  // to load as a float; YAML's own .nan / .inf spellings at least stay typed.
  if (std::isnan(v)) {
    return ".nan";
  }
  if (std::isinf(v)) {
    return v > 0.0 ? ".inf" : "-.inf";
  }
  // -0.0 == 0.0, so this also folds the negative zero that getRPY hands back for
  // an identity rotation (its pitch is -asin(0.0)). "-0.0" would load as the same
  // number, but it reads as a suspicious value in a config a human maintains.
  if (v == 0.0) {
    return "0.0";
  }

  std::ostringstream oss;
  oss << std::setprecision(kSignificantDigits) << v;
  std::string s = oss.str();
  // Keep every value unambiguously a float: bare `0` loads as an integer, which
  // a strictly-typed consumer rejects where `0.0` is accepted.
  if (
    s.find('.') == std::string::npos && s.find('e') == std::string::npos &&
    s.find('E') == std::string::npos) {
    s += ".0";
  }
  return s;
}

// format_double for an angle: see kAngleZeroFloor.
std::string format_angle(double v)
{
  return format_double(std::abs(v) < kAngleZeroFloor ? 0.0 : v);
}

// Group the transforms by parent frame, preserving the order parents and
// children were first seen. `children_seen` collects every child_frame_id, which
// is what identifies the roots (a parent that is nobody's child).
std::vector<ParentGroup> group_by_parent(
  std::span<const geometry_msgs::msg::TransformStamped> transforms,
  std::unordered_map<std::string, std::size_t> & group_index,
  std::unordered_set<std::string> & children_seen)
{
  std::vector<ParentGroup> groups;
  for (const auto & t : transforms) {
    const auto ins = group_index.emplace(t.header.frame_id, groups.size());
    if (ins.second) {
      groups.push_back({t.header.frame_id, {}});
    }
    groups[ins.first->second].children.push_back(&t);
    children_seen.insert(t.child_frame_id);
  }
  return groups;
}

// Indices into `groups`, breadth-first from every root in first-seen order, then
// whatever the walk could not reach (a cycle) so no group is lost.
std::vector<std::size_t> breadth_first_order(
  const std::vector<ParentGroup> & groups,
  const std::unordered_map<std::string, std::size_t> & group_index,
  const std::unordered_set<std::string> & children_seen)
{
  std::vector<std::string> queue;
  for (const auto & group : groups) {
    if (!children_seen.contains(group.parent)) {
      queue.push_back(group.parent);
    }
  }

  std::vector<std::size_t> order;
  order.reserve(groups.size());
  std::vector<bool> emitted(groups.size(), false);
  for (std::size_t head = 0; head < queue.size(); ++head) {
    const auto found = group_index.find(queue[head]);
    if (found == group_index.end() || emitted[found->second]) {
      // A leaf frame parents nothing, so it has no group of its own.
      continue;
    }
    emitted[found->second] = true;
    order.push_back(found->second);
    for (const auto * t : groups[found->second].children) {
      queue.push_back(t->child_frame_id);
    }
  }

  for (std::size_t i = 0; i < groups.size(); ++i) {
    if (!emitted[i]) {
      order.push_back(i);
    }
  }
  return order;
}

// `label` with line breaks folded to spaces, so a path containing one cannot
// escape the `#` comment it is written into and inject YAML.
std::string single_line(std::string_view label)
{
  std::string out(label);
  std::replace(out.begin(), out.end(), '\n', ' ');
  std::replace(out.begin(), out.end(), '\r', ' ');
  return out;
}

// The six keys a transform mapping must carry, and only these. Order matches the
// emitted order.
constexpr std::array<std::string_view, 6> kTransformKeys{"x", "y", "z", "roll", "pitch", "yaw"};

// `parent -> child` for error messages, matching the arrow the emitter's docs
// and `tf tree` use.
std::string edge_label(const std::string & parent, const std::string & child)
{
  return "'" + parent + "' -> '" + child + "'";
}

// Read one transform mapping into `out`. Returns false and sets `error` when a
// key is missing, unknown, or not a number. Unknown keys are rejected rather
// than ignored: a mistyped `pich` would otherwise leave pitch silently at 0.
bool read_transform_mapping(
  const YAML::Node & node, const std::string & parent, const std::string & child,
  geometry_msgs::msg::TransformStamped & out, std::string & error)
{
  double values[kTransformKeys.size()] = {};
  for (std::size_t i = 0; i < kTransformKeys.size(); ++i) {
    const std::string key(kTransformKeys[i]);
    const YAML::Node value = node[key];
    if (!value) {
      error = "transform " + edge_label(parent, child) + " is missing key '" + key + "'";
      return false;
    }
    if (!value.IsScalar()) {
      error = "transform " + edge_label(parent, child) + " key '" + key + "' must be a number";
      return false;
    }
    try {
      values[i] = value.as<double>();
    } catch (const YAML::Exception & e) {
      error = "transform " + edge_label(parent, child) + " key '" + key +
              "' must be a number: " + e.what();
      return false;
    }
  }

  for (const auto & entry : node) {
    const auto key = entry.first.as<std::string>();
    const bool known =
      std::find(kTransformKeys.begin(), kTransformKeys.end(), key) != kTransformKeys.end();
    if (!known) {
      error = "transform " + edge_label(parent, child) + " has unknown key '" + key +
              "'; only x, y, z, roll, pitch, yaw are allowed";
      return false;
    }
  }

  out.header.frame_id = parent;
  out.child_frame_id = child;
  out.transform.translation.x = values[0];
  out.transform.translation.y = values[1];
  out.transform.translation.z = values[2];
  out.transform.rotation = rpy_to_quaternion({values[3], values[4], values[5]});
  return true;
}

// True when `node` carries at least one of the six transform keys, i.e. it is
// meant to be a transform rather than a further nesting level.
bool has_transform_fields(const YAML::Node & node)
{
  return std::any_of(kTransformKeys.begin(), kTransformKeys.end(), [&node](std::string_view key) {
    return static_cast<bool>(node[std::string(key)]);
  });
}

// True when `node` is a mapping that carries none of the six transform keys,
// i.e. a further nesting level rather than a transform. The reference publisher
// recurses into such a node and silently drops the enclosing key; this parser
// reports it instead, since a dropped level means a transform lands under the
// wrong parent.
bool is_deeper_nesting(const YAML::Node & node)
{
  return node.IsMap() && !has_transform_fields(node);
}

}  // namespace

std::string emit_static_tf_tree_yaml(
  std::span<const geometry_msgs::msg::TransformStamped> transforms,
  // cppcheck-suppress passedByValue  // string_view is the canonical by-value idiom
  std::string_view source_label)
{
  std::unordered_map<std::string, std::size_t> group_index;
  std::unordered_set<std::string> children_seen;
  const std::vector<ParentGroup> groups = group_by_parent(transforms, group_index, children_seen);
  const std::vector<std::size_t> order = breadth_first_order(groups, group_index, children_seen);

  // std::ostringstream rather than fmt: bagwiz_tf carries no fmt dependency —
  // the same reason tf_transform_format.cpp avoids it. The structure is two
  // nested mappings, so it is emitted directly rather than through yaml-cpp,
  // which would neither give the forced `0.0` nor the blank line per group.
  std::ostringstream oss;
  oss << "# Static TF tree dumped by `bagwiz tf static dump`.\n";
  oss << "# Source bag: " << single_line(source_label) << "\n";
  oss << "# Rotations are roll/pitch/yaw in radians, in tf2's fixed-axis convention:\n";
  oss << "# tf2::Quaternion::setRPY(roll, pitch, yaw) reproduces the source quaternion.\n";
  for (const std::size_t idx : order) {
    const auto & group = groups[idx];
    oss << "\n" << yaml_key(group.parent) << ":\n";
    for (const auto * t : group.children) {
      const auto & tr = t->transform.translation;
      const RollPitchYaw rpy = quaternion_to_rpy(t->transform.rotation);
      oss << "  " << yaml_key(t->child_frame_id) << ":\n";
      oss << "    x: " << format_double(tr.x) << "\n";
      oss << "    y: " << format_double(tr.y) << "\n";
      oss << "    z: " << format_double(tr.z) << "\n";
      oss << "    roll: " << format_angle(rpy.roll) << "\n";
      oss << "    pitch: " << format_angle(rpy.pitch) << "\n";
      oss << "    yaw: " << format_angle(rpy.yaw) << "\n";
    }
  }
  return oss.str();
}

StaticTfTreeParseResult parse_static_tf_tree_yaml(const std::filesystem::path & yaml_path)
{
  StaticTfTreeParseResult result;
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path.string());
  } catch (const YAML::Exception & e) {
    result.error = "failed to parse static TF YAML '" + yaml_path.string() + "': " + e.what();
    return result;
  }
  if (!root || !root.IsMap()) {
    result.error =
      "static TF YAML '" + yaml_path.string() + "' is empty or not a top-level mapping";
    return result;
  }

  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  std::set<std::pair<std::string, std::string>> edges;
  try {
    for (const auto & parent_entry : root) {
      const auto parent = parent_entry.first.as<std::string>();
      if (parent.empty()) {
        result.error = "a parent frame id is empty";
        return result;
      }
      const YAML::Node & children = parent_entry.second;
      if (!children.IsMap()) {
        result.error = "parent frame '" + parent +
                       "' must hold a mapping of child frames, each with x, y, z, roll, pitch, yaw";
        return result;
      }
      if (children.size() == 0) {
        result.error = "parent frame '" + parent + "' declares no child frames";
        return result;
      }
      // One level too shallow: the top-level entry is itself a transform, so the
      // parent frame is missing. Without this the loop below would report the
      // first of the six keys as a malformed child frame, which points at the
      // wrong thing. The reference publisher instead broadcasts the transform
      // with an EMPTY parent frame id, which is broken TF either way.
      if (has_transform_fields(children)) {
        result.error = "'" + parent +
                       "' declares a transform directly; this schema needs a parent frame above it "
                       "(parent, then child holding x, y, z, roll, pitch, yaw)";
        return result;
      }

      for (const auto & child_entry : children) {
        const auto child = child_entry.first.as<std::string>();
        if (child.empty()) {
          result.error = "a child frame id under parent '" + parent + "' is empty";
          return result;
        }
        const YAML::Node & body = child_entry.second;
        if (!body.IsMap()) {
          result.error = "transform " + edge_label(parent, child) +
                         " must be a mapping with x, y, z, roll, pitch, yaw";
          return result;
        }
        // A third level means the file nests deeper than this schema. The
        // reference publisher would flatten it and drop `parent`, putting the
        // transform under the wrong frame, so refuse instead of guessing.
        if (is_deeper_nesting(body)) {
          result.error = "'" + parent + "' -> '" + child +
                         "' nests a further level; this schema is exactly two levels deep "
                         "(parent, then child holding x, y, z, roll, pitch, yaw)";
          return result;
        }
        if (parent == child) {
          result.error = "frame '" + parent + "' is its own parent";
          return result;
        }

        geometry_msgs::msg::TransformStamped transform;
        if (!read_transform_mapping(body, parent, child, transform, result.error)) {
          return result;
        }
        transforms.push_back(transform);
        edges.emplace(parent, child);
      }
    }
  } catch (const YAML::Exception & e) {
    // A non-scalar mapping key (`? [a, b] : ...`) is the remaining way a
    // well-formed document can still fail .as<std::string>().
    result.error =
      "static TF YAML '" + yaml_path.string() + "' has a malformed frame id: " + e.what();
    return result;
  }

  if (transforms.empty()) {
    result.error = "static TF YAML '" + yaml_path.string() + "' declares no transforms";
    return result;
  }
  // Everything above is per-edge; this is the only check on the set as a whole,
  // and it is what rejects a child claimed by two parents, opposite edges, and
  // cycles. Same validation `bagwiz tf tree` applies to a merged bag tree.
  if (const auto err = validate_tf_forest(edges, "in '" + yaml_path.string() + "'")) {
    result.error = *err;
    return result;
  }

  result.transforms = std::move(transforms);
  return result;
}

}  // namespace bagwiz::core
