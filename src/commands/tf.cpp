// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/logging.hpp"
#include "bagwiz/core/tf_transform_format.hpp"
#include "bagwiz/core/tf_value_extract.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <rang.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <fmt/core.h>
#include <tf2/buffer_core.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kLogger = "bagwiz.cmd.tf";
constexpr const char * kTfMessageType = "tf2_msgs/msg/TFMessage";
constexpr std::string_view kTfStaticSuffix = "tf_static";

bool is_static_tf_topic(std::string_view topic_name)
{
  if (topic_name.size() < kTfStaticSuffix.size()) {
    return false;
  }
  return topic_name.compare(
           topic_name.size() - kTfStaticSuffix.size(), kTfStaticSuffix.size(), kTfStaticSuffix) ==
         0;
}

struct TfTopic
{
  std::string name;
  bool is_static;
};

std::vector<TfTopic> collect_tf_topics(const io::BagReader & reader)
{
  std::vector<TfTopic> topics;
  for (const auto & t : reader.topics()) {
    if (t.type == kTfMessageType) {
      topics.push_back({t.name, is_static_tf_topic(t.name)});
    }
  }
  return topics;
}

// Replay the TF topics once: feed every contained TransformStamped into
// `buffer` with the correct static/dynamic flag, and (optionally) collect
// the distinct parent→child edges observed on static and dynamic topics.
//
// Decoding goes through the unified open_decoder() path so for MCAP
// inputs the schema-driven backend handles the work and tf2_msgs no
// longer needs to be on AMENT_PREFIX_PATH at runtime; only its
// header-only struct definition is required at build time (via
// extract_tf_message → geometry_msgs::msg::TransformStamped).
void load_tf(
  const std::filesystem::path & bag_path, const std::vector<TfTopic> & tf_topics,
  tf2::BufferCore & buffer,
  std::set<std::pair<std::string, std::string>> * static_edges_out = nullptr,
  std::set<std::pair<std::string, std::string>> * dynamic_edges_out = nullptr)
{
  auto tf_reader = io::open_read(bag_path);
  io::ReadFilter filter;
  for (const auto & t : tf_topics) {
    filter.topics.push_back(t.name);
  }
  tf_reader->set_filter(filter);

  std::unordered_map<std::string, bool> is_static_by_topic;
  for (const auto & t : tf_topics) {
    is_static_by_topic[t.name] = t.is_static;
  }

  // One decoder per TF topic so the schema_text differences across
  // shards / topics are handled by the factory rather than us.
  std::unordered_map<std::string, std::unique_ptr<core::decoder::Decoder>> decoder_by_topic;
  for (const auto & topic_info : tf_reader->topics()) {
    if (topic_info.type != kTfMessageType) {
      continue;
    }
    if (is_static_by_topic.find(topic_info.name) == is_static_by_topic.end()) {
      continue;
    }
    auto open = core::decoder::open_decoder(topic_info);
    if (!open.ok()) {
      throw std::runtime_error(
        "Could not open decoder for TF topic '" + topic_info.name + "': " + open.error);
    }
    decoder_by_topic.emplace(topic_info.name, std::move(open.decoder));
  }

  io::RawMessage raw;
  while (tf_reader->next(raw)) {
    auto it = decoder_by_topic.find(raw.topic->name);
    if (it == decoder_by_topic.end()) {
      continue;
    }
    const auto decoded = it->second->decode(raw.payload);
    if (!decoded.ok()) {
      throw std::runtime_error(
        "Failed to decode TF message on '" + raw.topic->name + "': " + decoded.error);
    }
    const auto transforms = core::extract_tf_message(*decoded.value);
    const bool is_static = is_static_by_topic.at(raw.topic->name);
    for (const auto & t : transforms) {
      if (!t.header.frame_id.empty() && !t.child_frame_id.empty()) {
        const auto edge = std::make_pair(t.header.frame_id, t.child_frame_id);
        if (is_static) {
          if (static_edges_out != nullptr) {
            static_edges_out->insert(edge);
          }
        } else {
          if (dynamic_edges_out != nullptr) {
            dynamic_edges_out->insert(edge);
          }
        }
      }
      buffer.setTransform(t, "bagwiz", is_static);
    }
  }
}

bool stdout_use_color()
{
  return ::isatty(STDOUT_FILENO) && std::getenv("NO_COLOR") == nullptr;
}

struct TreeGlyphs
{
  std::string branch_mid;
  std::string branch_end;
  std::string vertical_pad;
  std::string root_prefix;
};

TreeGlyphs make_tree_glyphs()
{
  if (std::getenv("BAGWIZ_TF_TREE_ASCII") != nullptr) {
    return {"|-- ", "`-- ", "|   ", ""};
  }
  return {
    "\u251c\u2500\u2500 ",
    "\u2514\u2500\u2500 ",
    "\u2502   ",
    "\u25cf ",
  };
}

std::string tf_section_rule(const char * label, bool use_color)
{
  const std::string row = std::string("\u2550\u2550\u2550 ") + label + " \u2550\u2550\u2550\n";
  if (!use_color) {
    return row;
  }
  rang::setControlMode(rang::control::Force);
  std::ostringstream oss;
  oss << rang::style::bold << row << rang::style::reset;
  std::string out = oss.str();
  rang::setControlMode(rang::control::Auto);
  return out;
}

std::string tf_format_tf_topics_section(
  const char * title, const std::string & topic_list_csv, bool use_color)
{
  std::string out = tf_section_rule(title, use_color);
  if (!use_color) {
    out += "  " + topic_list_csv + "\n\n";
    return out;
  }
  rang::setControlMode(rang::control::Force);
  std::ostringstream oss;
  oss << rang::fg::gray << "  " << topic_list_csv << '\n' << rang::style::reset;
  out += oss.str();
  rang::setControlMode(rang::control::Auto);
  out += '\n';
  return out;
}

enum class EdgeTfKind { kStaticOnly, kDynamicOnly, kBoth };

const char * tf_edge_kind_tag(EdgeTfKind kind)
{
  switch (kind) {
    case EdgeTfKind::kStaticOnly:
      return " [S]";
    case EdgeTfKind::kDynamicOnly:
      return " [D]";
    case EdgeTfKind::kBoth:
      return " [B]";
  }
  return "";
}

EdgeTfKind classify_tf_edge(
  const std::set<std::pair<std::string, std::string>> & static_edges,
  const std::set<std::pair<std::string, std::string>> & dynamic_edges, const std::string & parent,
  const std::string & child)
{
  const std::pair<std::string, std::string> pr{parent, child};
  const bool ins = static_edges.count(pr) != 0;
  const bool ind = dynamic_edges.count(pr) != 0;
  if (ins && ind) {
    return EdgeTfKind::kBoth;
  }
  if (ins) {
    return EdgeTfKind::kStaticOnly;
  }
  return EdgeTfKind::kDynamicOnly;
}

std::string tf_colored_tree_root_line(const std::string & text, bool use_color)
{
  if (!use_color) {
    return text;
  }
  rang::setControlMode(rang::control::Force);
  std::ostringstream oss;
  oss << rang::style::bold << text << rang::style::reset;
  std::string out = oss.str();
  rang::setControlMode(rang::control::Auto);
  return out;
}

std::string tf_colored_tree_edge_line(
  const std::string & prefix, const std::string & branch, const std::string & child,
  const std::string & suffix, bool use_color, EdgeTfKind kind)
{
  const char * const tag = tf_edge_kind_tag(kind);
  if (!use_color) {
    return prefix + branch + child + std::string(tag) + suffix;
  }
  rang::setControlMode(rang::control::Force);
  std::ostringstream oss;
  oss << rang::fg::gray << prefix << branch << rang::style::reset;
  // Bright blue / yellow / magenta read apart under common color-vision deficiency (avoid green vs
  // cyan).
  switch (kind) {
    case EdgeTfKind::kStaticOnly:
      oss << rang::fgB::blue;
      break;
    case EdgeTfKind::kDynamicOnly:
      oss << rang::fgB::yellow;
      break;
    case EdgeTfKind::kBoth:
      oss << rang::fgB::magenta;
      break;
  }
  oss << child << rang::style::reset;
  oss << rang::fg::gray << tag << suffix << rang::style::reset;
  std::string out = oss.str();
  rang::setControlMode(rang::control::Auto);
  return out;
}

// Legend block: same double-line emphasis as the tree section header (not a `#` comment).
std::string tf_format_tree_legend(bool use_color)
{
  std::string out = tf_section_rule("Legend", use_color);
  if (!use_color) {
    out += "  static-only [S] · dynamic-only [D] · both [B]\n\n";
    return out;
  }
  rang::setControlMode(rang::control::Force);
  std::ostringstream oss;
  oss << rang::fg::gray << "  " << rang::style::reset;
  oss << rang::fgB::blue << "static-only" << rang::style::reset;
  oss << rang::fg::gray << " [S] · " << rang::style::reset;
  oss << rang::fgB::yellow << "dynamic-only" << rang::style::reset;
  oss << rang::fg::gray << " [D] · " << rang::style::reset;
  oss << rang::fgB::magenta << "both" << rang::style::reset;
  oss << rang::fg::gray << " [B]\n" << rang::style::reset;
  out += oss.str();
  rang::setControlMode(rang::control::Auto);
  out += "\n";
  return out;
}

// Validates edge union for a forest (unique parent per child, no A→B together with B→A, no cycles).
// `kind_label` is "Static" or "Dynamic" for error messages.
std::optional<std::string> validate_union_edge_set(
  const std::set<std::pair<std::string, std::string>> & edges, const char * kind_label)
{
  for (const auto & pr : edges) {
    if (pr.first == pr.second) {
      return fmt::format(
        "{} TF union: self-referential edge '{}' -> '{}' is not allowed.", kind_label, pr.first,
        pr.second);
    }
  }

  for (const auto & pr : edges) {
    if (pr.first >= pr.second) {
      continue;
    }
    if (edges.count({pr.second, pr.first}) != 0) {
      return fmt::format(
        "{} TF union: opposite edges '{}' -> '{}' and '{}' -> '{}' cannot both appear.", kind_label,
        pr.first, pr.second, pr.second, pr.first);
    }
  }

  std::unordered_map<std::string, std::string> child_to_parent;
  for (const auto & pr : edges) {
    auto ins = child_to_parent.emplace(pr.second, pr.first);
    if (!ins.second && ins.first->second != pr.first) {
      return fmt::format(
        "{} TF union: child frame '{}' has parent '{}' in one transform and '{}' in another.",
        kind_label, pr.second, ins.first->second, pr.first);
    }
  }

  std::unordered_set<std::string> all_nodes;
  for (const auto & pr : edges) {
    all_nodes.insert(pr.first);
    all_nodes.insert(pr.second);
  }

  for (const auto & start : all_nodes) {
    std::unordered_set<std::string> seen_on_path;
    std::string cur = start;
    for (;;) {
      auto pit = child_to_parent.find(cur);
      if (pit == child_to_parent.end()) {
        break;
      }
      cur = pit->second;
      if (!seen_on_path.insert(cur).second) {
        return fmt::format(
          "{} TF union: edges contain a directed cycle (revisited frame '{}').", kind_label, cur);
      }
    }
  }

  return std::nullopt;
}

// Forest from an adjacency map (parent → sorted children) and sorted roots; each branch line
// colors the child frame by whether that parent→child edge appeared in static-only, dynamic-only,
// or both topic classes.
std::string format_merged_parent_map_forest(
  const std::unordered_map<std::string, std::vector<std::string>> & parent_to_children,
  const std::vector<std::string> & roots_sorted,
  const std::set<std::pair<std::string, std::string>> & static_edges,
  const std::set<std::pair<std::string, std::string>> & dynamic_edges, const TreeGlyphs & g,
  bool use_color)
{
  std::vector<std::string> lines;

  auto emit_children = [&](
                         auto && self, const std::string & parent, const std::string & prefix,
                         std::unordered_set<std::string> & visiting) -> void {
    auto pit = parent_to_children.find(parent);
    if (pit == parent_to_children.end()) {
      return;
    }
    const auto & kids = pit->second;
    for (std::size_t i = 0; i < kids.size(); ++i) {
      const bool last = (i + 1 == kids.size());
      const std::string & branch = last ? g.branch_end : g.branch_mid;
      const std::string next_prefix = prefix + (last ? "    " : g.vertical_pad);
      const auto & child = kids[i];
      const EdgeTfKind kind = classify_tf_edge(static_edges, dynamic_edges, parent, child);
      if (visiting.count(child) != 0) {
        lines.push_back(
          tf_colored_tree_edge_line(prefix, branch, child, " (cycle)", use_color, kind));
        continue;
      }
      lines.push_back(tf_colored_tree_edge_line(prefix, branch, child, "", use_color, kind));
      visiting.insert(child);
      self(self, child, next_prefix, visiting);
      visiting.erase(child);
    }
  };

  if (!roots_sorted.empty()) {
    for (std::size_t r = 0; r < roots_sorted.size(); ++r) {
      if (r > 0) {
        lines.emplace_back();
      }
      const auto & root = roots_sorted[r];
      lines.push_back(tf_colored_tree_root_line(g.root_prefix + root, use_color));
      std::unordered_set<std::string> visiting;
      visiting.insert(root);
      emit_children(emit_children, root, "", visiting);
    }
  }

  std::string out;
  for (std::size_t i = 0; i < lines.size(); ++i) {
    if (i > 0) {
      out += '\n';
    }
    out += lines[i];
  }
  if (!out.empty()) {
    out += '\n';
  }
  return out;
}

// Union of static and dynamic edge sets into one forest; branch colors reflect edge origin.
std::string format_merged_union_forest(
  const std::set<std::pair<std::string, std::string>> & static_edges,
  const std::set<std::pair<std::string, std::string>> & dynamic_edges, const TreeGlyphs & glyphs,
  bool use_color)
{
  std::set<std::pair<std::string, std::string>> edges;
  edges.insert(static_edges.begin(), static_edges.end());
  edges.insert(dynamic_edges.begin(), dynamic_edges.end());
  if (edges.empty()) {
    return {};
  }

  std::unordered_map<std::string, std::vector<std::string>> parent_to_children;
  std::unordered_set<std::string> child_marked;
  std::unordered_set<std::string> all_nodes;
  for (const auto & pr : edges) {
    all_nodes.insert(pr.first);
    all_nodes.insert(pr.second);
    parent_to_children[pr.first].push_back(pr.second);
    child_marked.insert(pr.second);
  }
  for (auto & kv : parent_to_children) {
    std::sort(kv.second.begin(), kv.second.end());
    kv.second.erase(std::unique(kv.second.begin(), kv.second.end()), kv.second.end());
  }

  std::vector<std::string> roots;
  for (const auto & n : all_nodes) {
    if (child_marked.count(n) == 0) {
      roots.push_back(n);
    }
  }
  std::sort(roots.begin(), roots.end());

  if (roots.empty()) {
    std::vector<std::pair<std::string, std::string>> sorted_edges(edges.begin(), edges.end());
    std::vector<std::string> err_lines;
    err_lines.emplace_back("# Internal error: no tree root after validation. Edges:");
    for (const auto & pr : sorted_edges) {
      err_lines.push_back(fmt::format("  {} -> {}", pr.first, pr.second));
    }
    std::string out;
    for (std::size_t i = 0; i < err_lines.size(); ++i) {
      if (i > 0) {
        out += '\n';
      }
      out += err_lines[i];
    }
    out += '\n';
    return out;
  }

  return format_merged_parent_map_forest(
    parent_to_children, roots, static_edges, dynamic_edges, glyphs, use_color);
}

}  // namespace

// `bagwiz tf` is a command group for TF inspection.
//
// Subcommands
// -----------
//   tree    Union of parent→child edges as one forest; each branch colors the child
//           frame by static-only / dynamic-only / both (validated per-class and combined).
//   static  Resolve the rigid transform from <from> to <to> using only the bag's
//           static TF tree, and print translation / quaternion / RPY (or JSON).
class TfCommand : public Command
{
public:
  [[nodiscard]] std::string_view name() const override { return "tf"; }
  [[nodiscard]] std::string_view description() const override { return "TF inspection"; }

  void configure(CLI::App & app) override
  {
    app.require_subcommand(1);
    configure_tree(app);
    configure_static(app);
  }

  int run() override
  {
    switch (selected_) {
      case Subcommand::kTree:
        return run_tree();
      case Subcommand::kStatic:
        return run_static();
      case Subcommand::kNone:
        BAGWIZ_LOG_ERROR(kLogger, "no subcommand selected");
        return 1;
    }
    return 1;
  }

private:
  enum class Subcommand { kNone, kTree, kStatic };
  Subcommand selected_ = Subcommand::kNone;

  struct TreeArgs
  {
    std::filesystem::path input_path;
  } tree_args_;

  struct StaticArgs
  {
    std::filesystem::path input_path;
    std::string from_frame;
    std::string to_frame;
    bool json = false;
  } static_args_;

  void configure_tree(CLI::App & app)
  {
    auto * sub = app.add_subcommand(
      "tree",
      "Validated union of parent→child edges (one tree; branches colored by static/dynamic/both)");
    sub->add_option("input", tree_args_.input_path, "Bag path (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    sub->callback([this]() { selected_ = Subcommand::kTree; });
  }

  int run_tree()
  {
    const auto & args = tree_args_;

    std::unique_ptr<io::BagReader> reader;
    try {
      reader = io::open_read(args.input_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", args.input_path.c_str(), e.what());
      return 1;
    }

    const auto tf_topics = collect_tf_topics(*reader);
    if (tf_topics.empty()) {
      BAGWIZ_LOG_ERROR(kLogger, "Bag has no tf2_msgs/msg/TFMessage topic; nothing to show.");
      return 1;
    }

    std::vector<std::string> dynamic_topics;
    std::vector<std::string> static_topics;
    dynamic_topics.reserve(tf_topics.size());
    static_topics.reserve(tf_topics.size());
    for (const auto & t : tf_topics) {
      if (t.is_static) {
        static_topics.push_back(t.name);
      } else {
        dynamic_topics.push_back(t.name);
      }
    }

    tf2::BufferCore tf_buffer{std::chrono::hours(24 * 365)};
    std::set<std::pair<std::string, std::string>> static_edges;
    std::set<std::pair<std::string, std::string>> dynamic_edges;
    try {
      load_tf(args.input_path, tf_topics, tf_buffer, &static_edges, &dynamic_edges);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to load TF from the bag: %s", e.what());
      return 1;
    }

    if (tf_buffer.getAllFrameNames().empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Bag has TFMessage topics but no transforms were decoded; nothing to show.");
      return 1;
    }

    if (const auto err = validate_union_edge_set(static_edges, "Static")) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", err->c_str());
      return 1;
    }
    if (const auto err = validate_union_edge_set(dynamic_edges, "Dynamic")) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", err->c_str());
      return 1;
    }

    std::set<std::pair<std::string, std::string>> merged_edges;
    merged_edges.insert(static_edges.begin(), static_edges.end());
    merged_edges.insert(dynamic_edges.begin(), dynamic_edges.end());
    if (!merged_edges.empty()) {
      if (const auto err = validate_union_edge_set(merged_edges, "Combined")) {
        BAGWIZ_LOG_ERROR(kLogger, "%s", err->c_str());
        return 1;
      }
    }

    auto join_topics = [](const std::vector<std::string> & names) -> std::string {
      if (names.empty()) {
        return "(none)";
      }
      std::string out;
      for (std::size_t i = 0; i < names.size(); ++i) {
        if (i > 0) {
          out += ", ";
        }
        out += names[i];
      }
      return out;
    };

    const bool color = stdout_use_color();
    const TreeGlyphs glyphs = make_tree_glyphs();

    fmt::print(
      stdout, "{}",
      tf_format_tf_topics_section("Dynamic TF topics", join_topics(dynamic_topics), color));
    fmt::print(
      stdout, "{}",
      tf_format_tf_topics_section("Static TF topics", join_topics(static_topics), color));
    fmt::print(stdout, "{}", tf_format_tree_legend(color));

    const std::string tree_body =
      merged_edges.empty() ? std::string("(none)\n")
                           : format_merged_union_forest(static_edges, dynamic_edges, glyphs, color);

    fmt::print(stdout, "{}", tf_section_rule("TF tree (static ∪ dynamic edges)", color));
    fmt::print(stdout, "{}", tree_body);

    if (std::fflush(stdout) != 0) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to write TF tree to stdout");
      return 1;
    }
    return 0;
  }

  void configure_static(CLI::App & app)
  {
    auto * sub = app.add_subcommand(
      "static",
      "Rigid transform from <from> to <to> resolved from the bag's static TF tree "
      "(tf2_echo convention)");
    sub->add_option("input", static_args_.input_path, "Bag path (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    sub->add_option("from", static_args_.from_frame, "Source frame id (<from>)")->required();
    sub->add_option("to", static_args_.to_frame, "Target frame id (<to>)")->required();
    sub->add_flag("--json", static_args_.json, "Emit the transform as JSON instead of text");
    sub->callback([this]() { selected_ = Subcommand::kStatic; });
  }

  int run_static()
  {
    const auto & args = static_args_;

    // CLI11 marks <from>/<to> required but accepts the empty string; reject
    // it up front so lookupTransform isn't asked to resolve a blank frame.
    if (args.from_frame.empty() || args.to_frame.empty()) {
      BAGWIZ_LOG_ERROR(kLogger, "Both <from> and <to> frame ids must be non-empty.");
      return 1;
    }

    std::unique_ptr<io::BagReader> reader;
    try {
      reader = io::open_read(args.input_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", args.input_path.c_str(), e.what());
      return 1;
    }

    // This subcommand resolves transforms purely from the static tree, so
    // dynamic /tf topics are intentionally ignored: only *tf_static topics
    // are fed into the buffer (as static entries).
    std::vector<TfTopic> static_topics;
    for (const auto & t : collect_tf_topics(*reader)) {
      if (t.is_static) {
        static_topics.push_back(t);
      }
    }
    if (static_topics.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "Bag has no static tf2_msgs/msg/TFMessage topic (e.g. /tf_static); nothing to resolve.");
      return 1;
    }

    tf2::BufferCore tf_buffer{std::chrono::hours(24 * 365)};
    try {
      load_tf(args.input_path, static_topics, tf_buffer);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to load static TF from the bag: %s", e.what());
      return 1;
    }

    geometry_msgs::msg::TransformStamped tf;
    try {
      // from→to: lookupTransform(target=<to>, source=<from>). Translation is
      // then <from>'s origin expressed in <to>. Matches
      // `ros2 run tf2_ros tf2_echo <from> <to>`. Static entries ignore the
      // query time, so TimePointZero is used.
      tf = tf_buffer.lookupTransform(args.to_frame, args.from_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Could not resolve static transform %s -> %s: %s", args.from_frame.c_str(),
        args.to_frame.c_str(), e.what());

      std::vector<std::string> frames = tf_buffer.getAllFrameNames();
      std::sort(frames.begin(), frames.end());
      std::string csv;
      for (std::size_t i = 0; i < frames.size(); ++i) {
        if (i > 0) {
          csv += ", ";
        }
        csv += frames[i];
      }
      BAGWIZ_LOG_ERROR(
        kLogger, "Available static frames: %s", csv.empty() ? "(none)" : csv.c_str());
      return 1;
    }

    const std::string out = args.json
                              ? core::format_transform_json(tf, args.from_frame, args.to_frame)
                              : core::format_transform_human(tf, args.from_frame, args.to_frame);
    // The human form ends with a newline; the JSON form does not, so add one.
    fmt::print(stdout, "{}{}", out, args.json ? "\n" : "");

    if (std::fflush(stdout) != 0) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to write transform to stdout");
      return 1;
    }
    return 0;
  }
};

BAGWIZ_REGISTER_COMMAND(TfCommand)

}  // namespace bagwiz::commands
