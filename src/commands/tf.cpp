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
#include "bagwiz/core/output_path.hpp"
#include "bagwiz/core/tf_static_injector.hpp"
#include "bagwiz/core/tf_value_extract.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <rang.hpp>

#include <fmt/core.h>
#include <tf2/buffer_core.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <span>
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
//   tree           Union of parent→child edges as one forest; each branch colors the child
//                  frame by static-only / dynamic-only / both (validated per-class and combined).
//   inject-static  Copy a destination bag with /tf_static injected from a source bag.
class TfCommand : public Command
{
public:
  [[nodiscard]] std::string_view name() const override { return "tf"; }
  [[nodiscard]] std::string_view description() const override { return "TF inspection"; }

  void configure(CLI::App & app) override
  {
    app.require_subcommand(1);
    configure_tree(app);
    configure_inject_static(app);
  }

  int run() override
  {
    switch (selected_) {
      case Subcommand::kTree:
        return run_tree();
      case Subcommand::kInjectStatic:
        return run_inject_static();
      case Subcommand::kNone:
        BAGWIZ_LOG_ERROR(kLogger, "no subcommand selected");
        return 1;
    }
    return 1;
  }

private:
  enum class Subcommand { kNone, kTree, kInjectStatic };
  Subcommand selected_ = Subcommand::kNone;

  struct TreeArgs
  {
    std::filesystem::path input_path;
  } tree_args_;

  struct InjectStaticArgs
  {
    std::filesystem::path src_path;
    std::filesystem::path dst_path;
    std::filesystem::path output_path;
    bool force = false;
    bool overwrite = false;  // replace any pre-existing output_path
  } inject_static_args_;

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

  void configure_inject_static(CLI::App & app)
  {
    auto * sub = app.add_subcommand(
      "inject-static",
      "Copy <dst> to <output> with /tf_static from <src> injected as a single message "
      "at <dst>'s start time. Source header.stamp values are rewritten to that same time.");
    sub->add_option("src", inject_static_args_.src_path, "Source bag (provides /tf_static)")
      ->required()
      ->check(CLI::ExistingPath);
    sub
      ->add_option(
        "dst", inject_static_args_.dst_path,
        "Destination bag (copied unchanged except for static TF)")
      ->required()
      ->check(CLI::ExistingPath);
    sub
      ->add_option(
        "-o,--output", inject_static_args_.output_path, "Output bag path (file or directory)")
      ->required();
    sub
      ->add_flag(
        "--force", inject_static_args_.force,
        "Overwrite per-topic when <dst> already has messages on a *tf_static topic.")
      ->default_val(false);
    sub->add_flag(
      "--overwrite", inject_static_args_.overwrite,
      "Replace -o/--output if it already exists. Without this flag, an "
      "existing output path stops the run.");
    sub->callback([this]() { selected_ = Subcommand::kInjectStatic; });
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

  int run_inject_static()
  {
    const auto & args = inject_static_args_;

    if (const auto r = core::prepare_output_path(args.output_path, args.overwrite); !r.ok) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", r.error.c_str());
      return 1;
    }

    // Step 1: Scan <src> and collect deduped TransformStamped[] per
    // *tf_static topic.
    core::CollectedTfStatic collected;
    try {
      collected = core::collect_tf_static_from_bag(args.src_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Failed to collect /tf_static from %s: %s", args.src_path.c_str(), e.what());
      return 1;
    }
    if (collected.by_topic.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Source bag %s has no tf2_msgs/msg/TFMessage *tf_static topic; nothing to inject.",
        args.src_path.c_str());
      return 1;
    }

    // Step 2: Open <dst>, learn its topic list, schemas, start_ns, and
    // per-topic counts.
    std::unique_ptr<io::BagReader> dst_reader;
    try {
      dst_reader = io::open_read(args.dst_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", args.dst_path.c_str(), e.what());
      return 1;
    }
    dst_reader->populate_schemas();

    io::BagReader::Stats dst_stats;
    try {
      dst_stats = dst_reader->compute_stats();
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Failed to compute stats on %s: %s", args.dst_path.c_str(), e.what());
      return 1;
    }
    if (dst_stats.start_ns <= 0) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "Destination bag %s has non-positive start time (%" PRId64
        " ns); cannot derive an injection timestamp.",
        args.dst_path.c_str(), static_cast<std::int64_t>(dst_stats.start_ns));
      return 1;
    }
    const std::int64_t start_ns = dst_stats.start_ns;

    // Snapshot <dst>'s topic list. The reader's span is invalidated by
    // close/reopen, and we will close this reader before opening the
    // copy pass — so make owning copies up-front.
    std::vector<io::TopicInfo> dst_topics(dst_reader->topics().begin(), dst_reader->topics().end());
    const auto dst_count_for = [&](const std::string & name) -> std::int64_t {
      auto it = dst_stats.per_topic.find(name);
      return it == dst_stats.per_topic.end() ? 0 : it->second;
    };
    const auto find_dst_topic = [&](const std::string & name) -> const io::TopicInfo * {
      for (const auto & t : dst_topics) {
        if (t.name == name) {
          return &t;
        }
      }
      return nullptr;
    };

    // Step 3: Enforce the conflict policy *before* opening the writer.
    // The decision per <src> static topic:
    //   * not present in <dst>           -> declare-new (schema from <src>)
    //   * present, count == 0           -> declare-keep (schema from <dst>)
    //   * present, count > 0, !force    -> conflict; report and abort
    //   * present, count > 0, force     -> drop-and-replace (suppress <dst>'s
    //                                       messages on this topic during copy)
    std::unordered_map<std::string, bool> suppress_topic_on_copy;  // topic -> drop existing
    std::unordered_map<std::string, io::TopicInfo>
      declare_for_new;  // topic -> TopicInfo to declare (only when missing from <dst>)
    bool had_conflict = false;
    for (const auto & [topic, _transforms] : collected.by_topic) {
      const auto * existing = find_dst_topic(topic);
      if (existing == nullptr) {
        // New topic in output; carry over the source's schema info.
        declare_for_new.emplace(topic, collected.source_topic_info.at(topic));
        continue;
      }
      if (existing->type != kTfMessageType) {
        BAGWIZ_LOG_ERROR(
          kLogger,
          "Topic '%s' exists in <dst> with type '%s', but <src> declares it as %s; refusing to "
          "inject incompatible payloads.",
          topic.c_str(), existing->type.c_str(), kTfMessageType);
        return 1;
      }
      const std::int64_t cnt = dst_count_for(topic);
      if (cnt == 0) {
        continue;
      }
      if (!args.force) {
        BAGWIZ_LOG_ERROR(
          kLogger,
          "Destination already has %" PRId64
          " message(s) on '%s'. Pass --force to overwrite (existing messages will be dropped).",
          cnt, topic.c_str());
        had_conflict = true;
        continue;
      }
      suppress_topic_on_copy[topic] = true;
      BAGWIZ_LOG_WARN(
        kLogger, "--force: dropping %" PRId64 " message(s) on '%s' in output.", cnt, topic.c_str());
    }
    if (had_conflict) {
      return 1;
    }

    // Step 4: Build the injected payload per topic. Rewrite every
    // TransformStamped.header.stamp to <dst>.start_ns so the static
    // transforms appear consistent with the destination's timeline.
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<std::int32_t>(start_ns / 1'000'000'000LL);
    stamp.nanosec = static_cast<std::uint32_t>(start_ns % 1'000'000'000LL);

    std::map<std::string, std::vector<std::byte>> payload_by_topic;
    for (auto & [topic, transforms] : collected.by_topic) {
      if (transforms.empty()) {
        // No edges to inject; skip silently so we do not emit an empty
        // /tf_static message on a topic with no actual content.
        continue;
      }
      for (auto & t : transforms) {
        t.header.stamp = stamp;
      }
      try {
        payload_by_topic[topic] = core::serialize_tf_message(transforms);
      } catch (const std::exception & e) {
        BAGWIZ_LOG_ERROR(
          kLogger, "Failed to serialize merged TFMessage for '%s': %s", topic.c_str(), e.what());
        return 1;
      }
    }
    if (payload_by_topic.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "Source bag has *tf_static topics but no valid TransformStamped entries to inject.");
      return 1;
    }

    // Step 5: Open the writer, declare topics, stream-copy <dst>, then
    // emit the injected payloads.
    //
    // Storage choice: inherit <dst>'s format whenever the user's -o
    // doesn't name a single-file format via its extension. Without
    // this, a sqlite3 <dst> (single-file or directory) with a -o like
    // "out_dir" or "out" would silently produce an mcap directory
    // because io::open_write's Auto resolution falls through to
    // Directory + Mcap when the extension isn't .mcap / .db3.
    io::CreateOptions copts = io::create_options_inheriting_format(args.dst_path, args.output_path);
    copts.mcap_compression = "none";

    std::unique_ptr<io::BagWriter> writer;
    try {
      writer = io::open_write(args.output_path, copts);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open output %s: %s", args.output_path.c_str(), e.what());
      return 1;
    }

    for (const auto & t : dst_topics) {
      try {
        writer->declare_topic(t);
      } catch (const std::exception & e) {
        BAGWIZ_LOG_ERROR(kLogger, "declare_topic failed for '%s': %s", t.name.c_str(), e.what());
        return 1;
      }
    }
    for (const auto & [topic, info] : declare_for_new) {
      try {
        writer->declare_topic(info);
      } catch (const std::exception & e) {
        BAGWIZ_LOG_ERROR(
          kLogger, "declare_topic failed for new topic '%s': %s", topic.c_str(), e.what());
        return 1;
      }
    }

    // Stream copy. Suppress messages on topics where --force will replace
    // the existing payload with the injected one.
    io::RawMessage raw;
    std::uint64_t copied = 0;
    std::uint64_t suppressed = 0;
    try {
      while (dst_reader->next(raw)) {
        if (suppress_topic_on_copy.count(raw.topic->name) != 0) {
          ++suppressed;
          continue;
        }
        writer->write(raw.topic->name, raw.timestamp_ns, raw.payload);
        ++copied;
      }
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Copy from <dst> failed: %s", e.what());
      return 1;
    }

    // Step 6: Inject one merged TFMessage per topic at <dst>.start_ns.
    std::uint64_t injected = 0;
    for (const auto & [topic, payload] : payload_by_topic) {
      try {
        writer->write(topic, start_ns, std::span<const std::byte>(payload.data(), payload.size()));
        ++injected;
      } catch (const std::exception & e) {
        BAGWIZ_LOG_ERROR(
          kLogger, "Failed to write injected /tf_static on '%s': %s", topic.c_str(), e.what());
        return 1;
      }
    }

    try {
      writer->close();
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Writer close() failed: %s", e.what());
      return 1;
    }

    BAGWIZ_LOG_INFO(
      kLogger,
      "tf inject-static: copied %" PRIu64 " message(s), suppressed %" PRIu64
      " on --force, injected %" PRIu64 " merged TFMessage(s) at start_ns=%" PRId64 ".",
      copied, suppressed, injected, start_ns);
    return 0;
  }
};

BAGWIZ_REGISTER_COMMAND(TfCommand)

}  // namespace bagwiz::commands
