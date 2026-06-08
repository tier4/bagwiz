// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/commands/tf_static_cp.hpp"
#include "bagwiz/commands/tf_walk.hpp"
#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/logging.hpp"
#include "bagwiz/core/tf_chain.hpp"
#include "bagwiz/core/tf_merge_check.hpp"
#include "bagwiz/core/tf_transform_format.hpp"
#include "bagwiz/core/tf_value_extract.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <rang.hpp>
#include <tf2/buffer_core.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <fmt/core.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <ostream>
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

// Replay the given TF topics once: when `buffer` is non-null, feed every
// contained TransformStamped into it with the correct static/dynamic flag;
// when `edges_by_topic_out` is non-null, collect the distinct parent→child
// edges into it keyed by the source topic name. `tf static calc` needs the
// buffer (to resolve transforms) but not the edges; `tf tree` needs the
// per-topic edges but not the buffer.
//
// When `conflict_checker` is non-null, every edge is run through it and the
// first cross-topic conflict (multi-parent, or static/dynamic mix) throws —
// used by `tf static calc` so several `*tf_static` topics are merged but a
// contradiction aborts instead of silently last-winning. `tf tree` leaves it
// null and runs its own forest validation downstream.
//
// Decoding goes through the unified open_decoder() path so for MCAP
// inputs the schema-driven backend handles the work and tf2_msgs no
// longer needs to be on AMENT_PREFIX_PATH at runtime; only its
// header-only struct definition is required at build time (via
// extract_tf_message → geometry_msgs::msg::TransformStamped).
void load_tf(
  const std::filesystem::path & bag_path, const std::vector<TfTopic> & tf_topics,
  tf2::BufferCore * buffer = nullptr,
  std::map<std::string, std::set<std::pair<std::string, std::string>>> * edges_by_topic_out =
    nullptr,
  core::TfMergeConflictChecker * conflict_checker = nullptr)
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
      if (conflict_checker != nullptr && !t.header.frame_id.empty() && !t.child_frame_id.empty()) {
        if (
          const auto conflict = conflict_checker->add(
            t.header.frame_id, t.child_frame_id, raw.topic->name, is_static)) {
          throw std::runtime_error("TF merge conflict: " + *conflict);
        }
      }
      if (
        edges_by_topic_out != nullptr && !t.header.frame_id.empty() && !t.child_frame_id.empty()) {
        (*edges_by_topic_out)[raw.topic->name].insert(
          std::make_pair(t.header.frame_id, t.child_frame_id));
      }
      if (buffer != nullptr) {
        buffer->setTransform(t, "bagwiz", is_static);
      }
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
  std::string row = std::string("\u2550\u2550\u2550 ") + label + " \u2550\u2550\u2550\n";
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

// Category of an edge for coloring: a child is reached either via a static
// (*tf_static) topic or a dynamic one. The two-way classification is what
// `tf tree` colors by; -1 means "do not classify" (a single-category tree is
// rendered plain).
enum class EdgeCategory { kNone = -1, kDynamic = 0, kStatic = 1 };

// Foreground color per edge category for the merged static+dynamic view.
void apply_category_fg(std::ostream & os, EdgeCategory category)
{
  if (category == EdgeCategory::kStatic) {
    os << rang::fgB::yellow;
  } else {
    os << rang::fgB::cyan;
  }
}

// Branch line for a child frame: dim branch glyphs then the child name. When
// `category` is kStatic / kDynamic (a mixed static+dynamic tree) the name is
// colored by that category and a " [S]" / " [D]" tag is appended so the class
// stays identifiable without color; when it is kNone (single-category tree) the
// name is plain and no tag is shown. `suffix` is e.g. " (cycle)".
std::string tf_tree_edge_line(
  const std::string & prefix, const std::string & branch, const std::string & child,
  const std::string & suffix, bool use_color, EdgeCategory category)
{
  const std::string tag = category == EdgeCategory::kStatic    ? " [S]"
                          : category == EdgeCategory::kDynamic ? " [D]"
                                                               : std::string{};
  if (!use_color) {
    return prefix + branch + child + tag + suffix;
  }
  rang::setControlMode(rang::control::Force);
  std::ostringstream oss;
  oss << rang::fg::gray << prefix << branch << rang::style::reset;
  if (category != EdgeCategory::kNone) {
    apply_category_fg(oss, category);
    oss << child << rang::style::reset;
    oss << rang::fg::gray << tag << rang::style::reset;
  } else {
    oss << child;
  }
  if (!suffix.empty()) {
    oss << rang::fg::gray << suffix << rang::style::reset;
  }
  std::string out = oss.str();
  rang::setControlMode(rang::control::Auto);
  return out;
}

// Validates an edge set as a forest (unique parent per child, no A→B together
// with B→A, no self edges, no cycles). `context` describes the source for the
// error messages, e.g. "for topic '/tf'" or "for the merged topics".
std::optional<std::string> validate_union_edge_set(
  const std::set<std::pair<std::string, std::string>> & edges, const std::string & context)
{
  for (const auto & pr : edges) {
    if (pr.first == pr.second) {
      return fmt::format(
        "TF tree {}: self-referential edge '{}' -> '{}' is not allowed.", context, pr.first,
        pr.second);
    }
  }

  for (const auto & pr : edges) {
    if (pr.first >= pr.second) {
      continue;
    }
    if (edges.count({pr.second, pr.first}) != 0) {
      return fmt::format(
        "TF tree {}: opposite edges '{}' -> '{}' and '{}' -> '{}' cannot both appear.", context,
        pr.first, pr.second, pr.second, pr.first);
    }
  }

  std::unordered_map<std::string, std::string> child_to_parent;
  for (const auto & pr : edges) {
    auto ins = child_to_parent.emplace(pr.second, pr.first);
    if (!ins.second && ins.first->second != pr.first) {
      return fmt::format(
        "TF tree {}: child frame '{}' has parent '{}' in one transform and '{}' in another.",
        context, pr.second, ins.first->second, pr.first);
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
          "TF tree {}: edges contain a directed cycle (revisited frame '{}').", context, cur);
      }
    }
  }

  return std::nullopt;
}

// Render a forest from an adjacency map (parent → sorted children) and sorted
// roots: bold root line, dim branch glyphs, and " (cycle)" on a frame already
// on the current path (rather than recursing). When `show_category` is set,
// each child is colored and " [S]" / " [D]"-tagged by its edge's category
// (looked up in `edge_to_category`); otherwise child names are plain.
std::string format_parent_map_forest(
  const std::unordered_map<std::string, std::vector<std::string>> & parent_to_children,
  const std::vector<std::string> & roots_sorted, const TreeGlyphs & g, bool use_color,
  const std::map<std::pair<std::string, std::string>, EdgeCategory> & edge_to_category,
  bool show_category)
{
  std::vector<std::string> lines;

  auto category_of = [&](const std::string & parent, const std::string & child) -> EdgeCategory {
    if (!show_category) {
      return EdgeCategory::kNone;
    }
    const auto eit = edge_to_category.find({parent, child});
    return eit != edge_to_category.end() ? eit->second : EdgeCategory::kNone;
  };

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
      const EdgeCategory category = category_of(parent, child);
      if (visiting.count(child) != 0) {
        lines.push_back(tf_tree_edge_line(prefix, branch, child, " (cycle)", use_color, category));
        continue;
      }
      lines.push_back(tf_tree_edge_line(prefix, branch, child, "", use_color, category));
      visiting.insert(child);
      self(self, child, next_prefix, visiting);
      visiting.erase(child);
    }
  };

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

// Build the adjacency map + sorted roots from the merged edge set and render the
// forest. `validate_union_edge_set` runs before this, so a non-empty edge set
// always has at least one root. `edge_to_category` / `show_category` are
// forwarded to the renderer for static/dynamic coloring (see
// format_parent_map_forest).
std::string format_tree_forest(
  const std::set<std::pair<std::string, std::string>> & edges, const TreeGlyphs & glyphs,
  bool use_color,
  const std::map<std::pair<std::string, std::string>, EdgeCategory> & edge_to_category,
  bool show_category)
{
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

  return format_parent_map_forest(
    parent_to_children, roots, glyphs, use_color, edge_to_category, show_category);
}

// Comma-joined list for human-readable messages; "(none)" when empty.
std::string join_csv(const std::vector<std::string> & names)
{
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
}

// Sorted, comma-separated list of every frame id known to `buffer`, or "(none)".
std::string sorted_frames_csv(const tf2::BufferCore & buffer)
{
  std::vector<std::string> frames = buffer.getAllFrameNames();
  std::sort(frames.begin(), frames.end());
  return join_csv(frames);
}

// Reserved <topics> selectors. "static" expands to every *tf_static topic,
// "dynamic" to every non-static TF topic. ROS topic names start with '/', so
// these bare words never collide with a real topic name.
constexpr const char * kSelectStatic = "static";
constexpr const char * kSelectDynamic = "dynamic";

// Expand the requested <topics> tokens into concrete TfTopics, deduplicated by
// name (first appearance wins). "static" / "dynamic" expand to all static /
// dynamic TF topics in the bag (and compose with literal topic names); any
// other token must name a TFMessage topic that exists. On an unknown literal,
// logs the offending names + the bag's available TF topics and returns false.
bool select_tree_topics(
  const std::vector<std::string> & requested, const std::vector<TfTopic> & tf_topics,
  std::vector<TfTopic> & selected_out)
{
  std::unordered_set<std::string> added;
  auto add_topic = [&](const TfTopic & t) {
    if (added.insert(t.name).second) {
      selected_out.push_back(t);
    }
  };

  std::vector<std::string> unknown;
  for (const auto & token : requested) {
    if (token == kSelectStatic) {
      for (const auto & t : tf_topics) {
        if (t.is_static) {
          add_topic(t);
        }
      }
    } else if (token == kSelectDynamic) {
      for (const auto & t : tf_topics) {
        if (!t.is_static) {
          add_topic(t);
        }
      }
    } else {
      const TfTopic * match = nullptr;
      for (const auto & t : tf_topics) {
        if (t.name == token) {
          match = &t;
          break;
        }
      }
      if (match != nullptr) {
        add_topic(*match);
      } else {
        unknown.push_back(token);
      }
    }
  }

  if (unknown.empty()) {
    return true;
  }

  std::vector<std::string> available;
  available.reserve(tf_topics.size());
  for (const auto & t : tf_topics) {
    available.push_back(t.name);
  }
  std::sort(available.begin(), available.end());
  BAGWIZ_LOG_ERROR(
    kLogger,
    "Not a tf2_msgs/msg/TFMessage topic in the bag (nor the 'static' / 'dynamic' selector): %s",
    join_csv(unknown).c_str());
  BAGWIZ_LOG_ERROR(kLogger, "Available TF topics: %s", join_csv(available).c_str());
  return false;
}

// Legend for the mixed static+dynamic view: a "═══ Legend ═══" rule then a
// colored "[D] dynamic" and "[S] static" line, plus a trailing blank line that
// separates it from the tree block.
std::string format_category_legend(bool use_color)
{
  std::string out = tf_section_rule("Legend", use_color);
  if (!use_color) {
    out += "  [D] dynamic\n";
    out += "  [S] static\n";
    out += "\n";
    return out;
  }
  rang::setControlMode(rang::control::Force);
  std::ostringstream oss;
  oss << "  ";
  apply_category_fg(oss, EdgeCategory::kDynamic);
  oss << "[D] dynamic" << rang::style::reset << '\n';
  oss << "  ";
  apply_category_fg(oss, EdgeCategory::kStatic);
  oss << "[S] static" << rang::style::reset << '\n';
  out += oss.str();
  rang::setControlMode(rang::control::Auto);
  out += "\n";
  return out;
}

}  // namespace

// `bagwiz tf` is a command group for TF inspection.
//
// Subcommands
// -----------
//   tree         Merge one or more tf2_msgs/msg/TFMessage <topics> into one
//                validated TF frame tree; on a TTY edges are colored by static
//                vs dynamic. The 'static' / 'dynamic' selectors pick all static
//                / dynamic TF topics. A merge conflict (same child via different
//                parents, or both static and dynamic) aborts with an error.
//   static calc  Resolve the rigid transform from <from> to <to> using only the
//                bag's static TF tree, and print translation / quaternion / RPY
//                (or JSON). 'static' is a command group; 'calc' is its action.
//   static cp    Copy every static TF topic from <src> into <dst> (in place, or
//                to a new bag via -o), preserving topic names and stamping each
//                at <dst>'s start time.
//   walk         Merge every TF topic into one buffer and step interactively
//                through the times the merged TF changed, showing <from> -> <to>
//                at each.
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
    configure_walk(app);
  }

  int run() override
  {
    switch (selected_) {
      case Subcommand::kTree:
        return run_tree();
      case Subcommand::kStaticCalc:
        return run_static_calc();
      case Subcommand::kStaticCp:
        return run_static_cp();
      case Subcommand::kWalk:
        return run_walk();
      case Subcommand::kNone:
        BAGWIZ_LOG_ERROR(kLogger, "no subcommand selected");
        return 1;
    }
    return 1;
  }

private:
  enum class Subcommand { kNone, kTree, kStaticCalc, kStaticCp, kWalk };
  Subcommand selected_ = Subcommand::kNone;

  struct TreeArgs
  {
    std::filesystem::path input_path;
    std::vector<std::string> topics;
  } tree_args_;

  struct StaticArgs
  {
    std::filesystem::path input_path;
    std::string from_frame;
    std::string to_frame;
    bool json = false;
  } static_args_;

  struct StaticCpArgs
  {
    std::filesystem::path src_path;
    std::filesystem::path dst_path;
    std::optional<std::filesystem::path> output_path;
    bool overwrite = false;
  } static_cp_args_;

  struct WalkArgs
  {
    std::filesystem::path input_path;
    std::string from_frame;
    std::string to_frame;
  } walk_args_;

  void configure_tree(CLI::App & app)
  {
    auto * sub = app.add_subcommand(
      "tree", "Merge one or more tf2_msgs/msg/TFMessage topics into one TF frame tree");
    sub->add_option("input", tree_args_.input_path, "Bag path (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    sub->add_option(
      "topics", tree_args_.topics,
      "tf2_msgs/msg/TFMessage topic(s) to merge and render; defaults to all TF topics in the bag "
      "when omitted. The selectors 'static' and 'dynamic' expand to all static (*tf_static) / "
      "dynamic TF topics respectively and may be combined with literal topic names. In the merged "
      "tree, edges are colored by static vs dynamic.");
    sub->callback([this]() { selected_ = Subcommand::kTree; });
  }

  int run_tree()
  {
    const auto & args = tree_args_;

    // Dedup the requested topics, preserving first-occurrence order, and reject
    // empty names (CLI11 accepts empty strings even for an explicit list). An
    // empty list is allowed and means "merge every TF topic in the bag".
    std::vector<std::string> requested;
    {
      std::unordered_set<std::string> seen;
      for (const auto & t : args.topics) {
        if (t.empty()) {
          BAGWIZ_LOG_ERROR(kLogger, "Every <topic> argument must be a non-empty topic name.");
          return 1;
        }
        if (seen.insert(t).second) {
          requested.push_back(t);
        }
      }
    }

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

    // With no explicit <topics>, default to every TF topic in the bag.
    // Otherwise expand the tokens: "static" / "dynamic" select all static /
    // dynamic TF topics (and compose with literal names); any other token must
    // be a TFMessage topic that exists. select_tree_topics logs the unknown
    // names + available TF topics on failure.
    std::vector<TfTopic> selected;
    if (requested.empty()) {
      selected = tf_topics;
    } else if (!select_tree_topics(requested, tf_topics, selected)) {
      return 1;
    }
    if (selected.empty()) {
      std::vector<std::string> available;
      available.reserve(tf_topics.size());
      for (const auto & t : tf_topics) {
        available.push_back(t.name);
      }
      std::sort(available.begin(), available.end());
      BAGWIZ_LOG_ERROR(
        kLogger, "No TF topics matched <topics> %s. Available TF topics: %s",
        join_csv(requested).c_str(), join_csv(available).c_str());
      return 1;
    }

    // Replay the selected topics, grouping edges by topic so each merged edge
    // can be tagged static or dynamic by its source. The conflict checker
    // rejects contradictory merges (a child given two parents by different
    // topics, or a child declared both static and dynamic) — consistent with
    // `traj dump` and `tf static calc`. No tf2 buffer is needed (unlike
    // `tf static calc`).
    std::map<std::string, std::set<std::pair<std::string, std::string>>> edges_by_topic;
    core::TfMergeConflictChecker conflict_checker;
    try {
      load_tf(args.input_path, selected, /*buffer=*/nullptr, &edges_by_topic, &conflict_checker);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to load TF from the bag: %s", e.what());
      return 1;
    }

    // Merge the edges and tag each with its source category. A frame cannot be
    // both static and dynamic (the conflict checker would have rejected it), so
    // every merged edge has a single well-defined category.
    std::unordered_map<std::string, bool> static_by_name;
    for (const auto & t : selected) {
      static_by_name[t.name] = t.is_static;
    }
    std::set<std::pair<std::string, std::string>> merged;
    std::map<std::pair<std::string, std::string>, EdgeCategory> edge_to_category;
    bool has_static = false;
    bool has_dynamic = false;
    for (const auto & [topic, edges] : edges_by_topic) {
      const bool is_static = static_by_name.at(topic);
      for (const auto & edge : edges) {
        merged.insert(edge);
        edge_to_category[edge] = is_static ? EdgeCategory::kStatic : EdgeCategory::kDynamic;
        if (is_static) {
          has_static = true;
        } else {
          has_dynamic = true;
        }
      }
    }
    if (merged.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Topic(s) %s carry no decodable transforms; nothing to show.",
        join_csv(requested).c_str());
      return 1;
    }

    // The merged set must also form a valid forest (no cycles, opposite edges,
    // self edges, or multi-parent); this complements the conflict checker.
    if (const auto err = validate_union_edge_set(merged, "for the merged topics")) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", err->c_str());
      return 1;
    }

    // Color/tag by static vs dynamic only when both are present; a single
    // category renders plain with the category named in the header.
    const bool show_category = has_static && has_dynamic;

    const bool color = stdout_use_color();
    const TreeGlyphs glyphs = make_tree_glyphs();

    if (show_category) {
      fmt::print(stdout, "{}", format_category_legend(color));
      fmt::print(stdout, "{}", tf_section_rule("TF tree", color));
    } else {
      const std::string header_label =
        std::string("TF tree (") + (has_static ? "static" : "dynamic") + ")";
      fmt::print(stdout, "{}", tf_section_rule(header_label.c_str(), color));
    }
    fmt::print(
      stdout, "{}", format_tree_forest(merged, glyphs, color, edge_to_category, show_category));

    if (std::fflush(stdout) != 0) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to write TF tree to stdout");
      return 1;
    }
    return 0;
  }

  // `static` is a command group, not a leaf: its actions live under
  // `static calc` (resolve a transform) and `static cp` (copy static TF between
  // bags). Modeling it as a group (require_subcommand(1)) keeps room for further
  // static-tree queries and keeps `bagwiz tf static` from doing anything without
  // an explicit verb.
  void configure_static(CLI::App & app)
  {
    auto * group = app.add_subcommand("static", "Static TF tree queries");
    group->require_subcommand(1);
    configure_static_calc(*group);
    configure_static_cp(*group);
  }

  void configure_static_calc(CLI::App & group)
  {
    auto * sub = group.add_subcommand(
      "calc",
      "Rigid transform from <from> to <to> resolved from the bag's static TF tree "
      "(tf2_echo convention)");
    sub->add_option("input", static_args_.input_path, "Bag path (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    sub->add_option("from", static_args_.from_frame, "Source frame id (<from>)")->required();
    sub->add_option("to", static_args_.to_frame, "Target frame id (<to>)")->required();
    sub->add_flag("--json", static_args_.json, "Emit the transform as JSON instead of text");
    sub->callback([this]() { selected_ = Subcommand::kStaticCalc; });
  }

  void configure_static_cp(CLI::App & group)
  {
    auto * sub = group.add_subcommand(
      "cp",
      "Copy every static TF topic from <src> into <dst>, preserving topic names. Each copied "
      "topic is written as one TFMessage stamped at <dst>'s start time.");
    sub->add_option("src", static_cp_args_.src_path, "Source bag to copy static TF from")
      ->required()
      ->check(CLI::ExistingPath);
    sub->add_option("dst", static_cp_args_.dst_path, "Destination bag to copy static TF into")
      ->required()
      ->check(CLI::ExistingPath);
    sub->add_option(
      "-o,--output", static_cp_args_.output_path,
      "Write the result to this new bag instead of rewriting <dst> in place.");
    sub->add_flag(
      "--overwrite", static_cp_args_.overwrite,
      "Permit clobbering: replace an existing -o/--output path, and replace any static topic in "
      "<dst> whose name collides with one being copied. Without it, either conflict aborts.");
    sub->callback([this]() { selected_ = Subcommand::kStaticCp; });
  }

  int run_static_calc()
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

    // Merge every static topic into one buffer, but refuse to silently
    // last-wins on a contradiction: the checker aborts when two static topics
    // give the same child different parents.
    tf2::BufferCore tf_buffer{std::chrono::hours(24 * 365)};
    core::TfMergeConflictChecker conflict_checker;
    try {
      load_tf(
        args.input_path, static_topics, &tf_buffer, /*edges_by_topic_out=*/nullptr,
        &conflict_checker);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to load static TF from the bag: %s", e.what());
      return 1;
    }

    // tf2's lookupTransform returns an identity transform when target == source
    // WITHOUT checking the frame exists, so `tf static calc <f> <f>` for an
    // absent frame would otherwise print a bogus identity transform. Reject
    // either endpoint up front when the static tree does not contain it.
    const std::vector<std::string> missing =
      core::missing_frames(tf_buffer, args.from_frame, args.to_frame);
    if (!missing.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Frame(s) not present in the bag's static TF tree: %s", join_csv(missing).c_str());
      BAGWIZ_LOG_ERROR(
        kLogger, "Available static frames: %s", sorted_frames_csv(tf_buffer).c_str());
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
      // Both endpoints exist (validated above) but are not connected in the
      // static tree at TimePointZero.
      BAGWIZ_LOG_ERROR(
        kLogger, "Could not resolve static transform %s -> %s: %s", args.from_frame.c_str(),
        args.to_frame.c_str(), e.what());
      BAGWIZ_LOG_ERROR(
        kLogger, "Available static frames: %s", sorted_frames_csv(tf_buffer).c_str());
      return 1;
    }

    std::string out;
    if (args.json) {
      out = core::format_transform_json(tf, args.from_frame, args.to_frame);
    } else {
      // Resolve the full frame chain so the direction line lists every
      // intermediate frame, not just the endpoints. Static entries ignore the
      // query time (TimePointZero). The lookupTransform above already
      // succeeded, so a chain exists; fall back to the bare endpoints if
      // resolve_chain cannot reconstruct it.
      std::vector<std::string> chain =
        core::resolve_chain(tf_buffer, args.from_frame, args.to_frame, tf2::TimePointZero);
      if (chain.empty()) {
        chain = {args.from_frame, args.to_frame};
      }
      out = core::format_transform_human(tf, chain, "  (static)");
    }
    // The human form ends with a newline; the JSON form does not, so add one.
    fmt::print(stdout, "{}{}", out, args.json ? "\n" : "");

    if (std::fflush(stdout) != 0) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to write transform to stdout");
      return 1;
    }
    return 0;
  }

  int run_static_cp()
  {
    const auto & args = static_cp_args_;
    return run_tf_static_cp(args.src_path, args.dst_path, args.output_path, args.overwrite);
  }

  void configure_walk(CLI::App & app)
  {
    auto * sub = app.add_subcommand(
      "walk",
      "Step interactively through the times the merged TF changed, showing the "
      "<from> -> <to> transform at each (merges every TF topic in the bag)");
    sub->add_option("input", walk_args_.input_path, "Bag path (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    sub->add_option("from", walk_args_.from_frame, "Source frame id (<from>)")->required();
    sub->add_option("to", walk_args_.to_frame, "Target frame id (<to>)")->required();
    sub->callback([this]() { selected_ = Subcommand::kWalk; });
  }

  int run_walk()
  {
    const auto & args = walk_args_;

    // CLI11 marks <from>/<to> required but accepts the empty string; reject it
    // up front so lookupTransform isn't asked to resolve a blank frame.
    if (args.from_frame.empty() || args.to_frame.empty()) {
      BAGWIZ_LOG_ERROR(kLogger, "Both <from> and <to> frame ids must be non-empty.");
      return 1;
    }

    return run_tf_walk(args.input_path, args.from_frame, args.to_frame);
  }
};

BAGWIZ_REGISTER_COMMAND(TfCommand)

}  // namespace bagwiz::commands
