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
// edges into it keyed by the source topic name. `tf static` needs the buffer
// (to resolve transforms) but not the edges; `tf tree` needs the per-topic
// edges but not the buffer.
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
    nullptr)
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

// Per-topic foreground color for the merged multi-topic view. rang color
// manipulators can't be stored in a container, so selection is by index here.
// Indices past the palette wrap around; the [N] tag and Topics legend keep the
// mapping unambiguous even when colors repeat or are disabled.
void apply_topic_fg(std::ostream & os, std::size_t topic_index)
{
  switch (topic_index % 6) {
    case 0:
      os << rang::fgB::blue;
      break;
    case 1:
      os << rang::fgB::yellow;
      break;
    case 2:
      os << rang::fgB::magenta;
      break;
    case 3:
      os << rang::fgB::cyan;
      break;
    case 4:
      os << rang::fgB::green;
      break;
    default:
      os << rang::fgB::red;
      break;
  }
}

// Branch line for a child frame: dim branch glyphs then the child name. When
// `topic_index >= 0` (merged multi-topic view) the name is colored by that
// topic's palette entry and a " [N]" tag (N = topic_index + 1) is appended so
// the source topic stays identifiable without color; when it is < 0
// (single-topic view) the name is plain and no tag is shown. `suffix` is e.g.
// " (cycle)".
std::string tf_tree_edge_line(
  const std::string & prefix, const std::string & branch, const std::string & child,
  const std::string & suffix, bool use_color, int topic_index)
{
  const std::string tag = topic_index >= 0 ? fmt::format(" [{}]", topic_index + 1) : std::string{};
  if (!use_color) {
    return prefix + branch + child + tag + suffix;
  }
  rang::setControlMode(rang::control::Force);
  std::ostringstream oss;
  oss << rang::fg::gray << prefix << branch << rang::style::reset;
  if (topic_index >= 0) {
    apply_topic_fg(oss, static_cast<std::size_t>(topic_index));
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
// on the current path (rather than recursing). When `multi` is set, each child
// is colored and " [N]"-tagged by the topic that owns its parent→child edge
// (looked up in `edge_to_topic`); otherwise child names are plain.
std::string format_parent_map_forest(
  const std::unordered_map<std::string, std::vector<std::string>> & parent_to_children,
  const std::vector<std::string> & roots_sorted, const TreeGlyphs & g, bool use_color,
  const std::map<std::pair<std::string, std::string>, std::size_t> & edge_to_topic, bool multi)
{
  std::vector<std::string> lines;

  auto topic_index_of = [&](const std::string & parent, const std::string & child) -> int {
    if (!multi) {
      return -1;
    }
    const auto eit = edge_to_topic.find({parent, child});
    return eit != edge_to_topic.end() ? static_cast<int>(eit->second) : -1;
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
      const int topic_index = topic_index_of(parent, child);
      if (visiting.count(child) != 0) {
        lines.push_back(
          tf_tree_edge_line(prefix, branch, child, " (cycle)", use_color, topic_index));
        continue;
      }
      lines.push_back(tf_tree_edge_line(prefix, branch, child, "", use_color, topic_index));
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
// always has at least one root. `edge_to_topic` / `multi` are forwarded to the
// renderer for per-topic coloring (see format_parent_map_forest).
std::string format_topic_forest(
  const std::set<std::pair<std::string, std::string>> & edges, const TreeGlyphs & glyphs,
  bool use_color, const std::map<std::pair<std::string, std::string>, std::size_t> & edge_to_topic,
  bool multi)
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
    parent_to_children, roots, glyphs, use_color, edge_to_topic, multi);
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

// Maps each requested topic name to its TfTopic entry, preserving requested
// order. On success returns true and fills `selected_out`; on failure logs the
// names that are not tf2_msgs/msg/TFMessage topics plus the bag's available TF
// topics, and returns false.
bool resolve_requested_topics(
  const std::vector<std::string> & requested, const std::vector<TfTopic> & tf_topics,
  std::vector<TfTopic> & selected_out)
{
  std::vector<std::string> unknown;
  for (const auto & name : requested) {
    const TfTopic * match = nullptr;
    for (const auto & t : tf_topics) {
      if (t.name == name) {
        match = &t;
        break;
      }
    }
    if (match != nullptr) {
      selected_out.push_back(*match);
    } else {
      unknown.push_back(name);
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
    kLogger, "Not a tf2_msgs/msg/TFMessage topic in the bag: %s", join_csv(unknown).c_str());
  BAGWIZ_LOG_ERROR(kLogger, "Available TF topics: %s", join_csv(available).c_str());
  return false;
}

// Attributes every edge to the topic that defines it (by `requested` index) and
// rejects an edge shared by two different topics. Fills `edge_to_topic_out`;
// returns nullopt on success, or the offending message when an edge is shared.
std::optional<std::string> attribute_edges_to_topics(
  const std::vector<std::string> & requested,
  const std::map<std::string, std::set<std::pair<std::string, std::string>>> & edges_by_topic,
  std::map<std::pair<std::string, std::string>, std::size_t> & edge_to_topic_out)
{
  std::map<std::pair<std::string, std::string>, std::string> edge_owner;
  for (std::size_t i = 0; i < requested.size(); ++i) {
    const auto & topic = requested[i];
    const auto it = edges_by_topic.find(topic);
    if (it == edges_by_topic.end()) {
      continue;
    }
    for (const auto & edge : it->second) {
      const auto ins = edge_owner.emplace(edge, topic);
      if (!ins.second) {
        return fmt::format(
          "TF tree: edge '{}' -> '{}' is defined on both topic '{}' and topic '{}'; merged topics "
          "must not share an edge.",
          edge.first, edge.second, ins.first->second, topic);
      }
      edge_to_topic_out[edge] = i;
    }
  }
  return std::nullopt;
}

// Runs every TF-tree validation for the selected topics: each topic's edges
// must form a valid forest; for multiple topics no edge may be shared and the
// merged set must also be a valid forest. Fills `edge_to_topic_out` (multi
// only). Returns the first failure message, or nullopt when consistent.
std::optional<std::string> validate_tree_topics(
  const std::vector<std::string> & requested,
  const std::map<std::string, std::set<std::pair<std::string, std::string>>> & edges_by_topic,
  const std::set<std::pair<std::string, std::string>> & merged, bool multi,
  std::map<std::pair<std::string, std::string>, std::size_t> & edge_to_topic_out)
{
  for (const auto & name : requested) {
    const auto it = edges_by_topic.find(name);
    if (it == edges_by_topic.end()) {
      continue;
    }
    if (auto err = validate_union_edge_set(it->second, fmt::format("for topic '{}'", name))) {
      return err;
    }
  }
  if (!multi) {
    return std::nullopt;
  }
  if (auto err = attribute_edges_to_topics(requested, edges_by_topic, edge_to_topic_out)) {
    return err;
  }
  return validate_union_edge_set(merged, "for the merged topics");
}

// Topics legend: a "═══ Topics ═══" rule followed by one "  [N] <topic>" line
// per topic, each colored with that topic's palette entry on a TTY, then a
// trailing blank line that separates it from the tree block.
std::string format_topics_legend(const std::vector<std::string> & topics, bool use_color)
{
  std::string out = tf_section_rule("Topics", use_color);
  if (!use_color) {
    for (std::size_t i = 0; i < topics.size(); ++i) {
      out += fmt::format("  [{}] {}\n", i + 1, topics[i]);
    }
    out += "\n";
    return out;
  }
  rang::setControlMode(rang::control::Force);
  std::ostringstream oss;
  for (std::size_t i = 0; i < topics.size(); ++i) {
    oss << "  ";
    apply_topic_fg(oss, i);
    oss << "[" << (i + 1) << "] " << topics[i] << rang::style::reset << '\n';
  }
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
//   tree    Merge one or more tf2_msgs/msg/TFMessage <topics> into one validated
//           TF frame tree; on a TTY each topic's edges get a distinct color.
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
    std::vector<std::string> topics;
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
      "tree", "Merge one or more tf2_msgs/msg/TFMessage topics into one TF frame tree");
    sub->add_option("input", tree_args_.input_path, "Bag path (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    sub->add_option(
      "topics", tree_args_.topics,
      "tf2_msgs/msg/TFMessage topic(s) to merge and render; defaults to all TF "
      "topics in the bag when omitted");
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

    // With no explicit <topics>, default to every TF topic in the bag, sorted by
    // name so the per-topic [N] legend ordering is deterministic. Otherwise every
    // requested topic must be a TFMessage topic that exists in the bag;
    // resolve_requested_topics logs the unknown names + available TF topics.
    std::vector<TfTopic> selected;
    if (requested.empty()) {
      selected = tf_topics;
      std::sort(selected.begin(), selected.end(), [](const TfTopic & a, const TfTopic & b) {
        return a.name < b.name;
      });
      requested.reserve(selected.size());
      for (const auto & t : selected) {
        requested.push_back(t.name);
      }
    } else if (!resolve_requested_topics(requested, tf_topics, selected)) {
      return 1;
    }

    // Replay only the requested topics, keeping their edges grouped by topic so
    // the merged tree can color each edge by its source. No tf2 buffer is needed
    // (unlike `tf static`, which resolves transforms through one).
    std::map<std::string, std::set<std::pair<std::string, std::string>>> edges_by_topic;
    try {
      load_tf(args.input_path, selected, /*buffer=*/nullptr, &edges_by_topic);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to load TF from the bag: %s", e.what());
      return 1;
    }

    std::set<std::pair<std::string, std::string>> merged;
    for (const auto & entry : edges_by_topic) {
      merged.insert(entry.second.begin(), entry.second.end());
    }
    if (merged.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Topic(s) %s carry no decodable transforms; nothing to show.",
        join_csv(requested).c_str());
      return 1;
    }

    // Single topic stays a plain tree; two or more get per-topic colors + tags.
    const bool multi = requested.size() >= 2;

    std::map<std::pair<std::string, std::string>, std::size_t> edge_to_topic;
    if (
      const auto err =
        validate_tree_topics(requested, edges_by_topic, merged, multi, edge_to_topic)) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", err->c_str());
      return 1;
    }

    const bool color = stdout_use_color();
    const TreeGlyphs glyphs = make_tree_glyphs();

    if (multi) {
      fmt::print(stdout, "{}", format_topics_legend(requested, color));
      fmt::print(stdout, "{}", tf_section_rule("TF tree", color));
    } else {
      const std::string header_label = "TF tree (" + requested.front() + ")";
      fmt::print(stdout, "{}", tf_section_rule(header_label.c_str(), color));
    }
    fmt::print(stdout, "{}", format_topic_forest(merged, glyphs, color, edge_to_topic, multi));

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
      load_tf(args.input_path, static_topics, &tf_buffer);
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
