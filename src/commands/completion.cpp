// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/completion.hpp"

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/logging.hpp"
#include "bagwiz/core/tf_value_extract.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kLogger = "bagwiz.cmd.complete";
constexpr std::string_view kCompletionCommand = "__complete";
constexpr int kMinimumCompletionProbeArgc = 2;
constexpr int kCompletionCommandArg = 1;
constexpr int kMinimumCompletionArgc = 3;
constexpr int kCursorWordArg = 2;
constexpr int kCompletionWordsBeginArg = 3;
constexpr std::size_t kTopLevelCommandWord = 0;
constexpr std::size_t kFirstCommandArgWord = 1;
constexpr std::size_t kSecondCommandArgWord = 2;
constexpr std::size_t kThirdCommandArgWord = 3;
constexpr std::size_t kFourthCommandArgWord = 4;
constexpr std::size_t kFifthCommandArgWord = 5;

constexpr std::string_view kTfMessageType = "tf2_msgs/msg/TFMessage";

// Message types `traj dump` can process. This MUST mirror the supported set
// dispatched in src/commands/traj.cpp (TrajCommand::run_dump); keep the two in
// sync. A topic typed as anything outside this set is rejected by the command,
// so completion never offers it.
constexpr std::array<std::string_view, 4> kTrajDumpSupportedTypes{{
  kTfMessageType,
  "geometry_msgs/msg/PoseStamped",
  "geometry_msgs/msg/PoseWithCovarianceStamped",
  "nav_msgs/msg/Odometry",
}};

// `tf tree` renders only tf2_msgs/msg/TFMessage topics.
constexpr std::array<std::string_view, 1> kTfTreeSupportedTypes{{kTfMessageType}};

// Image topic types `generate video` operates on. This MUST mirror
// is_supported_type() in src/commands/generate_video.cpp; keep the two in sync.
// As with `traj dump` / `tf tree`, a topic typed as anything outside this set is
// rejected by the command, so completion never offers it.
constexpr std::array<std::string_view, 2> kGenerateVideoSupportedTypes{{
  "sensor_msgs/msg/Image",
  "sensor_msgs/msg/CompressedImage",
}};

// CameraInfo topic type accepted by `generate video --cam-info`. This MUST
// mirror the camera-info type constant in src/commands/generate_video.cpp.
constexpr std::array<std::string_view, 1> kCameraInfoType{{
  "sensor_msgs/msg/CameraInfo",
}};

constexpr std::array<std::string_view, 1> kPointCloud2Type{{
  "sensor_msgs/msg/PointCloud2",
}};

constexpr std::array<std::string_view, 1> kImuType{{
  "sensor_msgs/msg/Imu",
}};

// Declarative table of commands that take a positional <topic> argument.
// `subcommand` is empty when the command has no subcommand level (e.g.
// `bagwiz walk <input> <topic>`). `input_word` and `topic_word` are
// indices into CompletionRequest::words AFTER the leading "bagwiz" has
// been stripped, so position 0 is the top-level command. `allowed_types`
// restricts the offered topics to those whose type is listed (e.g. `tf tree`
// to TFMessage, `traj dump` to the message types it can process); an empty
// span offers every topic in the bag. When `variadic` is set the command takes
// one-or-more topics, so completion fires at `topic_word` AND every later
// positional slot (`bagwiz tf tree <input> <topic>...`); otherwise it fires
// only at the single `topic_word`.
struct TopicArgBinding
{
  std::string_view command{};
  std::string_view subcommand{};
  std::size_t input_word{0};
  std::size_t topic_word{0};
  std::span<const std::string_view> allowed_types{};
  bool variadic{false};
};

constexpr std::array<TopicArgBinding, 10> kTopicBindings{{
  {"walk", "", kFirstCommandArgWord, kSecondCommandArgWord, {}, false},
  {"traj", "dump", kSecondCommandArgWord, kThirdCommandArgWord, kTrajDumpSupportedTypes, false},
  {"traj", "join", kSecondCommandArgWord, kFourthCommandArgWord, {}, false},
  {"tf", "tree", kSecondCommandArgWord, kThirdCommandArgWord, kTfTreeSupportedTypes, true},
  {"topic", "drop", kSecondCommandArgWord, kThirdCommandArgWord, {}, true},
  {"topic", "keep", kSecondCommandArgWord, kThirdCommandArgWord, {}, true},
  // `rename` completes only its <src_topic> slot (an existing topic) from the
  // bag; <dst_topic> is a new name with nothing to suggest, so the binding is
  // non-variadic and fires at the single topic_word.
  {"topic", "rename", kSecondCommandArgWord, kThirdCommandArgWord, {}, false},
  // `generate video <input> <image_topic> <output>`: complete the single <image_topic> slot
  // from the bag's image topics (kGenerateVideoSupportedTypes). <input> and
  // <output> are paths that fall through to the shell's file completion.
  {"generate", "video", kSecondCommandArgWord, kThirdCommandArgWord, kGenerateVideoSupportedTypes,
   false},
  // `map slam <input> <pcd_topic> <output_root>`: complete the single <pcd_topic>
  // slot from the bag's PointCloud2 topics. <input> and <output_root> are paths
  // that fall through to the shell's file completion.
  {"map", "slam", kSecondCommandArgWord, kThirdCommandArgWord, kPointCloud2Type, false},
  // `cam-info replace <input> <calib_yaml> <topic>...`: complete each <topic>
  // slot from the bag's CameraInfo topics (the only type `cam-info replace`
  // rewrites). <input> falls through to file completion, and <calib_yaml> — the
  // word before the first <topic> — is a YAML path that also falls through; the
  // binding's earlier-slot flag guard skips both. The first topic sits at word 4
  // because the `replace` action verb shifts every positional one slot right; the
  // binding is variadic, so completion fires at word 4 and every later slot.
  {"cam-info", "replace", kSecondCommandArgWord, kFourthCommandArgWord, kCameraInfoType, true},
}};

enum class CompletionShell { Bash, Zsh, Fish };

struct CompletionRequest
{
  std::vector<std::string> words;
  std::size_t cursor_word = 0;
};

struct ShellDefinition
{
  CompletionShell shell{};
  std::string_view name{};
};

const std::vector<ShellDefinition> & shell_definitions()
{
  static const std::vector<ShellDefinition> kDefinitions{
    {CompletionShell::Bash, "bash"},
    {CompletionShell::Zsh, "zsh"},
    {CompletionShell::Fish, "fish"},
  };
  return kDefinitions;
}

std::vector<std::string_view> supported_shell_names()
{
  std::vector<std::string_view> names;
  for (const auto & definition : shell_definitions()) {
    names.push_back(definition.name);
  }
  return names;
}

std::vector<std::string> supported_shell_name_strings()
{
  std::vector<std::string> names;
  for (const auto & definition : shell_definitions()) {
    names.emplace_back(definition.name);
  }
  return names;
}

std::optional<CompletionShell> parse_shell(const std::string_view & name)
{
  for (const auto & definition : shell_definitions()) {
    if (definition.name == name) {
      return definition.shell;
    }
  }
  return std::nullopt;
}

bool starts_with(std::string_view value, std::string_view prefix)
{
  return value.size() >= prefix.size() && value.substr(0, prefix.size()) == prefix;
}

std::string basename(std::string_view path)
{
  const auto pos = path.find_last_of("/\\");
  if (pos == std::string_view::npos) {
    return std::string{path};
  }
  return std::string{path.substr(pos + 1)};
}

std::optional<std::size_t> parse_size(std::string_view text)
{
  try {
    std::size_t consumed = 0;
    const auto value = static_cast<std::size_t>(std::stoul(std::string{text}, &consumed));
    if (consumed != text.size()) {
      return std::nullopt;
    }
    return value;
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

std::filesystem::path expand_current_user_home(const std::filesystem::path & path)
{
  const std::string text = path.string();
  if (text != "~" && !starts_with(text, "~/")) {
    return path;
  }

  const char * const home = std::getenv("HOME");
  if (home == nullptr || std::string_view{home}.empty()) {
    return path;
  }

  if (text == "~") {
    return std::filesystem::path{home};
  }
  return std::filesystem::path{home} / text.substr(2);
}

std::optional<std::string> env_var(const char * name)
{
  const char * const value = std::getenv(name);
  if (value == nullptr || std::string_view{value}.empty()) {
    return std::nullopt;
  }
  return std::string{value};
}

std::optional<std::filesystem::path> home_directory()
{
  const auto home = env_var("HOME");
  if (!home) {
    return std::nullopt;
  }
  return std::filesystem::path{*home};
}

std::optional<std::filesystem::path> install_path_for(CompletionShell shell)
{
  const auto home = home_directory();
  if (!home) {
    return std::nullopt;
  }

  switch (shell) {
    case CompletionShell::Bash: {
      const auto base = env_var("XDG_DATA_HOME").value_or((*home / ".local" / "share").string());
      return std::filesystem::path{base} / "bash-completion" / "completions" / "bagwiz";
    }
    case CompletionShell::Zsh:
      return *home / ".zsh" / "completions" / "_bagwiz";
    case CompletionShell::Fish: {
      const auto base = env_var("XDG_CONFIG_HOME").value_or((*home / ".config").string());
      return std::filesystem::path{base} / "fish" / "completions" / "bagwiz.fish";
    }
  }
  return std::nullopt;
}

bool write_script_to(
  const std::filesystem::path & target, const std::string_view & contents, bool overwrite)
{
  std::error_code ec;
  if (std::filesystem::exists(target, ec) && !overwrite) {
    BAGWIZ_LOG_ERROR(
      kLogger, "refusing to overwrite existing file: %s (pass -w/--overwrite to replace it)",
      target.string().c_str());
    return false;
  }

  const auto parent = target.parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent, ec);
    if (ec) {
      BAGWIZ_LOG_ERROR(
        kLogger, "failed to create directory %s: %s", parent.string().c_str(),
        ec.message().c_str());
      return false;
    }
  }

  std::ofstream stream(target, std::ios::trunc);
  if (!stream) {
    BAGWIZ_LOG_ERROR(kLogger, "failed to open %s for writing", target.string().c_str());
    return false;
  }
  stream << contents;
  if (!stream) {
    BAGWIZ_LOG_ERROR(kLogger, "failed to write completion script to %s", target.string().c_str());
    return false;
  }
  return true;
}

CompletionRequest parse_request(int argc, char * const * argv)
{
  CompletionRequest request;
  if (argc < kMinimumCompletionArgc) {
    return request;
  }

  // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  const auto cursor_word = parse_size(argv[kCursorWordArg]);
  if (!cursor_word) {
    return request;
  }

  request.cursor_word = *cursor_word;
  for (int i = kCompletionWordsBeginArg; i < argc; ++i) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    request.words.emplace_back(argv[i]);
  }

  if (!request.words.empty() && basename(request.words.front()) == "bagwiz") {
    request.words.erase(request.words.begin());
    if (request.cursor_word > 0) {
      --request.cursor_word;
    }
  }

  if (request.cursor_word > request.words.size()) {
    request.cursor_word = request.words.size();
  }
  return request;
}

std::string current_word(const CompletionRequest & request)
{
  if (request.cursor_word < request.words.size()) {
    return request.words[request.cursor_word];
  }
  return {};
}

std::vector<std::string> matching(
  const std::vector<std::string_view> & candidates, const std::string_view & prefix)
{
  std::vector<std::string> result;
  for (const auto candidate : candidates) {
    if (starts_with(candidate, prefix)) {
      result.emplace_back(candidate);
    }
  }
  std::sort(result.begin(), result.end());
  return result;
}

// Flags exposed by the top-level CLI::App. `--help` / `-h` are auto-added
// by CLI11 for every App, and `--version` is wired in main().
constexpr std::array<std::string_view, 3> kTopLevelFlags{
  "--help",
  "--version",
  "-h",
};

// `--help` / `-h` are auto-added by CLI11 for every App and subcommand. Each
// per-command flag table prepends these so the user always sees them on
// `-<TAB>`, even when the command/subcommand defines no other flags of its
// own.
constexpr std::array<std::string_view, 2> kCommonHelpFlags{
  "--help",
  "-h",
};

std::vector<std::string_view> with_help(std::initializer_list<std::string_view> flags)
{
  std::vector<std::string_view> result;
  result.reserve(flags.size() + kCommonHelpFlags.size());
  for (const auto & flag : kCommonHelpFlags) {
    result.push_back(flag);
  }
  for (const auto & flag : flags) {
    result.push_back(flag);
  }
  return result;
}

std::vector<std::string> top_level_candidates(const std::string_view & prefix)
{
  if (starts_with(prefix, "-")) {
    return matching({kTopLevelFlags.begin(), kTopLevelFlags.end()}, prefix);
  }

  std::vector<std::string> result;
  for (const auto & cmd : Registry::instance().all()) {
    // Hidden commands (e.g. the `joke` easter egg) are omitted from
    // completion just as they are from --help, so they never surface to a
    // user who does not already know they exist.
    if (cmd->hidden()) {
      continue;
    }
    if (starts_with(cmd->name(), prefix)) {
      result.emplace_back(cmd->name());
    }
  }
  std::sort(result.begin(), result.end());
  return result;
}

// Topic-name completion candidates from the bag at `input_path` whose names
// start with `prefix`. When `allowed_types` is non-empty, a topic is offered
// only if its type is one of the listed types (e.g. `tf tree`'s single
// TFMessage type, or `traj dump`'s supported set); an empty `allowed_types`
// offers every topic. Best-effort: a bag that fails to open yields no
// candidates and the shell's default file completion takes over.
std::vector<std::string> complete_topics(
  const std::filesystem::path & input_path, const std::string_view & prefix,
  std::span<const std::string_view> allowed_types)
{
  std::vector<std::string> result;
  try {
    const auto reader = io::open_read(expand_current_user_home(input_path));
    for (const auto & topic : reader->topics()) {
      if (!starts_with(topic.name, prefix)) {
        continue;
      }
      if (
        !allowed_types.empty() &&
        std::find(allowed_types.begin(), allowed_types.end(), topic.type) == allowed_types.end()) {
        continue;
      }
      result.push_back(topic.name);
    }
  } catch (const std::exception &) {
    return {};
  }

  std::sort(result.begin(), result.end());
  return result;
}

constexpr std::string_view kTfStaticSuffix = "tf_static";

// True when a TF topic's name marks it static (ends with "tf_static", e.g.
// "/tf_static"). `tf static calc` resolves only the static tree, so its
// frame-id completion is restricted to these topics.
bool is_static_tf_topic_name(std::string_view name)
{
  return name.size() >= kTfStaticSuffix.size() &&
         name.compare(
           name.size() - kTfStaticSuffix.size(), kTfStaticSuffix.size(), kTfStaticSuffix) == 0;
}

// Soft cap on TF messages scanned for frame-id discovery. Static TF is
// usually one message; dynamic TF re-publishes the same edges, so the
// distinct frame-id set saturates well before this cap. The cap keeps
// per-keystroke completion latency bounded on multi-GB bags.
constexpr std::size_t kFrameIdScanMessageCap = 5000;

// Walks the bag's tf2_msgs/msg/TFMessage topics once and returns the
// sorted, deduplicated set of header.frame_id / child_frame_id values
// it observed. When `static_only` is true only *tf_static topics are
// scanned (for `tf static calc`, which resolves the static tree); otherwise
// every TF topic contributes. Reads at most `kFrameIdScanMessageCap`
// messages so completion stays responsive on large bags. Swallows every
// exception:
// completion is best-effort and a bag that fails to open should silently fall
// through to the shell's file-completion fallback rather than spew
// errors during TAB.
std::vector<std::string> collect_tf_frame_ids(
  const std::filesystem::path & bag_path, bool static_only = false)
{
  std::vector<std::string> frame_ids;
  try {
    auto reader = io::open_read(expand_current_user_home(bag_path));

    std::vector<std::string> tf_topic_names;
    for (const auto & t : reader->topics()) {
      if (t.type == kTfMessageType && (!static_only || is_static_tf_topic_name(t.name))) {
        tf_topic_names.push_back(t.name);
      }
    }
    if (tf_topic_names.empty()) {
      return {};
    }

    io::ReadFilter filter;
    filter.topics = std::move(tf_topic_names);
    reader->set_filter(filter);

    std::unordered_map<std::string, std::unique_ptr<core::decoder::Decoder>> decoders;
    for (const auto & topic_info : reader->topics()) {
      if (topic_info.type != kTfMessageType) {
        continue;
      }
      if (static_only && !is_static_tf_topic_name(topic_info.name)) {
        continue;
      }
      auto open = core::decoder::open_decoder(topic_info);
      if (!open.ok()) {
        continue;  // best-effort: skip undecodable topics rather than abort
      }
      decoders.emplace(topic_info.name, std::move(open.decoder));
    }
    if (decoders.empty()) {
      return {};
    }

    std::unordered_set<std::string> seen;
    io::RawMessage raw;
    std::size_t scanned = 0;
    while (scanned < kFrameIdScanMessageCap && reader->next(raw)) {
      ++scanned;
      auto it = decoders.find(raw.topic->name);
      if (it == decoders.end()) {
        continue;
      }
      const auto decoded = it->second->decode(raw.payload);
      if (!decoded.ok()) {
        continue;
      }
      for (const auto & t : core::extract_tf_message(*decoded.value)) {
        if (!t.header.frame_id.empty()) {
          seen.insert(t.header.frame_id);
        }
        if (!t.child_frame_id.empty()) {
          seen.insert(t.child_frame_id);
        }
      }
    }

    frame_ids.assign(seen.begin(), seen.end());
    std::sort(frame_ids.begin(), frame_ids.end());
  } catch (const std::exception &) {
    return {};
  }
  return frame_ids;
}

// Completion candidates for the value of `--from` / `--to`. Returns the
// bag's TF frame ids filtered by `prefix`. When the bag yields no frame
// ids to suggest — whether it failed to open or opened cleanly but carries
// no TF data — we return an empty list so completion simply offers nothing
// and the shell's default file-completion fallback takes over.
std::vector<std::string> complete_frame_id_value(
  const std::filesystem::path & input_path, const std::string_view & prefix,
  bool static_only = false)
{
  std::vector<std::string> result;
  for (const auto & frame : collect_tf_frame_ids(input_path, static_only)) {
    if (starts_with(frame, prefix)) {
      result.push_back(frame);
    }
  }
  return result;
}

// True when `binding` applies at the request's cursor position: command and
// subcommand match, the cursor sits on a topic slot (the single `topic_word`,
// or `topic_word`-and-later for a variadic binding), and no positional slot
// before the first topic has been replaced by a flag. Callers must ensure
// `request.words` is non-empty (so words[0] is valid). A matched slot implies
// `topic_word <= words.size()` (parse_request clamps cursor_word to
// words.size()), and the explicit `input_word` guard below means the caller can
// dereference `words[input_word]` safely regardless of the binding's indices.
bool binding_applies(const TopicArgBinding & binding, const CompletionRequest & request)
{
  if (binding.command != request.words[kTopLevelCommandWord]) {
    return false;
  }
  if (!binding.subcommand.empty()) {
    if (request.words.size() <= kFirstCommandArgWord) {
      return false;
    }
    if (request.words[kFirstCommandArgWord] != binding.subcommand) {
      return false;
    }
  }
  const bool slot_matches = binding.variadic ? (request.cursor_word >= binding.topic_word)
                                             : (request.cursor_word == binding.topic_word);
  if (!slot_matches) {
    return false;
  }
  // The caller dereferences words[input_word]; guard it explicitly so the table
  // stays safe even for a future binding with input_word >= topic_word.
  if (request.words.size() <= binding.input_word) {
    return false;
  }
  for (std::size_t i = kFirstCommandArgWord; i < binding.topic_word; ++i) {
    if (request.words[i].starts_with("-")) {
      return false;
    }
  }
  return true;
}

// Looks up the cursor position in kTopicBindings and, if a binding applies,
// dispatches to topic completion. Returns std::nullopt when no binding applies
// so the caller can fall through to per-command completion. Returning an empty
// vector means "binding matched, no candidates" (e.g. bad bag path) — the
// shell's default file completion then takes over.
std::optional<std::vector<std::string>> try_topic_completion(const CompletionRequest & request)
{
  if (request.words.empty()) {
    return std::nullopt;
  }

  const auto current = current_word(request);
  if (current.starts_with("-")) {
    return std::nullopt;
  }

  for (const auto & binding : kTopicBindings) {
    if (!binding_applies(binding, request)) {
      continue;
    }
    const auto & input_arg = request.words[binding.input_word];
    return complete_topics(input_arg, current, binding.allowed_types);
  }

  return std::nullopt;
}

std::vector<std::string> complete_complete_command(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (current.starts_with("-")) {
    return matching(with_help({"--install", "--overwrite", "-w"}), current);
  }
  if (request.cursor_word == kFirstCommandArgWord) {
    return matching(supported_shell_names(), current);
  }
  return {};
}

// `convert msg` is a nested command group with one action verb, `geo`,
// shifting every argument one word right of the flat `format` subcommand:
//
//   geo: `convert`(0) `msg`(1) `geo`(2) `<input>`(3) [--src V] [--dst V]
//        [--topic ...] [--crs V] [--origin V] [--frame-id V] [-o <out>]
//        [-w|--overwrite]
//
// At the action slot (word 2) the only candidate is `geo`. Past it, `-` words
// surface the geo flags, and the `--src` / `--dst` / `--crs` slots complete from
// the same snake_case choice sets the command's CLI::IsMember checks enforce
// (kept in sync by hand, like the other hard-coded candidate sets here). The
// `<input>` and `-o` values are paths that fall through to file completion.
std::vector<std::string> complete_convert_msg(
  const CompletionRequest & request, const std::string & current)
{
  if (request.cursor_word == kSecondCommandArgWord) {
    if (current.starts_with("-")) {
      return matching({kCommonHelpFlags.begin(), kCommonHelpFlags.end()}, current);
    }
    return matching({"geo"}, current);
  }

  // Reaching here implies cursor_word > kSecondCommandArgWord, so words[2]
  // exists (parse_request clamps cursor_word to words.size()).
  const auto & action = request.words[kSecondCommandArgWord];
  if (action != "geo") {
    return {};
  }

  if (request.cursor_word >= kThirdCommandArgWord && current.starts_with("-")) {
    return matching(
      with_help(
        {"--crs", "--dst", "--frame-id", "--origin", "--output", "--overwrite", "--src", "--topic",
         "-o", "-w"}),
      current);
  }

  if (request.cursor_word > 0) {
    const auto & previous = request.words[request.cursor_word - 1];
    if (previous == "--src") {
      return matching({"nav_sat_fix"}, current);
    }
    if (previous == "--dst") {
      return matching({"pose_with_covariance_stamped", "pose_stamped"}, current);
    }
    if (previous == "--crs") {
      return matching({"enu", "utm"}, current);
    }
  }
  return {};
}

std::vector<std::string> complete_convert(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (request.cursor_word == kFirstCommandArgWord) {
    if (current.starts_with("-")) {
      return matching({kCommonHelpFlags.begin(), kCommonHelpFlags.end()}, current);
    }
    return matching({"format", "msg"}, current);
  }

  const auto & mode = request.words[kFirstCommandArgWord];

  // `msg` is a nested command group (`msg geo`); its positional shape
  // differs from the flat `format` subcommand, so it is handled apart.
  if (mode == "msg") {
    return complete_convert_msg(request, current);
  }

  if (request.cursor_word >= kSecondCommandArgWord && current.starts_with("-")) {
    if (mode == "format") {
      return matching(with_help({"--overwrite", "--storage", "-s", "-w"}), current);
    }
  }

  if (
    request.cursor_word > 0 && (request.words[request.cursor_word - 1] == "--storage" ||
                                request.words[request.cursor_word - 1] == "-s")) {
    return matching({"mcap", "sqlite3"}, current);
  }
  return {};
}

// Shared frame-id completion entry point. Looks up the bag path at
// `input_word` (the per-command positional slot that holds the input
// bag) and dispatches to complete_frame_id_value. Bails out when the
// slot is missing or holds a flag — otherwise we would invoke
// io::open_read on something that is definitely not a bag path.
//
// Callers: traj dump/join --from/--to flag-value completion (bag at
// word 2). Parameterising the slot keeps the helper reusable for any
// future command that places the bag at a different positional index.
std::vector<std::string> complete_frame_id_arg(
  const CompletionRequest & request, std::size_t input_word, const std::string_view & current,
  bool static_only = false)
{
  if (request.words.size() <= input_word) {
    return {};
  }
  const auto & bag_arg = request.words[input_word];
  if (bag_arg.empty() || bag_arg.starts_with("-")) {
    return {};
  }
  return complete_frame_id_value(bag_arg, current, static_only);
}

std::vector<std::string> complete_traj(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (request.cursor_word == kFirstCommandArgWord) {
    if (current.starts_with("-")) {
      return matching({kCommonHelpFlags.begin(), kCommonHelpFlags.end()}, current);
    }
    return matching({"dump", "join"}, current);
  }

  if (request.cursor_word >= kSecondCommandArgWord && current.starts_with("-")) {
    const auto & mode = request.words[kFirstCommandArgWord];
    if (mode == "dump") {
      return matching(
        with_help({"--format", "--from", "--overwrite", "--to", "-f", "-w"}), current);
    }
    if (mode == "join") {
      return matching(
        with_help(
          {"--force", "--format", "--from", "--msg-type", "--output", "--overwrite", "--to", "-f",
           "-o", "-t", "-w"}),
        current);
    }
  }

  if (request.cursor_word > 0) {
    const auto & previous = request.words[request.cursor_word - 1];
    if (previous == "--format" || previous == "-f") {
      return matching({"tum"}, current);
    }
    if (previous == "--msg-type" || previous == "-t") {
      return matching({"tf"}, current);
    }
    if (previous == "--from" || previous == "--to") {
      return complete_frame_id_arg(request, kSecondCommandArgWord, current);
    }
  }
  return {};
}

// `tf static` is a command group with two actions, `calc` and `cp`. The action
// verb adds one positional slot, shifting every argument one word to the right
// of the flat `tf` subcommands.
//
//   calc: `tf`(0) `static`(1) `calc`(2) `<input>`(3) `<from>`(4) `<to>`(5) [--json]
//   cp:   `tf`(0) `static`(1) `cp`(2)   `<src>`(3)   `<dst>`(4)  [-o <out>] [-w|--overwrite]
//
// At the action slot (word 2) the candidates are `calc` / `cp`. For `calc`,
// `--json` is offered for any `-` word and the <from>/<to> slots complete from
// the bag's static `*tf_static` frame ids only (the bag path sits at word 3);
// unlike `tf walk`, dynamic-only frames are never offered. For `cp`, the
// <src>/<dst>/-o values are bag paths that fall through to the shell's file
// completion, so only the flags are surfaced.
std::vector<std::string> complete_tf_static(
  const CompletionRequest & request, const std::string & current)
{
  if (request.cursor_word == kSecondCommandArgWord) {
    if (current.starts_with("-")) {
      return matching({kCommonHelpFlags.begin(), kCommonHelpFlags.end()}, current);
    }
    return matching({"calc", "cp"}, current);
  }

  // Reaching here implies cursor_word > kSecondCommandArgWord, so words[2]
  // exists (parse_request clamps cursor_word to words.size()).
  const auto & action = request.words[kSecondCommandArgWord];

  if (action == "calc") {
    if (request.cursor_word >= kThirdCommandArgWord && current.starts_with("-")) {
      return matching(with_help({"--json"}), current);
    }
    if (
      request.cursor_word == kFourthCommandArgWord || request.cursor_word == kFifthCommandArgWord) {
      return complete_frame_id_arg(request, kThirdCommandArgWord, current, /*static_only=*/true);
    }
    return {};
  }

  if (action == "cp") {
    if (request.cursor_word >= kThirdCommandArgWord && current.starts_with("-")) {
      return matching(with_help({"--output", "--overwrite", "-o", "-w"}), current);
    }
    return {};
  }

  return {};
}

std::vector<std::string> complete_tf(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (request.cursor_word == kFirstCommandArgWord) {
    if (current.starts_with("-")) {
      return matching({kCommonHelpFlags.begin(), kCommonHelpFlags.end()}, current);
    }
    return matching({"static", "tree", "walk"}, current);
  }

  const auto & mode = request.words[kFirstCommandArgWord];

  // `static` is a nested command group (`static calc`); its positional shape
  // differs from the flat `tree` / `walk` subcommands, so it is handled apart.
  if (mode == "static") {
    return complete_tf_static(request, current);
  }

  if (request.cursor_word >= kSecondCommandArgWord && current.starts_with("-")) {
    // `tree` and `walk` carry just the implicit help flags.
    return matching({kCommonHelpFlags.begin(), kCommonHelpFlags.end()}, current);
  }

  // `tf walk <input> <from> <to>`: complete the <from>/<to> slots from the bag's
  // TF frame ids (bag path at the <input> slot, word 2). The <input> slot itself
  // falls through to the shell's file completion. `tf walk` merges every TF
  // topic, so it offers frame ids from all of them (static + dynamic).
  if (
    mode == "walk" &&
    (request.cursor_word == kThirdCommandArgWord || request.cursor_word == kFourthCommandArgWord)) {
    return complete_frame_id_arg(request, kSecondCommandArgWord, current);
  }

  return {};
}

// `topic` is a command group with three action verbs, `drop`, `keep`, and
// `rename`. At the action slot (word 1) the candidates are those verbs.
// Topic-name completion for the positional slots is handled earlier by
// try_topic_completion via kTopicBindings (drop/keep complete every <topics>...
// slot; rename completes only its <src_topic> slot); here we surface each verb's
// own flags for any `-` word.
//
//   drop:   `topic`(0) `drop`(1)   `<input>`(2) `<topics>...`(3+)        [-o <out>]
//   [-w|--overwrite] keep:   `topic`(0) `keep`(1)   `<input>`(2) `<topics>...`(3+)        [-o
//   <out>] [-w|--overwrite] rename: `topic`(0) `rename`(1) `<input>`(2) `<src_topic>`(3)
//   `<dst_topic>`(4) [-o <out>]
//   [-w|--overwrite]
std::vector<std::string> complete_topic(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (request.cursor_word == kFirstCommandArgWord) {
    if (current.starts_with("-")) {
      return matching({kCommonHelpFlags.begin(), kCommonHelpFlags.end()}, current);
    }
    return matching({"drop", "keep", "rename"}, current);
  }

  if (request.cursor_word >= kSecondCommandArgWord && current.starts_with("-")) {
    const auto & verb = request.words[kFirstCommandArgWord];
    if (verb == "drop" || verb == "keep" || verb == "rename") {
      return matching(with_help({"--output", "--overwrite", "-o", "-w"}), current);
    }
  }
  return {};
}

// `generate` is a command group for producing media from a rosbag; its sole
// subcommand is `video`. At the subcommand slot (word 1) the only candidate is
// `video`. The `<image_topic>` positional is completed earlier by
// try_topic_completion via kTopicBindings (image topics only); <input>/<output>
// are paths that fall through to the shell's file completion. Here we surface
// `video` plus its own flags for any `-` word, and values for `--cam-info`
// (CameraInfo topics), `--pcd` (PointCloud2 topics), and the enum choices for
// `--field` and `--scheme`.
//
//   video: `generate`(0) `video`(1) `<input>`(2) `<image_topic>`(3) `<output>`(4)
//          [--cam-info <topic>] [--undistort] [--resize <s>] [--pcd <topic>...]
//          [--field <f>] [--min <v>] [--max <v>] [--scheme <s>] [--point-size <n>]
//          [--alpha <a>] [-w|--overwrite]
std::vector<std::string> complete_generate(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (request.cursor_word == kFirstCommandArgWord) {
    if (current.starts_with("-")) {
      return matching({kCommonHelpFlags.begin(), kCommonHelpFlags.end()}, current);
    }
    return matching({"video"}, current);
  }

  if (request.cursor_word >= kSecondCommandArgWord && current.starts_with("-")) {
    const auto & sub = request.words[kFirstCommandArgWord];
    if (sub == "video") {
      return matching(
        with_help(
          {"--alpha", "--cam-info", "--field", "--max", "--min", "--overwrite", "--pcd",
           "--point-size", "--resize", "--scheme", "--undistort", "-w"}),
        current);
    }
  }

  auto complete_topic_after_flag =
    [&](const std::string_view & flag, const std::span<const std::string_view> & types) {
      if (request.cursor_word == 0 || request.words[request.cursor_word - 1] != flag) {
        return std::optional<std::vector<std::string>>{};
      }
      if (request.words.size() <= kSecondCommandArgWord) {
        return std::optional<std::vector<std::string>>{std::vector<std::string>{}};
      }
      const auto & bag_arg = request.words[kSecondCommandArgWord];
      if (bag_arg.empty() || bag_arg.starts_with("-")) {
        return std::optional<std::vector<std::string>>{std::vector<std::string>{}};
      }
      return std::optional<std::vector<std::string>>{
        complete_topics(expand_current_user_home(bag_arg), current, types)};
    };

  if (auto cam_info = complete_topic_after_flag("--cam-info", kCameraInfoType); cam_info) {
    return std::move(*cam_info);
  }
  if (auto pcd = complete_topic_after_flag("--pcd", kPointCloud2Type); pcd) {
    return std::move(*pcd);
  }

  if (request.cursor_word > 0 && request.words[request.cursor_word - 1] == "--field") {
    return matching({"distance", "intensity", "x", "y", "z"}, current);
  }
  if (request.cursor_word > 0 && request.words[request.cursor_word - 1] == "--scheme") {
    return matching({"inferno", "jet", "magma", "plasma", "rainbow", "turbo", "viridis"}, current);
  }
  return {};
}

// `map` is a command group for map generation and post-processing. Its action
// verbs are `slam` and `viewer`. The verb adds one positional slot, shifting
// every argument one word to the right of a flat command.
//
//   slam:   `map`(0) `slam`(1) `<input>`(2) `<pcd_topic>`(3) `<output_root>`(4)
//           [--backend <cpu|cuda|auto>] [--frame <frame_id>] [--imu <topic>]
//           [--gnss <topic>] [--input-res <m>] [--min-range <m>] [--max-range <m>]
//           [-t|--threads <N>] [--viewer] [-w|--overwrite]
//           [--no-progress] [--no-warmup-recovery] [--no-cooldown-recovery]
//           [--recovery-min-inliers <f>] [--submap-keyframes <N>]
//   viewer: `map`(0) `viewer`(1) `<map>`(2)
//
// At the action slot (word 1) the candidates are `slam` and `viewer` (or the
// help flags for a `-` word). Past it, the positional <pcd_topic> slot for
// `map slam` is completed earlier by try_topic_completion via kTopicBindings
// (PointCloud2 topics only); here we surface `slam`'s flags for any `-` word and
// complete the value of `--imu` from the bag's sensor_msgs/msg/Imu topics.
// `viewer` has no value-bearing flags and its single <map> positional is a path.

std::vector<std::string> complete_map(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (request.cursor_word == kFirstCommandArgWord) {
    if (current.starts_with("-")) {
      return matching({kCommonHelpFlags.begin(), kCommonHelpFlags.end()}, current);
    }
    return matching({"slam", "viewer"}, current);
  }

  // Reaching here implies cursor_word >= kSecondCommandArgWord, so words[1] exists.
  const auto & verb = request.words[kFirstCommandArgWord];
  if (verb == "viewer") {
    if (current.starts_with("-")) {
      return matching({kCommonHelpFlags.begin(), kCommonHelpFlags.end()}, current);
    }
    return {};
  }
  // Only `slam` has flags or a bag to complete from.
  if (verb != "slam") {
    return {};
  }

  if (current.starts_with("-")) {
    return matching(
      with_help(
        {"--backend", "--frame", "--gnss", "--imu", "--input-res", "--max-range", "--min-range",
         "--no-cooldown-recovery", "--no-progress", "--no-warmup-recovery", "--overwrite",
         "--recovery-min-inliers", "--submap-keyframes", "--threads", "--viewer", "-t", "-w"}),
      current);
  }

  if (request.cursor_word > 0 && request.words[request.cursor_word - 1] == "--backend") {
    return matching({"auto", "cpu", "cuda"}, current);
  }

  if (request.cursor_word == 0 || request.words[request.cursor_word - 1] != "--imu") {
    return {};
  }
  if (request.words.size() <= kSecondCommandArgWord) {
    return {};
  }
  const auto & bag_arg = request.words[kSecondCommandArgWord];
  if (bag_arg.empty() || bag_arg.starts_with("-")) {
    return {};
  }
  return complete_topics(expand_current_user_home(bag_arg), current, kImuType);
}

// `ls <input>` lists topics. Its only flag is `-l/--long` (per-topic COUNT and
// HZ); <input> is a path that falls through to the shell's file completion. We
// surface `--long` plus the implicit help flags for any `-` word.
//
//   ls: `ls`(0) `<input>`(1) [-l|--long]
std::vector<std::string> complete_ls(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (current.starts_with("-")) {
    return matching(with_help({"--long"}), current);
  }
  return {};
}

// `walk <input> <topic>` walks a single topic's messages. Its <topic> positional
// is completed earlier by try_topic_completion via kTopicBindings (every topic in
// the bag); <input> is a path that falls through to the shell's file completion.
// Here we surface walk's own `--cam-info` flag (plus the implicit help flags) for
// any `-` word, and complete the value of `--cam-info` from the bag's CameraInfo
// topics — mirroring `generate video --cam-info`. The bag path sits at the
// <input> slot, word 1.
//
//   walk: `walk`(0) `<input>`(1) `<topic>`(2) [--cam-info <topic>]
std::vector<std::string> complete_walk(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (current.starts_with("-")) {
    return matching(with_help({"--cam-info"}), current);
  }

  // Complete the value of `--cam-info` from the bag's CameraInfo topics. Bail out
  // when the <input> slot is missing or holds a flag, so we never call the bag
  // reader on something that is not a bag path.
  if (request.cursor_word > 0 && request.words[request.cursor_word - 1] == "--cam-info") {
    if (request.words.size() <= kFirstCommandArgWord) {
      return {};
    }
    const auto & bag_arg = request.words[kFirstCommandArgWord];
    if (bag_arg.empty() || bag_arg.starts_with("-")) {
      return {};
    }
    return complete_topics(expand_current_user_home(bag_arg), current, kCameraInfoType);
  }
  return {};
}

// `check` is a command group for rosbag integrity checks. Its sole subcommand is
// `broken`. At the subcommand slot (word 1) the only candidate is `broken` (or the
// implicit help flags for a `-` word). Past it, `broken`'s flags are surfaced for
// any `-` word; its single <input> positional is a path that falls through to the
// shell's file completion.
//
//   broken: `check`(0) `broken`(1) `<input>`(2) [--rm] [--deep]
std::vector<std::string> complete_check(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (request.cursor_word == kFirstCommandArgWord) {
    if (current.starts_with("-")) {
      return matching({kCommonHelpFlags.begin(), kCommonHelpFlags.end()}, current);
    }
    return matching({"broken"}, current);
  }

  if (request.cursor_word >= kSecondCommandArgWord && current.starts_with("-")) {
    const auto & sub = request.words[kFirstCommandArgWord];
    if (sub == "broken") {
      return matching(with_help({"--deep", "--rm"}), current);
    }
  }
  return {};
}

// `cam-info` is a command group for sensor_msgs/msg/CameraInfo operations. Its
// sole subcommand is `replace`. At the subcommand slot (word 1) the only candidate
// is `replace` (or the implicit help flags for a `-` word). The variadic <topic>...
// positionals are completed earlier by try_topic_completion via kTopicBindings
// (CameraInfo topics only, at word 4 and every later slot); <input> and
// <calib_yaml> are paths that fall through to the shell's file completion. Here we
// surface `replace`'s flags for any `-` word. The `--frame-id` value is a free-form
// header override with nothing to suggest, and `-o`/`--output`'s value is an output
// path, so neither gets value completion.
//
//   replace: `cam-info`(0) `replace`(1) `<input>`(2) `<calib_yaml>`(3) `<topic>...`(4+)
//            [--frame-id <id>] [-o <out>] [-w|--overwrite]
std::vector<std::string> complete_cam_info(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (request.cursor_word == kFirstCommandArgWord) {
    if (current.starts_with("-")) {
      return matching({kCommonHelpFlags.begin(), kCommonHelpFlags.end()}, current);
    }
    return matching({"replace"}, current);
  }

  if (request.cursor_word >= kSecondCommandArgWord && current.starts_with("-")) {
    const auto & sub = request.words[kFirstCommandArgWord];
    if (sub == "replace") {
      return matching(with_help({"--frame-id", "--output", "--overwrite", "-o", "-w"}), current);
    }
  }
  return {};
}

// `pcd` is a command group for PointCloud2 topic processing. Its subcommands are
// `concat` and `undistort`. At the subcommand slot (word 1) the candidates are
// those two (or the implicit help flags for a `-` word). `<input>` is a path
// that falls through to the shell's file completion. Past the subcommand we
// surface each subcommand's own flags for any `-` word.
//
// For `concat`, `<output_topic>` is a free-form new topic name with nothing to
// suggest. PointCloud2 topic values complete for every `--pcd` value
// (read from the bag named at word 2). `--stamp-offset` takes a single
// `<topic>=<value>`, so its `<topic>` half completes to the same PointCloud2
// topics (as `<topic>=`) until the value word contains `=`. `--frame`,
// `--tolerance`, and `-o`/`--output` take free-form / numeric / path values, so
// they get no value completion.
//
//   concat: `pcd`(0) `concat`(1) `<input>`(2) `<output_topic>`(3)
//           --pcd <t...> [--frame <f>] [--tolerance <val>]
//           [--stamp-offset <t=v>]... [-o <out>] [--drop-inputs] [--force]
//           [-w|--overwrite]
//
// For `undistort`, `<pose_topic>` is a free-form topic name (accepted types are
// TFMessage / Odometry / PoseStamped / PoseWithCovarianceStamped) with nothing
// to suggest. `--pcd` is variadic and completes PointCloud2 topics from the bag
// named at word 2, mirroring concat's `--pcd`. `--from`/`--to` complete
// the bag's TF frame ids, mirroring `traj dump`/`join`. `-o`/`--output` takes a
// path, so it gets no value completion.
//
//   undistort: `pcd`(0) `undistort`(1) `<input>`(2) `<pose_topic>`(3)
//              --pcd <t...> [--from <frame>] [--to <frame>] [-o <out>]
//              [-w|--overwrite]
std::vector<std::string> complete_pcd(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (request.cursor_word == kFirstCommandArgWord) {
    if (current.starts_with("-")) {
      return matching({kCommonHelpFlags.begin(), kCommonHelpFlags.end()}, current);
    }
    return matching({"concat", "undistort"}, current);
  }

  if (request.cursor_word >= kSecondCommandArgWord && current.starts_with("-")) {
    const auto & sub = request.words[kFirstCommandArgWord];
    if (sub == "concat") {
      return matching(
        with_help(
          {"--drop-inputs", "--force", "--frame", "--output", "--overwrite", "--pcd",
           "--stamp-offset", "--tolerance", "-o", "-w"}),
        current);
    }
    if (sub == "undistort") {
      return matching(
        with_help({"--from", "--output", "--overwrite", "--pcd", "--to", "-o", "-w"}), current);
    }
  }

  // --stamp-offset takes a single <topic>=<value>; complete the <topic> half from
  // the bag's PointCloud2 topics (like --pcd) while the value word has no
  // '=' yet. Each candidate carries a trailing '=' so the shell scripts drop the
  // auto-space and leave the cursor on the value. Unlike --pcd this is
  // single-valued, so only the word immediately after --stamp-offset is its value.
  if (
    request.cursor_word > kSecondCommandArgWord &&
    request.words[request.cursor_word - 1] == "--stamp-offset" &&
    current.find('=') == std::string::npos) {
    const auto & bag_arg = request.words[kSecondCommandArgWord];
    if (!bag_arg.empty() && !bag_arg.starts_with("-")) {
      auto topics = complete_topics(expand_current_user_home(bag_arg), current, kPointCloud2Type);
      for (auto & topic : topics) {
        topic += '=';
      }
      return topics;
    }
  }

  // --pcd is variadic: complete PointCloud2 topics for every value in
  // its run, not just the first. Walk back from the cursor to the nearest option
  // word; if it is --pcd, the cursor is still consuming its values, so
  // offer topics from the bag named at word 2 (the <input> positional).
  for (std::size_t w = request.cursor_word; w > kSecondCommandArgWord;) {
    const auto & word = request.words[--w];
    if (!word.starts_with("-")) {
      continue;  // a topic value already given to --pcd; keep scanning
    }
    if (word == "--pcd") {
      const auto & bag_arg = request.words[kSecondCommandArgWord];
      if (!bag_arg.empty() && !bag_arg.starts_with("-")) {
        return complete_topics(expand_current_user_home(bag_arg), current, kPointCloud2Type);
      }
    }
    break;  // the nearest option decides; a non-pcd option ends the run
  }

  // undistort's --pcd is likewise variadic: complete PointCloud2 topics for
  // every value in its run, not just the first, mirroring --pcd above.
  for (std::size_t w = request.cursor_word; w > kSecondCommandArgWord;) {
    const auto & word = request.words[--w];
    if (!word.starts_with("-")) {
      continue;  // a topic value already given to --pcd; keep scanning
    }
    if (word == "--pcd") {
      const auto & bag_arg = request.words[kSecondCommandArgWord];
      if (!bag_arg.empty() && !bag_arg.starts_with("-")) {
        return complete_topics(expand_current_user_home(bag_arg), current, kPointCloud2Type);
      }
    }
    break;  // the nearest option decides; a non-pcd option ends the run
  }

  // undistort's --from/--to complete the bag's TF frame ids, mirroring
  // `traj dump`/`join` (bag path at the same word 2 <input> slot).
  if (request.cursor_word > 0) {
    const auto & previous = request.words[request.cursor_word - 1];
    if (previous == "--from" || previous == "--to") {
      return complete_frame_id_arg(request, kSecondCommandArgWord, current);
    }
  }
  return {};
}

std::vector<std::string> complete_request(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (request.words.empty() || request.cursor_word == kTopLevelCommandWord) {
    return top_level_candidates(current);
  }

  if (auto topic_candidates = try_topic_completion(request)) {
    return std::move(*topic_candidates);
  }

  const auto & command = request.words.front();
  if (command == "complete") {
    return complete_complete_command(request);
  }
  if (command == "convert") {
    return complete_convert(request);
  }
  if (command == "traj") {
    return complete_traj(request);
  }
  if (command == "tf") {
    return complete_tf(request);
  }
  if (command == "topic") {
    return complete_topic(request);
  }
  if (command == "generate") {
    return complete_generate(request);
  }
  if (command == "map") {
    return complete_map(request);
  }
  if (command == "cam-info") {
    return complete_cam_info(request);
  }
  if (command == "pcd") {
    return complete_pcd(request);
  }
  if (command == "check") {
    return complete_check(request);
  }
  if (command == "walk") {
    return complete_walk(request);
  }
  if (command == "ls") {
    return complete_ls(request);
  }
  return {};
}

void print_candidates(const std::vector<std::string> & candidates)
{
  for (const auto & candidate : candidates) {
    std::cout << candidate << '\n';
  }
}

const char * bash_completion_script()
{
  return R"BWCOMP(# bash completion for bagwiz.
# Install with:
#   bagwiz complete bash > ~/.local/share/bash-completion/completions/bagwiz

_bagwiz_completion()
{
  local cur out
  cur="${COMP_WORDS[COMP_CWORD]}"

  if ! out="$(bagwiz __complete "$COMP_CWORD" "${COMP_WORDS[@]}" 2>/dev/null)"; then
    return 0
  fi

  if [[ -z "${out}" ]]; then
    return 0
  fi

  local IFS=$'\n'
  COMPREPLY=($(compgen -W "${out}" -- "${cur}"))

  # `<topic>=` candidates (e.g. `pcd concat --stamp-offset <topic>=<value>`) must
  # not receive the default trailing space, so the value can be typed right after
  # the `=`. Suppress it only when every candidate ends with `=`.
  if [[ ${#COMPREPLY[@]} -gt 0 ]]; then
    local __bw_all_eq=1 __bw_c
    for __bw_c in "${COMPREPLY[@]}"; do
      [[ "${__bw_c}" == *= ]] || { __bw_all_eq=0; break; }
    done
    [[ ${__bw_all_eq} -eq 1 ]] && compopt -o nospace
  fi
}

complete -o default -F _bagwiz_completion bagwiz
)BWCOMP";
}

const char * zsh_completion_script()
{
  return R"BWCOMP(#compdef bagwiz
# zsh completion for bagwiz.
# Install with:
#   mkdir -p ~/.zsh/completions
#   bagwiz complete zsh > ~/.zsh/completions/_bagwiz
# Then ensure ~/.zsh/completions is in $fpath before `compinit` in ~/.zshrc:
#   fpath=(~/.zsh/completions $fpath)
#   autoload -Uz compinit && compinit

_bagwiz()
{
  local -a candidates
  local out

  if ! out="$(bagwiz __complete $((CURRENT - 1)) "${words[@]}" 2>/dev/null)"; then
    _files
    return 0
  fi

  if [[ -z "${out}" ]]; then
    _files
    return 0
  fi

  local IFS=$'\n'
  candidates=(${(f)out})

  if (( ${#candidates} == 0 )); then
    _files
    return 0
  fi

  # `<topic>=` candidates (e.g. `pcd concat --stamp-offset <topic>=<value>`) must
  # keep the cursor on the value, so add them with an empty suffix (no trailing
  # space) instead of via _describe. Only when every candidate ends with `=`.
  local __bw_all_eq=1 __bw_c
  for __bw_c in "${candidates[@]}"; do
    [[ "${__bw_c}" == *= ]] || { __bw_all_eq=0; break; }
  done
  if (( __bw_all_eq )); then
    compadd -S '' -- "${candidates[@]}" && return 0
    _files
    return 0
  fi

  _describe -t bagwiz 'bagwiz' candidates && return 0
  _files
}

compdef _bagwiz bagwiz

if [ "${funcstack[1]}" = "_bagwiz" ]; then
  _bagwiz "$@"
fi
)BWCOMP";
}

const char * fish_completion_script()
{
  return R"BWCOMP(# fish completion for bagwiz.
# Install with:
#   bagwiz complete fish > ~/.config/fish/completions/bagwiz.fish

function __bagwiz_complete
    set -l tokens (commandline -opc)
    set -l current (commandline -ct)
    set -l cursor (count $tokens)
    bagwiz __complete $cursor $tokens $current 2>/dev/null
end

function __bagwiz_no_candidates
    set -l result (__bagwiz_complete)
    test -z "$result"
end

# Show bagwiz-supplied candidates when the helper returns any; otherwise fall
# back to the shell's default file completion (matches bash's `complete -o
# default` behavior).
complete -c bagwiz -f -a '(__bagwiz_complete)'
complete -c bagwiz -F -n __bagwiz_no_candidates
)BWCOMP";
}

const char * completion_script(CompletionShell shell)
{
  switch (shell) {
    case CompletionShell::Bash:
      return bash_completion_script();
    case CompletionShell::Zsh:
      return zsh_completion_script();
    case CompletionShell::Fish:
      return fish_completion_script();
  }
  return "";
}

// The command that loads `target` into the current shell session. All three
// shells autoload the script from the standard completion directory on the next
// startup, so the only thing needed to activate it immediately is to source the
// file we just wrote.
std::string activate_command(CompletionShell shell, const std::filesystem::path & target)
{
  switch (shell) {
    case CompletionShell::Bash:
    case CompletionShell::Zsh:
    case CompletionShell::Fish:
      return "source " + target.string();
  }
  return {};
}

class CompleteCommand : public Command
{
public:
  [[nodiscard]] std::string_view name() const override { return "complete"; }
  [[nodiscard]] std::string_view description() const override
  {
    return "Generate shell completion scripts";
  }

  void configure(CLI::App & app) override
  {
    app.add_option("shell", shell_, "Shell to generate completions for")
      ->required()
      ->check(CLI::IsMember(supported_shell_name_strings()));
    app.add_flag(
      "--install", install_,
      "Write the script to the shell's standard completion directory instead of stdout");
    app.add_flag(
      "-w,--overwrite", overwrite_, "Overwrite an existing file when used with --install");
  }

  int run() override
  {
    const auto shell = parse_shell(shell_);
    if (!shell) {
      BAGWIZ_LOG_ERROR(kLogger, "unsupported shell: %s", shell_.c_str());
      return 1;
    }

    if (!install_) {
      std::cout << completion_script(*shell);
      return 0;
    }

    const auto target = install_path_for(*shell);
    if (!target) {
      BAGWIZ_LOG_ERROR(kLogger, "cannot determine install path: HOME is not set");
      return 1;
    }

    if (!write_script_to(*target, completion_script(*shell), overwrite_)) {
      return 1;
    }
    std::cout << "installed: " << target->string() << '\n'
              << "Completion will be active in new terminal sessions.\n"
              << "To enable it in the current shell now, run:\n"
              << "  " << activate_command(*shell, *target) << '\n';
    return 0;
  }

private:
  std::string shell_;
  bool install_ = false;
  bool overwrite_ = false;
};

}  // namespace

std::vector<std::string> supported_shells()
{
  return supported_shell_name_strings();
}

std::optional<std::string> completion_script_for(const std::string_view & shell)
{
  const auto parsed = parse_shell(shell);
  if (!parsed) {
    return std::nullopt;
  }
  return std::string{completion_script(*parsed)};
}

std::optional<std::filesystem::path> default_install_path_for(const std::string_view & shell)
{
  const auto parsed = parse_shell(shell);
  if (!parsed) {
    return std::nullopt;
  }
  return install_path_for(*parsed);
}

std::optional<std::string> activate_command_for(
  const std::string_view & shell, const std::filesystem::path & target)
{
  const auto parsed = parse_shell(shell);
  if (!parsed) {
    return std::nullopt;
  }
  return activate_command(*parsed, target);
}

bool install_completion_script(
  const std::string_view & shell, const std::filesystem::path & target, bool overwrite)
{
  const auto parsed = parse_shell(shell);
  if (!parsed) {
    BAGWIZ_LOG_ERROR(kLogger, "unsupported shell: %s", std::string(shell).c_str());
    return false;
  }
  return write_script_to(target, completion_script(*parsed), overwrite);
}

bool is_completion_request(int argc, char * const * argv)
{
  if (argc < kMinimumCompletionProbeArgc) {
    return false;
  }
  // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  const char * const command = argv[kCompletionCommandArg];
  return command != nullptr && std::string_view{command} == kCompletionCommand;
}

int run_completion_request(int argc, char * const * argv)
{
  const auto request = parse_request(argc, argv);
  print_candidates(complete_request(request));
  return 0;
}

BAGWIZ_REGISTER_COMMAND(CompleteCommand)

}  // namespace bagwiz::commands
