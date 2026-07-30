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
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/tf/tf_topics.hpp"
#include "bagwiz/core/tf/tf_value_extract.hpp"
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

// Image topic types the shared to_packed_raster() decoder accepts —
// `generate video` rendering, `walk`'s image preview, and `map slam --cam`
// colorization all gate on it. This MUST mirror is_supported_image_type() in
// bagwiz_image/src/core/image/packed_raster.cpp (and is_supported_type() in
// src/commands/generate_video_common.cpp); keep them in sync. As with `traj dump` /
// `tf tree`, a topic typed as anything outside this set is rejected by the
// command, so completion never offers it.
constexpr std::array<std::string_view, 2> kImageTopicTypes{{
  "sensor_msgs/msg/Image",
  "sensor_msgs/msg/CompressedImage",
}};

// CameraInfo topic type accepted by `generate video --cam-info`. This MUST
// mirror the camera-info type constants in src/commands/cam_info_common.hpp
// and bagwiz_image/src/core/image/camera_info_resolver.cpp (the resolver
// `generate video --cam-info` goes through).
constexpr std::array<std::string_view, 1> kCameraInfoType{{
  "sensor_msgs/msg/CameraInfo",
}};

// Every command that takes multiple topics spells the flag this way, so
// topics are named consistently across the whole CLI.
constexpr std::array<std::string_view, 2> kTopicsFlags{{"-t", "--topics"}};

// Required rosbag input is always behind -i/--input after the flag conversion.
constexpr std::array<std::string_view, 2> kInputFlags{{"-i", "--input"}};

// Single topic operand behind -t/--topic.
constexpr std::array<std::string_view, 2> kSingleTopicFlags{{"-t", "--topic"}};

// Source topic for `topic rename`.
constexpr std::array<std::string_view, 2> kSrcTopicFlags{{"-s", "--src-topic"}};

// PointCloud2 topic(s) for `map slam --pcd`.
constexpr std::array<std::string_view, 1> kPcdFlags{{"--pcd"}};

constexpr std::array<std::string_view, 1> kPointCloud2Type{{
  "sensor_msgs/msg/PointCloud2",
}};

constexpr std::array<std::string_view, 1> kImuType{{
  "sensor_msgs/msg/Imu",
}};

// Declarative table of commands whose topic-value slots should be completed
// from a bag. `subcommand` is empty when the command has no subcommand level
// (e.g. `bagwiz walk -i <bag> -t <topic>`). `input_flags` names the flag(s)
// whose value provides the bag path; `topic_flags` names the flag(s) whose
// value(s) should be completed as topic names. `allowed_types` restricts the
// offered topics to those whose type is listed (e.g. `tf tree` to TFMessage,
// `traj dump` to the message types it can process); an empty span offers every
// topic in the bag. `variadic` is true when the flag accepts several values per
// occurrence.
struct TopicArgBinding
{
  std::string_view command{};
  std::string_view subcommand{};
  std::span<const std::string_view> input_flags{};
  std::span<const std::string_view> topic_flags{};
  std::span<const std::string_view> allowed_types{};
  bool variadic{false};
};

constexpr std::array<TopicArgBinding, 11> kTopicBindings{{
  // `walk -i <bag> -t <topic>`
  {"walk", "", kInputFlags, kSingleTopicFlags, {}, false},
  // `traj dump -i <bag> -t <topic>`
  {"traj", "dump", kInputFlags, kSingleTopicFlags, kTrajDumpSupportedTypes, false},
  // `tf tree -i <bag> [-t/--topics <topic>...]`
  {"tf", "tree", kInputFlags, kTopicsFlags, kTfTreeSupportedTypes, true},
  // `topic drop|keep -i <bag> -t/--topics <selector>...`
  {"topic", "drop", kInputFlags, kTopicsFlags, {}, true},
  {"topic", "keep", kInputFlags, kTopicsFlags, {}, true},
  // `topic rename -i <bag> -s/--src-topic <topic>` (dst is a new name)
  {"topic", "rename", kInputFlags, kSrcTopicFlags, {}, false},
  // `generate video -i <bag> -t <topic>`
  {"generate", "video", kInputFlags, kSingleTopicFlags, kImageTopicTypes, false},
  // `map slam -i <bag> --pcd <topic>`
  {"map", "slam", kInputFlags, kPcdFlags, kPointCloud2Type, false},
  // `cam-info replace -i <bag> -t/--topics <topic>...`
  {"cam-info", "replace", kInputFlags, kTopicsFlags, kCameraInfoType, true},
  // `cam-info dump -i <bag> -t <topic>`
  {"cam-info", "dump", kInputFlags, kSingleTopicFlags, kCameraInfoType, false},
  // `cam-info recompute-p -i <bag> -t/--topics <topic>...`
  {"cam-info", "recompute-p", kInputFlags, kTopicsFlags, kCameraInfoType, true},
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
  const auto expanded = expand_current_user_home(input_path);

  // A bare single-file `.db3.zstd` envelope carries no metadata.yaml, so reading
  // its topic list forces a full decompress of the whole database to a temp file
  // — seconds of hang per TAB on a multi-GB bag. Offer nothing so the shell's
  // default file completion takes over instead of blocking. Directory bags (FILE-
  // or MESSAGE-mode) serve their topic list from metadata.yaml without touching
  // the envelope, so they stay fast and are deliberately not skipped here.
  std::error_code ec;
  if (!std::filesystem::is_directory(expanded, ec) && io::is_file_compressed_bag(expanded)) {
    return {};
  }

  try {
    const auto reader = io::open_read(expanded);
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
  const auto expanded = expand_current_user_home(bag_path);

  // Frame-id discovery iterates TF messages. For a FILE-mode zstd bag the first
  // read decompresses the whole shard to a temp .db3 up front — seconds of hang
  // per TAB on a multi-GB bag, regardless of the scan cap. Offer nothing instead.
  // MESSAGE-mode bags decompress per message (bounded by kFrameIdScanMessageCap)
  // and uncompressed bags are cheap, so both keep working.
  if (io::is_file_compressed_bag(expanded)) {
    return {};
  }

  try {
    auto reader = io::open_read(expanded);

    std::vector<std::string> tf_topic_names;
    for (const auto & t : reader->topics()) {
      if (t.type == kTfMessageType && (!static_only || core::is_static_tf_topic(t.name))) {
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
      if (static_only && !core::is_static_tf_topic(topic_info.name)) {
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

// Completion candidates for the value of `--of` / `--ref`. Returns the
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

// Returns the value of the most recently occurring flag in `flags`, or nullopt
// if none is present before the cursor word.
std::optional<std::string_view> find_flag_value(
  const CompletionRequest & request, std::span<const std::string_view> flags)
{
  for (std::size_t i = request.cursor_word; i > kFirstCommandArgWord; --i) {
    const auto & prev = request.words[i - 1];
    for (const auto & flag : flags) {
      if (prev == flag) {
        if (i < request.words.size()) {
          return request.words[i];
        }
        return std::nullopt;
      }
    }
  }
  return std::nullopt;
}

// True when the cursor sits in a value slot owned by `flag`. Unlike a plain
// `words[cursor - 1] == flag` check this walks back over the flag's earlier
// values, so a variadic flag completes at its second and later value slots too.
// The walk stops at the top-level command word: nothing at or before it is a
// flag value.
bool is_value_slot_of(const CompletionRequest & request, const std::string_view & flag)
{
  for (std::size_t i = request.cursor_word; i > kFirstCommandArgWord; --i) {
    const auto & word = request.words[i - 1];
    if (word == flag) {
      return true;
    }
    if (word.starts_with("-")) {
      return false;  // some other flag owns this slot
    }
  }
  return false;
}

// True when `binding` applies at the request's cursor position: command and
// subcommand match, the cursor sits on a value slot of one of the topic flags,
// and the input flag(s) name a reachable bag path.
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
  const bool on_a_flag_value = std::any_of(
    binding.topic_flags.begin(), binding.topic_flags.end(),
    [&](const std::string_view & f) { return is_value_slot_of(request, f); });
  if (!on_a_flag_value) {
    return false;
  }
  const auto input_path = find_flag_value(request, binding.input_flags);
  return input_path.has_value() && !input_path->empty() && !input_path->starts_with("-");
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
    const auto input_path = find_flag_value(request, binding.input_flags);
    if (!input_path) {
      continue;
    }
    return complete_topics(*input_path, current, binding.allowed_types);
  }

  return std::nullopt;
}

std::vector<std::string> complete_complete_command(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (current.starts_with("-")) {
    return matching(with_help({"--install", "--overwrite", "--shell", "-w"}), current);
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
    return matching({"format"}, current);
  }

  const auto & mode = request.words[kFirstCommandArgWord];

  if (request.cursor_word >= kSecondCommandArgWord && current.starts_with("-")) {
    if (mode == "format") {
      return matching(
        with_help({"--input", "--output", "--overwrite", "--storage", "-i", "-o", "-w"}), current);
    }
  }

  if (request.cursor_word > 0 && request.words[request.cursor_word - 1] == "--storage") {
    return matching({"mcap", "sqlite3"}, current);
  }
  return {};
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
        with_help(
          {"--format", "--input", "--of", "--output", "--overwrite", "--ref", "--topic", "-f", "-i",
           "-o", "-t", "-w"}),
        current);
    }
    if (mode == "join") {
      return matching(
        with_help(
          {"--force", "--format", "--input", "--msg-type", "--of", "--output", "--overwrite",
           "--ref", "--topic", "--traj", "-i", "-m", "-o", "-t", "-w"}),
        current);
    }
  }

  if (request.cursor_word > 0) {
    const auto & previous = request.words[request.cursor_word - 1];
    if (previous == "--format" || previous == "-f") {
      return matching({"tum"}, current);
    }
    if (previous == "--msg-type" || previous == "-m") {
      return matching({"tf"}, current);
    }
    if (previous == "--of" || previous == "--ref") {
      const auto bag_arg = find_flag_value(request, kInputFlags);
      if (!bag_arg || bag_arg->empty() || bag_arg->starts_with("-")) {
        return {};
      }
      return complete_frame_id_value(*bag_arg, current);
    }
  }
  return {};
}

// `tf static` is a command group with two actions, `calc` and `cp`. The action
// verb adds one positional slot, shifting every argument one word to the right
// of the flat `tf` subcommands.
//
//   calc: `tf`(0) `static`(1) `calc`(2) -i|--input <bag> --of <frame> --ref <frame> [--json]
//   cp:   `tf`(0) `static`(1) `cp`(2)   --src <bag> --dst <bag> [-o <out>] [-w|--overwrite]
//
// At the action slot (word 2) the candidates are `calc` / `cp`. For `calc`,
// `-i`/`--input`/`--json`/`--of`/`--ref` are offered for any `-` word, and the
// `--of`/`--ref` value slots complete from the bag's static `*tf_static` frame
// ids only. For `cp`, the `--src`/`--dst`/`--output` flags are surfaced.
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
      return matching(with_help({"--input", "--json", "--of", "--ref", "-i"}), current);
    }
    if (request.cursor_word > 0) {
      const auto & previous = request.words[request.cursor_word - 1];
      if (previous == "--of" || previous == "--ref") {
        const auto bag_arg = find_flag_value(request, kInputFlags);
        if (!bag_arg || bag_arg->empty() || bag_arg->starts_with("-")) {
          return {};
        }
        return complete_frame_id_value(*bag_arg, current, /*static_only=*/true);
      }
    }
    return {};
  }

  if (action == "cp") {
    if (request.cursor_word >= kThirdCommandArgWord && current.starts_with("-")) {
      return matching(
        with_help({"--dst", "--output", "--overwrite", "--src", "-o", "-w"}), current);
    }
    return {};
  }

  return {};
}

// `tf` has two subcommands: `tree`, and `static` (itself a nested command
// group, handled by complete_tf_static). At the subcommand slot (word 1) the
// candidates are `static` / `tree`.
//
//   tree: `tf`(0) `tree`(1) -i|--input <bag> [-t|--topics <topic-or-selector>...]
//
// `tree`'s -t/--topics value completion is handled earlier by
// try_topic_completion via kTopicBindings (TFMessage topics only, at every
// value slot since the flag is variadic and optional); here we surface only
// `tree`'s own flags for any `-` word.
std::vector<std::string> complete_tf(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (request.cursor_word == kFirstCommandArgWord) {
    if (current.starts_with("-")) {
      return matching({kCommonHelpFlags.begin(), kCommonHelpFlags.end()}, current);
    }
    return matching({"static", "tree"}, current);
  }

  const auto & mode = request.words[kFirstCommandArgWord];

  // `static` is a nested command group (`static calc`); its positional shape
  // differs from the flat `tree` subcommand, so it is handled apart.
  if (mode == "static") {
    return complete_tf_static(request, current);
  }

  if (request.cursor_word >= kSecondCommandArgWord && current.starts_with("-")) {
    if (mode == "tree") {
      return matching(with_help({"--input", "--topics", "-i", "-t"}), current);
    }
    return matching({kCommonHelpFlags.begin(), kCommonHelpFlags.end()}, current);
  }

  return {};
}

// `topic` is a command group with three action verbs, `drop`, `keep`, and
// `rename`. At the action slot (word 1) the candidates are those verbs.
// Topic-name completion for `drop`/`keep` is handled earlier by
// try_topic_completion via kTopicBindings in flag mode (every -t/--topics value
// slot); `rename` completes only its --src-topic value slot the same way.
// Here we surface each verb's own flags for any `-` word.
//
//   drop:   `topic`(0) `drop`(1)   -i|--input <bag> -t|--topics <selector>...
//           [-o <out>] [-w|--overwrite]
//   keep:   `topic`(0) `keep`(1)   -i|--input <bag> -t|--topics <selector>...
//           [-o <out>] [-w|--overwrite]
//   rename: `topic`(0) `rename`(1) -i|--input <bag> -s|--src-topic <topic>
//           -d|--dst-topic <topic> [-o <out>] [-w|--overwrite]
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
    if (verb == "drop" || verb == "keep") {
      return matching(
        with_help({"--input", "--output", "--overwrite", "--topics", "-i", "-o", "-t", "-w"}),
        current);
    }
    if (verb == "rename") {
      return matching(
        with_help(
          {"--dst-topic", "--input", "--output", "--overwrite", "--src-topic", "-d", "-i", "-o",
           "-s", "-w"}),
        current);
    }
  }
  return {};
}

// `generate` is a command group for producing media from a rosbag; its sole
// subcommand is `video`. At the subcommand slot (word 1) the only candidate is
// `video`. The image topic is completed earlier by try_topic_completion via
// kTopicBindings (image topics only). Here we surface `video` plus its own
// flags for any `-` word, and values for `--cam-info` (CameraInfo topics),
// `--pcd` (PointCloud2 topics), and the enum choices for `--field` and
// `--scheme`.
//
//   video: `generate`(0) `video`(1) -i|--input <bag> -t|--topic <image_topic>
//          -o|--output <path> [--cam-info <topic>] [--undistort] [--resize <s>]
//          [--pcd <topic>...] [--field <f>] [--min <v>] [--max <v>]
//          [--scheme <s>] [--point-size <n>] [--alpha <a>] [-w|--overwrite]
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
          {"--alpha", "--cam-info", "--field", "--input", "--max", "--min", "--output",
           "--overwrite", "--pcd", "--point-size", "--resize", "--scheme", "--topic", "--undistort",
           "-i", "-o", "-t", "-w"}),
        current);
    }
  }

  auto complete_topic_after_flag =
    [&](const std::string_view & flag, const std::span<const std::string_view> & types) {
      if (request.cursor_word == 0 || request.words[request.cursor_word - 1] != flag) {
        return std::optional<std::vector<std::string>>{};
      }
      const auto bag_arg = find_flag_value(request, kInputFlags);
      if (!bag_arg || bag_arg->empty() || bag_arg->starts_with("-")) {
        return std::optional<std::vector<std::string>>{std::vector<std::string>{}};
      }
      return std::optional<std::vector<std::string>>{
        complete_topics(expand_current_user_home(*bag_arg), current, types)};
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
//   slam:   `map`(0) `slam`(1) -i|--input <bag> --pcd <topic> -o|--output <root>
//           [--backend <cpu|cuda|auto>] [--frame <frame_id>] [--imu <topic>]
//           [--gnss <topic>] [--cam <topic>...] [--cam-info <topic>...]
//           [--cam-min-dist <m>] [--cam-keyframe-blur]
//           [--input-res <m>] [--min-range <m>] [--max-range <m>]
//           [-j|--threads <N>] [--viewer] [-w|--overwrite]
//           [--no-progress] [--no-warmup-fill] [--no-cooldown-fill]
//           [--no-color-propagate] [--fill-min-inliers <f>] [--submap-keyframes <N>]
//           [--remove-outliers] [--outlier-r <m>] [--outlier-k <N>]
//           [--remove-dynamic] [--dynamic-res <m>] [--dynamic-ds <m>] [--dynamic-dp <N>]
//   viewer: `map`(0) `viewer`(1) -m|--map <map>
//
// At the action slot (word 1) the candidates are `slam` and `viewer` (or the
// help flags for a `-` word). Past it, the `--pcd` slot for `map slam` is
// completed earlier by try_topic_completion via kTopicBindings (PointCloud2
// topics only); here we surface `slam`'s flags for any `-` word and complete the
// values of `--imu` (Imu topics), `--cam` (image topics), and `--cam-info`
// (CameraInfo topics) from the bag. `viewer` has no value-bearing flags and its
// `--map` value is a path.

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
      return matching(with_help({"--map", "-m"}), current);
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
        {"--backend",
         "--cam",
         "--cam-info",
         "--cam-keyframe-blur",
         "--cam-min-dist",
         "--dynamic-dp",
         "--dynamic-ds",
         "--dynamic-res",
         "--fill-min-inliers",
         "--frame",
         "--gnss",
         "--imu",
         "--input",
         "--input-res",
         "--max-range",
         "--min-range",
         "--no-color-propagate",
         "--no-cooldown-fill",
         "--no-progress",
         "--no-warmup-fill",
         "--outlier-k",
         "--outlier-r",
         "--output",
         "--overwrite",
         "--pcd",
         "--remove-dynamic",
         "--remove-outliers",
         "--submap-keyframes",
         "--threads",
         "--viewer",
         "-i",
         "-j",
         "-o",
         "-w"}),
      current);
  }

  if (request.cursor_word > 0 && request.words[request.cursor_word - 1] == "--backend") {
    return matching({"auto", "cpu", "cuda"}, current);
  }

  // Topic-bearing flags: complete the value(s) from the bag's topics of the
  // type(s) the flag accepts. --imu takes exactly one value, so it completes
  // only immediately after the flag. --cam and --cam-info accept several
  // values per occurrence (CLI11 consumes every following non-flag word), so
  // the governing flag is found by walking left past the values already
  // typed; any other intervening flag ends that value run.
  if (request.cursor_word == 0) {
    return {};
  }
  std::span<const std::string_view> flag_topic_types;
  if (request.words[request.cursor_word - 1] == "--imu") {
    flag_topic_types = kImuType;
  } else {
    std::string_view governing;
    for (std::size_t w = request.cursor_word; w > kFirstCommandArgWord;) {
      --w;
      const auto & word = request.words[w];
      if (!word.empty() && word.front() == '-') {
        governing = word;
        break;
      }
    }
    if (governing == "--cam") {
      flag_topic_types = kImageTopicTypes;
    } else if (governing == "--cam-info") {
      flag_topic_types = kCameraInfoType;
    } else {
      return {};
    }
  }
  const auto bag_arg = find_flag_value(request, kInputFlags);
  if (!bag_arg || bag_arg->empty() || bag_arg->starts_with("-")) {
    return {};
  }
  return complete_topics(expand_current_user_home(*bag_arg), current, flag_topic_types);
}

// `ls -i|--input <bag>` lists topics. Its only flag is `-l/--long` (per-topic
// COUNT and HZ); <input> is a path that falls through to the shell's file
// completion. We surface `-i`/`--input` and `-l`/`--long` plus the implicit help
// flags for any `-` word.
//
//   ls: `ls`(0) -i|--input <bag> [-l|--long]
std::vector<std::string> complete_ls(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (current.starts_with("-")) {
    return matching(with_help({"--input", "--long", "-i", "-l"}), current);
  }
  return {};
}

// `trim -i|--input <bag>` copies only the messages inside a time window. All
// its flags are surfaced for any `-` word; <input> is a path that falls through
// to the shell's file completion. The value of `--align` is completed from the
// bag's topics (any type), mirroring `walk --cam-info`; `--stamp` completes its
// two clock choices.
//
//   trim: `trim`(0) -i|--input <bag>
//         {[--start <off>] [--end <off>|--duration <len>] | --both <off> |
//          --align <topics>...} [--stamp header|recv] [-o <out>] [-w]
std::vector<std::string> complete_trim(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (current.starts_with("-")) {
    return matching(
      with_help(
        {"--align", "--both", "--duration", "--end", "--input", "--output", "--overwrite",
         "--stamp", "--start", "-i", "-o", "-w"}),
      current);
  }

  if (request.cursor_word > 0 && request.words[request.cursor_word - 1] == "--stamp") {
    return matching({"header", "recv"}, current);
  }

  // Complete the value of `--align` from the bag's topic list. Bail out when
  // the -i/--input value is missing or holds a flag.
  if (request.cursor_word > 0 && request.words[request.cursor_word - 1] == "--align") {
    const auto bag_arg = find_flag_value(request, kInputFlags);
    if (!bag_arg || bag_arg->empty() || bag_arg->starts_with("-")) {
      return {};
    }
    return complete_topics(expand_current_user_home(*bag_arg), current, {});
  }
  return {};
}

// `walk -i|--input <bag> -t|--topic <topic>` walks a single topic's messages.
// Its topic is completed earlier by try_topic_completion via kTopicBindings
// (every topic in the bag). Here we surface walk's own `--cam-info` flag (plus
// the implicit help flags) for any `-` word, and complete the value of
// `--cam-info` from the bag's CameraInfo topics — mirroring `generate video
// --cam-info`.
//
//   walk: `walk`(0) -i|--input <bag> -t|--topic <topic> [--cam-info <topic>]
std::vector<std::string> complete_walk(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (current.starts_with("-")) {
    return matching(with_help({"--cam-info", "--input", "--topic", "-i", "-t"}), current);
  }

  // Complete the value of `--cam-info` from the bag's CameraInfo topics. Bail out
  // when the -i/--input value is missing or holds a flag.
  if (request.cursor_word > 0 && request.words[request.cursor_word - 1] == "--cam-info") {
    const auto bag_arg = find_flag_value(request, kInputFlags);
    if (!bag_arg || bag_arg->empty() || bag_arg->starts_with("-")) {
      return {};
    }
    return complete_topics(expand_current_user_home(*bag_arg), current, kCameraInfoType);
  }
  return {};
}

// `check` is a command group for rosbag integrity checks. Its sole subcommand is
// `broken`. At the subcommand slot (word 1) the only candidate is `broken` (or the
// implicit help flags for a `-` word). Past it, `broken`'s flags are surfaced for
// any `-` word; its `-i`/`--input` value is a path that falls through to the
// shell's file completion.
//
//   broken: `check`(0) `broken`(1) -i|--input <bag> [--rm] [--deep]
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
      return matching(with_help({"--deep", "--input", "--rm", "-i"}), current);
    }
  }
  return {};
}

// `cam-info` is a command group for sensor_msgs/msg/CameraInfo operations. Its
// subcommands are `replace`, `recompute-p`, and `dump`. At the subcommand slot
// (word 1) those are the candidates (or the implicit help flags for a `-`
// word).
//
// All three subcommands' topic values are completed earlier by
// try_topic_completion via kTopicBindings, so nothing in this function completes
// a topic value.
//
// <input> and <calib> are paths that fall through to the shell's file
// completion. `--frame-id`'s value is a free-form header override with nothing
// to suggest, `-o`/`--output`'s is an output path, and `-a`/`--alpha`'s is a
// free number in [0, 1], so none of those get value completion.
//
//   replace:     `cam-info`(0) `replace`(1) -i|--input <bag> --yaml <yaml>
//                -t|--topics <topic>... [--frame-id <id>] [-o <out>] [-w|--overwrite]
//   recompute-p: `cam-info`(0) `recompute-p`(1) -i|--input <bag>
//                [-t|--topics <topic>...] [-a|--alpha <a>] [-o <out>] [-w|--overwrite]
//   dump:        `cam-info`(0) `dump`(1) -i|--input <bag> -t|--topic <topic>
//                [-o <out>] [-w|--overwrite]
std::vector<std::string> complete_cam_info(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (request.cursor_word == kFirstCommandArgWord) {
    if (current.starts_with("-")) {
      return matching({kCommonHelpFlags.begin(), kCommonHelpFlags.end()}, current);
    }
    return matching({"replace", "recompute-p", "dump"}, current);
  }

  if (request.words.size() <= kFirstCommandArgWord) {
    return {};
  }
  const auto & sub = request.words[kFirstCommandArgWord];

  if (request.cursor_word >= kSecondCommandArgWord && current.starts_with("-")) {
    if (sub == "replace") {
      return matching(
        with_help(
          {"--frame-id", "--input", "--output", "--overwrite", "--topics", "--yaml", "-i", "-o",
           "-t", "-w"}),
        current);
    }
    if (sub == "recompute-p") {
      return matching(
        with_help(
          {"--alpha", "--input", "--output", "--overwrite", "--topics", "-a", "-i", "-o", "-t",
           "-w"}),
        current);
    }
    if (sub == "dump") {
      return matching(
        with_help({"--input", "--output", "--overwrite", "--topic", "-i", "-o", "-t", "-w"}),
        current);
    }
    return {};
  }

  return {};
}

// `pcd` is a command group for PointCloud2 topic processing. Its subcommands are
// `concat` and `undistort`. At the subcommand slot (word 1) the candidates are
// those two (or the implicit help flags for a `-` word). `-i`/`--input` names a
// path that falls through to the shell's file completion. Past the subcommand we
// surface each subcommand's own flags for any `-` word.
//
// For `concat`, `-t`/`--topic` names a free-form new topic name with nothing to
// suggest. PointCloud2 topic values complete for every `--pcd` value (read from
// the input bag). `--stamp-offset` takes one or more `<topic>=<value>` values
// per occurrence, so its `<topic>` half completes to the same PointCloud2 topics
// (as `<topic>=`) at every value in its run, until the cursor moves past `=`
// onto the `<value>` half. `--frame`, `--tolerance`, and `-o`/`--output` take
// free-form / numeric / path values, so they get no value completion.
//
//   concat: `pcd`(0) `concat`(1) -i|--input <bag> -t|--topic <output_topic>
//           --pcd <t...> [--frame <f>] [--tolerance <val>]
//           [--stamp-offset <t=v>...]... [-o <out>] [--drop-inputs] [--force]
//           [-j|--threads <N>] [-w|--overwrite]
//
// For `undistort`, `--pose` names a topic (accepted types are TFMessage /
// Odometry / PoseStamped / PoseWithCovarianceStamped) with nothing to suggest.
// `--pcd` is variadic and completes PointCloud2 topics from the input bag,
// mirroring concat's `--pcd`. `--ref`/`--of` complete the bag's TF frame ids,
// mirroring `traj dump`/`join`. `-o`/`--output` takes a path and `-j`/`--threads`
// takes a count, so they get no value completion.
//
//   undistort: `pcd`(0) `undistort`(1) -i|--input <bag> --pose <topic>
//              --pcd <t...> [--ref <frame>] [--of <frame>] [-o <out>]
//              [-j|--threads <N>] [-w|--overwrite]
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
          {"--drop-inputs", "--force", "--frame", "--input", "--output", "--overwrite", "--pcd",
           "--stamp-offset", "--threads", "--tolerance", "--topic", "-i", "-j", "-o", "-t", "-w"}),
        current);
    }
    if (sub == "undistort") {
      return matching(
        with_help(
          {"--input", "--of", "--output", "--overwrite", "--pcd", "--pose", "--ref", "--threads",
           "-i", "-j", "-o", "-w"}),
        current);
    }
  }

  // --stamp-offset consumes one or more <topic>=<value> values per occurrence
  // (the CLI option is a vector with no arity limit, like --pcd), so complete
  // the <topic> half for every value in its run, not just the word immediately
  // after the flag. Walk back from the cursor over the values already given
  // (bash splits a typed value at '=', and the resulting `topic`/`=`/`value`
  // fragments are all skipped here as non-option words); if the nearest option
  // word is --stamp-offset, the cursor is still consuming its values. Each
  // candidate carries a trailing '=' so the shell scripts drop the auto-space
  // and leave the cursor on the value.
  //
  // The <value> half (a duration) has nothing to suggest. zsh/fish keep a
  // typed topic=value unsplit, so a value in progress shows up as a current
  // word already containing '='; bash splits it, leaving a bare '=' as the
  // word right before the cursor. Both cases return no candidates.
  if (request.cursor_word > kFirstCommandArgWord && current.find('=') == std::string::npos) {
    const bool after_equals = request.words[request.cursor_word - 1] == "=";
    for (std::size_t w = request.cursor_word; w > kFirstCommandArgWord;) {
      const auto & word = request.words[--w];
      if (!word.starts_with("-")) {
        continue;  // a value already given to --stamp-offset; keep scanning
      }
      if (word == "--stamp-offset") {
        if (after_equals) {
          return {};  // on the <value> half (bash split at '='); nothing to offer
        }
        const auto bag_arg = find_flag_value(request, kInputFlags);
        if (bag_arg && !bag_arg->empty() && !bag_arg->starts_with("-")) {
          auto topics =
            complete_topics(expand_current_user_home(*bag_arg), current, kPointCloud2Type);
          for (auto & topic : topics) {
            topic += '=';
          }
          return topics;
        }
      }
      break;  // the nearest option decides; a non-stamp-offset option ends the run
    }
  }

  // --pcd is variadic: complete PointCloud2 topics for every value in its run,
  // not just the first. Walk back from the cursor to the nearest option word; if
  // it is --pcd, the cursor is still consuming its values, so offer topics from
  // the input bag.
  for (std::size_t w = request.cursor_word; w > kFirstCommandArgWord;) {
    const auto & word = request.words[--w];
    if (!word.starts_with("-")) {
      continue;  // a topic value already given to --pcd; keep scanning
    }
    if (word == "--pcd") {
      const auto bag_arg = find_flag_value(request, kInputFlags);
      if (bag_arg && !bag_arg->empty() && !bag_arg->starts_with("-")) {
        return complete_topics(expand_current_user_home(*bag_arg), current, kPointCloud2Type);
      }
    }
    break;  // the nearest option decides; a non-pcd option ends the run
  }

  // undistort's --of/--ref complete the bag's TF frame ids, mirroring
  // `traj dump`/`join`.
  if (request.cursor_word > 0) {
    const auto & previous = request.words[request.cursor_word - 1];
    if (previous == "--of" || previous == "--ref") {
      const auto bag_arg = find_flag_value(request, kInputFlags);
      if (!bag_arg || bag_arg->empty() || bag_arg->starts_with("-")) {
        return {};
      }
      return complete_frame_id_value(*bag_arg, current);
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
  if (command == "trim") {
    return complete_trim(request);
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
    app.add_option("--shell", shell_, "Shell to generate completions for")
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
