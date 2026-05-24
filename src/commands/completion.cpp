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
#include "bagwiz/io/bag_io.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{

constexpr std::string_view kCompletionCommand = "__complete";
constexpr int kMinimumCompletionProbeArgc = 2;
constexpr int kCompletionCommandArg = 1;
constexpr int kMinimumCompletionArgc = 3;
constexpr int kCursorWordArg = 2;
constexpr int kCompletionWordsBeginArg = 3;
constexpr std::size_t kTopLevelCommandWord = 0;
constexpr std::size_t kFirstCommandArgWord = 1;
constexpr std::size_t kSecondCommandArgWord = 2;
constexpr std::size_t kFourthCommandArgWord = 4;

// Declarative table of commands that take a positional <topic> argument.
// `subcommand` is empty when the command has no subcommand level (e.g.
// `bagwiz walk <input> <topic>`). `input_word` and `topic_word` are
// indices into CompletionRequest::words AFTER the leading "bagwiz" has
// been stripped, so position 0 is the top-level command.
struct TopicArgBinding
{
  std::string_view command{};
  std::string_view subcommand{};
  std::size_t input_word{0};
  std::size_t topic_word{0};
};

constexpr std::array<TopicArgBinding, 3> kTopicBindings{{
  {"walk", "", kFirstCommandArgWord, kSecondCommandArgWord},
  {"traj", "dump", kSecondCommandArgWord, kFourthCommandArgWord},
  {"traj", "join", kSecondCommandArgWord, kFourthCommandArgWord},
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
  const std::filesystem::path & target, const std::string_view & contents, bool force)
{
  std::error_code ec;
  if (std::filesystem::exists(target, ec) && !force) {
    std::cerr << "refusing to overwrite existing file: " << target
              << " (pass --force to overwrite)\n";
    return false;
  }

  const auto parent = target.parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent, ec);
    if (ec) {
      std::cerr << "failed to create directory " << parent << ": " << ec.message() << '\n';
      return false;
    }
  }

  std::ofstream stream(target, std::ios::trunc);
  if (!stream) {
    std::cerr << "failed to open " << target << " for writing\n";
    return false;
  }
  stream << contents;
  if (!stream) {
    std::cerr << "failed to write completion script to " << target << '\n';
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

std::vector<std::string> top_level_candidates(const std::string_view & prefix)
{
  std::vector<std::string> result;
  for (const auto & cmd : Registry::instance().all()) {
    if (starts_with(cmd->name(), prefix)) {
      result.emplace_back(cmd->name());
    }
  }
  std::sort(result.begin(), result.end());
  return result;
}

std::vector<std::string> complete_topics(
  const std::filesystem::path & input_path, const std::string_view & prefix)
{
  std::vector<std::string> result;
  try {
    const auto reader = io::open_read(expand_current_user_home(input_path));
    for (const auto & topic : reader->topics()) {
      if (starts_with(topic.name, prefix)) {
        result.push_back(topic.name);
      }
    }
  } catch (const std::exception &) {
    return {};
  }

  std::sort(result.begin(), result.end());
  return result;
}

// Looks up the cursor position in kTopicBindings and, if a binding matches
// and every positional slot before the topic is non-flag, dispatches to
// complete_topics. Returns std::nullopt when no binding applies so the
// caller can fall through to per-command completion. Returning an empty
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

  const auto & top_level = request.words[kTopLevelCommandWord];

  for (const auto & binding : kTopicBindings) {
    if (binding.command != top_level) {
      continue;
    }
    if (!binding.subcommand.empty()) {
      if (request.words.size() <= kFirstCommandArgWord) {
        continue;
      }
      if (request.words[kFirstCommandArgWord] != binding.subcommand) {
        continue;
      }
    }
    if (request.cursor_word != binding.topic_word) {
      continue;
    }

    bool earlier_slot_is_flag = false;
    for (std::size_t i = kFirstCommandArgWord; i < binding.topic_word; ++i) {
      if (request.words[i].starts_with("-")) {
        earlier_slot_is_flag = true;
        break;
      }
    }
    if (earlier_slot_is_flag) {
      continue;
    }

    return complete_topics(request.words[binding.input_word], current);
  }

  return std::nullopt;
}

std::vector<std::string> complete_complete_command(const CompletionRequest & request)
{
  if (request.cursor_word == kFirstCommandArgWord) {
    return matching(supported_shell_names(), current_word(request));
  }
  return {};
}

std::vector<std::string> complete_convert(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (request.cursor_word == kFirstCommandArgWord) {
    return matching({"1to2", "2to1", "storage"}, current);
  }

  if (request.cursor_word >= kSecondCommandArgWord && current.starts_with("-")) {
    const auto & mode = request.words[kFirstCommandArgWord];
    if (mode == "1to2") {
      return matching(
        {"--allow-md5-mismatch", "--overwrite", "--storage", "--strict", "-s"}, current);
    }
    if (mode == "2to1") {
      return matching({"--overwrite", "--strict"}, current);
    }
    if (mode == "storage") {
      return matching({"--overwrite", "--storage", "-s"}, current);
    }
  }

  if (
    request.cursor_word > 0 && (request.words[request.cursor_word - 1] == "--storage" ||
                                request.words[request.cursor_word - 1] == "-s")) {
    return matching({"mcap", "sqlite3"}, current);
  }
  return {};
}

std::vector<std::string> complete_traj(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (request.cursor_word == kFirstCommandArgWord) {
    return matching({"dump", "join"}, current);
  }

  if (request.cursor_word >= kSecondCommandArgWord && current.starts_with("-")) {
    const auto & mode = request.words[kFirstCommandArgWord];
    if (mode == "dump") {
      return matching({"--format", "--from", "--overwrite", "--to", "-f"}, current);
    }
    if (mode == "join") {
      return matching(
        {"--force", "--format", "--from", "--msg-type", "--output", "--overwrite", "--to", "-f",
         "-o", "-t"},
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
  }
  return {};
}

std::vector<std::string> complete_tf(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (request.cursor_word == kFirstCommandArgWord) {
    return matching({"inject-static", "tree", "walk"}, current);
  }

  if (request.cursor_word >= kSecondCommandArgWord && current.starts_with("-")) {
    const auto & mode = request.words[kFirstCommandArgWord];
    if (mode == "walk") {
      return matching({"--rot", "-r"}, current);
    }
    if (mode == "inject-static") {
      return matching({"--force", "--output", "--overwrite", "-o"}, current);
    }
  }

  if (request.cursor_word > 0) {
    const auto & previous = request.words[request.cursor_word - 1];
    if (previous == "--rot" || previous == "-r") {
      return matching({"euler", "euler_deg", "euler_rad", "quat"}, current);
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
    app.add_flag("--force", force_, "Overwrite an existing file when used with --install");
  }

  int run() override
  {
    const auto shell = parse_shell(shell_);
    if (!shell) {
      std::cerr << "unsupported shell: " << shell_ << '\n';
      return 1;
    }

    if (!install_) {
      std::cout << completion_script(*shell);
      return 0;
    }

    const auto target = install_path_for(*shell);
    if (!target) {
      std::cerr << "cannot determine install path: HOME is not set\n";
      return 1;
    }

    if (!write_script_to(*target, completion_script(*shell), force_)) {
      return 1;
    }
    std::cout << "installed: " << target->string() << '\n';
    return 0;
  }

private:
  std::string shell_;
  bool install_ = false;
  bool force_ = false;
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

bool install_completion_script(
  const std::string_view & shell, const std::filesystem::path & target, bool force)
{
  const auto parsed = parse_shell(shell);
  if (!parsed) {
    std::cerr << "unsupported shell: " << shell << '\n';
    return false;
  }
  return write_script_to(target, completion_script(*parsed), force);
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
