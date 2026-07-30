// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/parse_error.hpp"

#include <cctype>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace bagwiz::commands
{
namespace
{

// Build a map from every short/long name of every option in the App tree to
// that option's preferred display name. The display name uses the first short
// and first long name joined as "-s/--long"; if only one form exists, just
// that form is used.
void collect_option_display_names(
  const CLI::App & app, std::unordered_map<std::string, std::string> & display_names)
{
  for (const CLI::Option * opt : app.get_options()) {
    const std::vector<std::string> & snames = opt->get_snames();
    const std::vector<std::string> & lnames = opt->get_lnames();
    if (snames.empty() && lnames.empty()) {
      continue;
    }

    std::string display;
    if (!snames.empty() && !lnames.empty()) {
      display = "-" + snames.front() + "/--" + lnames.front();
    } else if (!snames.empty()) {
      display = "-" + snames.front();
    } else {
      display = "--" + lnames.front();
    }

    for (const std::string & sname : snames) {
      display_names["-" + sname] = display;
    }
    for (const std::string & lname : lnames) {
      display_names["--" + lname] = display;
    }
  }

  for (const CLI::App * sub : app.get_subcommands()) {
    collect_option_display_names(*sub, display_names);
  }
}

// True if `s[pos]` begins an option-name token: -X or --X where X is a letter.
bool is_option_start(std::string_view s, std::size_t pos)
{
  if (pos >= s.size() || s[pos] != '-') {
    return false;
  }
  if (pos + 1 >= s.size()) {
    return false;
  }
  const unsigned char first = static_cast<unsigned char>(s[pos + 1]);
  if (s[pos + 1] == '-') {
    return pos + 2 < s.size() && std::isalpha(static_cast<unsigned char>(s[pos + 2]));
  }
  return std::isalpha(first);
}

// Return the index one past the end of the option-name token starting at pos.
// Valid CLI11 names contain letters, digits, '-', and '_'.
std::size_t option_token_end(std::string_view s, std::size_t pos)
{
  // Consume one or two leading dashes.
  std::size_t end = pos;
  while (end < s.size() && s[end] == '-' && end - pos < 2) {
    ++end;
  }
  while (end < s.size() &&
         (std::isalnum(static_cast<unsigned char>(s[end])) || s[end] == '-' || s[end] == '_')) {
    ++end;
  }
  return end;
}

}  // namespace

std::string rewrite_parse_error(const CLI::App & app, const CLI::Error & error)
{
  std::unordered_map<std::string, std::string> display_names;
  collect_option_display_names(app, display_names);
  if (display_names.empty()) {
    return std::string(error.what());
  }

  const std::string_view message = error.what();
  std::string result;
  result.reserve(message.size());

  std::size_t i = 0;
  while (i < message.size()) {
    if (is_option_start(message, i)) {
      const std::size_t end = option_token_end(message, i);
      const std::string token(message.substr(i, end - i));
      const auto it = display_names.find(token);
      if (it != display_names.end()) {
        result += it->second;
      } else {
        result += token;
      }
      i = end;
    } else {
      result.push_back(message[i]);
      ++i;
    }
  }

  return result;
}

}  // namespace bagwiz::commands
