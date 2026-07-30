// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/parse_error.hpp"

#include "CLI/CLI.hpp"

#include <gtest/gtest.h>

#include <string>

namespace
{

using bagwiz::commands::rewrite_parse_error;

TEST(ParseError, RequiredOptionWithShortFormShowsBoth)
{
  CLI::App app;
  std::string sink;
  app.add_option("-i,--input", sink, "Bag path")->required();

  try {
    app.parse(std::vector<std::string>{"prog"});
    FAIL() << "parse should throw RequiredError";
  } catch (const CLI::RequiredError & e) {
    EXPECT_EQ(rewrite_parse_error(app, e), "-i/--input is required");
  }
}

TEST(ParseError, RequiredOptionWithoutShortFormIsUnchanged)
{
  CLI::App app;
  std::string sink;
  app.add_option("--input", sink, "Bag path")->required();

  try {
    app.parse(std::vector<std::string>{"prog"});
    FAIL() << "parse should throw RequiredError";
  } catch (const CLI::RequiredError & e) {
    EXPECT_EQ(rewrite_parse_error(app, e), "--input is required");
  }
}

TEST(ParseError, RequiredOptionWithOnlyShortFormIsUnchanged)
{
  CLI::App app;
  std::string sink;
  app.add_option("-i", sink, "Bag path")->required();

  try {
    app.parse(std::vector<std::string>{"prog"});
    FAIL() << "parse should throw RequiredError";
  } catch (const CLI::RequiredError & e) {
    EXPECT_EQ(rewrite_parse_error(app, e), "-i is required");
  }
}

TEST(ParseError, SubcommandRequiredIsUnchanged)
{
  CLI::App app;
  app.add_subcommand("sub", "A subcommand");
  app.require_subcommand(1);

  try {
    app.parse(std::vector<std::string>{"prog"});
    FAIL() << "parse should throw RequiredError";
  } catch (const CLI::RequiredError & e) {
    EXPECT_EQ(rewrite_parse_error(app, e), "A subcommand is required");
  }
}

TEST(ParseError, RequiresErrorShowsBothForms)
{
  CLI::App app;
  std::string foo;
  std::string bar;
  CLI::Option * foo_opt = app.add_option("-f,--foo", foo);
  app.add_option("-b,--bar", bar)->needs(foo_opt);

  try {
    app.parse(std::vector<std::string>{"prog", "--bar", "x"});
    FAIL() << "parse should throw RequiresError";
  } catch (const CLI::RequiresError & e) {
    EXPECT_EQ(rewrite_parse_error(app, e), "-b/--bar requires -f/--foo");
  }
}

TEST(ParseError, ExcludesErrorShowsBothForms)
{
  CLI::App app;
  bool foo = false;
  bool bar = false;
  CLI::Option * foo_opt = app.add_flag("-f,--foo", foo);
  app.add_flag("-b,--bar", bar)->excludes(foo_opt);

  try {
    app.parse(std::vector<std::string>{"prog", "--foo", "--bar"});
    FAIL() << "parse should throw ExcludesError";
  } catch (const CLI::ExcludesError & e) {
    EXPECT_EQ(rewrite_parse_error(app, e), "-f/--foo excludes -b/--bar");
  }
}

TEST(ParseError, RewritesOptionNamesInNestedSubcommands)
{
  CLI::App app;
  app.require_subcommand(1);
  CLI::App * map = app.add_subcommand("map", "map cmd");
  CLI::App * slam = map->add_subcommand("slam", "slam cmd");
  map->require_subcommand(1);
  std::string sink;
  slam->add_option("-i,--input", sink, "Bag path")->required();

  try {
    // CLI11's string parse overload handles nested named subcommands
    // ("map slam") the same way the production binary does with argc/argv.
    app.parse("prog map slam");
    FAIL() << "parse should throw RequiredError";
  } catch (const CLI::RequiredError & e) {
    EXPECT_EQ(rewrite_parse_error(app, e), "-i/--input is required");
  }
}

TEST(ParseError, RewrittenMessageStartsWithShortLongForm)
{
  CLI::App app;
  std::string sink;
  app.add_option("-i,--input", sink, "Bag path")->required();

  try {
    app.parse(std::vector<std::string>{"prog"});
    FAIL() << "parse should throw RequiredError";
  } catch (const CLI::RequiredError & e) {
    const std::string rewritten = rewrite_parse_error(app, e);
    EXPECT_EQ(rewritten, "-i/--input is required");
  }
}

}  // namespace
