// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__TUI__SCREEN_SCOPE_HPP_
#define BAGWIZ__CORE__TUI__SCREEN_SCOPE_HPP_

#include <ostream>

namespace bagwiz::core::tui
{

struct ScreenScopeConfig
{
  bool hide_cursor = true;
  bool disable_autowrap = true;
  bool use_alt_screen = false;
  // When null, defaults to &std::cout.
  std::ostream * out = nullptr;
};

// RAII guard that applies the requested terminal-state changes on
// construction and restores them in reverse order on destruction:
//   * (optional) enter the xterm alternate screen buffer
//   * (optional) hide the cursor
//   * (optional) disable autowrap
//
// The destructor moves the cursor to (size.rows, 1) before restoring
// state when alt-screen is OFF so the next shell prompt does not land
// on top of the last rendered footer line.
//
// Safe to construct when stdout is not a TTY; the ostream is still
// written to (callers can capture in tests), but the underlying
// terminal state is unaffected because the byte stream is read as
// plain text downstream.
class ScreenScope
{
public:
  explicit ScreenScope(ScreenScopeConfig cfg);
  ~ScreenScope();

  ScreenScope(const ScreenScope &) = delete;
  ScreenScope & operator=(const ScreenScope &) = delete;
  ScreenScope(ScreenScope &&) = delete;
  ScreenScope & operator=(ScreenScope &&) = delete;

  [[nodiscard]] std::ostream & out() const noexcept { return *out_; }

  // Temporarily restore terminal state for a line-input scope:
  // re-show the cursor, re-enable autowrap, and leave the alt screen if
  // applicable. Pair every call with resume() before the next render.
  void suspend();
  void resume();

private:
  void apply();
  void restore();

  ScreenScopeConfig cfg_;
  std::ostream * out_;
  bool active_ = false;
};

}  // namespace bagwiz::core::tui

#endif  // BAGWIZ__CORE__TUI__SCREEN_SCOPE_HPP_
