// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__TUI__PAGER_HPP_
#define BAGWIZ__CORE__TUI__PAGER_HPP_

#include "bagwiz/core/base/terminal_input.hpp"
#include "bagwiz/core/tui/layout.hpp"

#include <cstddef>
#include <functional>
#include <istream>
#include <ostream>
#include <string>
#include <vector>

namespace bagwiz::core::tui
{

// Navigation keys recognised by the pager. App-specific events are
// reported through the app-key callback instead.
enum class NavKey {
  kNone,             // app-specific event; forward to OnAppKey
  kNext,             // advance one item
  kPrev,             // back one item
  kFirst,            // jump to first item
  kLast,             // jump to last item
  kStepForward1s,    // jump to the next item at least one second ahead
  kStepBackward1s,   // jump to the previous item at least one second behind
  kStepForward10s,   // jump ~10 seconds ahead ('>')
  kStepBackward10s,  // jump ~10 seconds behind ('<')
  kScrollUp,         // scroll body up by one line
  kScrollDown,       // scroll body down by one line
  kScrollHead,       // jump body to first line
  kScrollTail,       // jump body to last line
  kQuit,             // exit the pager
  kResize,           // terminal was resized
};

// Pure mapping from KeyEvent to NavKey. App-specific events return
// kNone so the caller forwards them.
NavKey to_nav_key(KeyEvent ev) noexcept;

// One frame's content. Lines are plain strings; the pager handles
// width-aware truncation and per-row erase.
struct Frame
{
  std::vector<std::string> header;
  std::vector<std::string> body;
  std::vector<std::string> footer;
};

struct PagerConfig
{
  bool use_alt_screen = false;
  bool disable_autowrap = true;
  bool hide_cursor = true;
  // Defaults to &std::cout when null.
  std::ostream * out = nullptr;
};

// Return value for app callbacks instructing the pager what to do.
enum class AppKeyResult {
  kHandled,  // state changed; redraw on the next frame
  kIgnored,  // no state change; do not redraw
  kQuit,     // exit the pager loop
};

// ScrollablePager orchestrates the render+input loop. Apps provide
// three callbacks:
//   * BuildFrame: produce the next Frame given the current scroll
//     offset and the terminal size. The number of header / footer
//     rows drawn is taken from `frame.header.size()` and
//     `frame.footer.size()`, so the app can pre-wrap content to the
//     terminal width and the pager will adapt the body region around
//     whatever the frame reports.
//   * OnNav: react to navigation NavKeys (next/prev/first/last). The
//     pager handles kScroll* and kQuit itself but reports kResize via
//     this callback for visibility / state-bookkeeping.
//   * OnAppKey: invoked for KeyEvents that to_nav_key() maps to kNone
//     (e.g. kSaveYaml, kToggleArrayExpand). The callback may call
//     with_line_input() on `*this`.
//
// The pager owns: scroll offset, terminal scope (cursor hide /
// autowrap / alt-screen), and the SIGWINCH flag.
class ScrollablePager
{
public:
  using BuildFrame = std::function<Frame(std::size_t scroll_offset, Size term_size)>;
  using OnNav = std::function<AppKeyResult(NavKey nav)>;
  using OnAppKey = std::function<AppKeyResult(KeyEvent ev)>;

  explicit ScrollablePager(PagerConfig cfg = {});

  // Run the loop until OnNav / OnAppKey returns kQuit, the user presses
  // kQuit, or read_key_event returns kQuit on EOF. Returns 0.
  int run(const BuildFrame & build_frame, const OnNav & on_nav, const OnAppKey & on_app_key);

  // Temporarily yield the terminal to a cooked-mode line-input scope.
  // body() runs with std::cin / std::cout (or the pager's ostream)
  // configured as a normal line-buffered TTY. On return the pager
  // re-applies its terminal scope and forces a full redraw.
  void with_line_input(const std::function<void(std::istream &, std::ostream &)> & body);

  [[nodiscard]] std::size_t scroll_offset() const noexcept { return scroll_; }
  void set_scroll_offset(std::size_t v) noexcept { scroll_ = v; }

  // Mark the next iteration as needing a redraw. Equivalent to
  // returning kHandled from a callback.
  void request_redraw() noexcept { needs_redraw_ = true; }

private:
  void render_frame(const Frame & frame);

  PagerConfig cfg_;
  std::ostream * out_;
  std::size_t scroll_ = 0;
  bool needs_redraw_ = true;
  // Total rows occupied by the most recently rendered frame; used to
  // erase orphan rows when the layout shrinks (e.g. terminal resize or
  // header/footer wraps to fewer lines than the previous frame).
  int last_drawn_rows_ = 0;
};

}  // namespace bagwiz::core::tui

#endif  // BAGWIZ__CORE__TUI__PAGER_HPP_
