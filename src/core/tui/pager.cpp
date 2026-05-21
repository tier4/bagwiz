// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tui/pager.hpp"

#include "bagwiz/core/terminal_input.hpp"
#include "bagwiz/core/tui/internal/signal_handler.hpp"
#include "bagwiz/core/tui/layout.hpp"
#include "bagwiz/core/tui/renderer.hpp"
#include "bagwiz/core/tui/screen_scope.hpp"

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <string>
#include <utility>

namespace bagwiz::core::tui
{

namespace
{
// Module-level pointer to the TerminalRawMode owned by the active
// run() invocation so with_line_input() can find it without growing
// the public API surface. Only one pager runs at a time.
TerminalRawMode * g_active_raw_mode = nullptr;

// RAII guard: publishes a TerminalRawMode pointer to g_active_raw_mode
// for the lifetime of the scope and clears it on destruction. Without
// this, an exception escaping the pager loop would leave the global
// pointing at a destroyed stack object.
class ActiveRawModeScope
{
public:
  explicit ActiveRawModeScope(TerminalRawMode * raw_mode) noexcept { g_active_raw_mode = raw_mode; }
  ~ActiveRawModeScope() { g_active_raw_mode = nullptr; }
  ActiveRawModeScope(const ActiveRawModeScope &) = delete;
  ActiveRawModeScope & operator=(const ActiveRawModeScope &) = delete;
  ActiveRawModeScope(ActiveRawModeScope &&) = delete;
  ActiveRawModeScope & operator=(ActiveRawModeScope &&) = delete;
};
}  // namespace

NavKey to_nav_key(KeyEvent ev) noexcept
{
  switch (ev) {
    case KeyEvent::kNext:
      return NavKey::kNext;
    case KeyEvent::kPrev:
      return NavKey::kPrev;
    case KeyEvent::kFirst:
      return NavKey::kFirst;
    case KeyEvent::kLast:
      return NavKey::kLast;
    case KeyEvent::kScrollUp:
      return NavKey::kScrollUp;
    case KeyEvent::kScrollDown:
      return NavKey::kScrollDown;
    case KeyEvent::kScrollHead:
      return NavKey::kScrollHead;
    case KeyEvent::kScrollTail:
      return NavKey::kScrollTail;
    case KeyEvent::kQuit:
      return NavKey::kQuit;
    case KeyEvent::kResize:
      return NavKey::kResize;
    case KeyEvent::kSaveYaml:
    case KeyEvent::kToggleArrayExpand:
    case KeyEvent::kUnknown:
      return NavKey::kNone;
  }
  return NavKey::kNone;
}

ScrollablePager::ScrollablePager(PagerConfig cfg)
: cfg_(cfg), out_(cfg.out != nullptr ? cfg.out : &std::cout)
{
}

void ScrollablePager::render_frame(const Frame & frame)
{
  const Size sz = query_terminal_size();

  // Take the header / footer row counts directly from the frame so the
  // caller can pre-wrap to terminal width. Clamp the footer if the
  // header alone already covers the whole screen.
  const int header_rows = std::min(static_cast<int>(frame.header.size()), std::max(0, sz.rows));
  const int footer_rows =
    std::min(static_cast<int>(frame.footer.size()), std::max(0, sz.rows - header_rows));

  const FixedLayout lay{sz, header_rows, footer_rows};

  for (int i = 0; i < header_rows; ++i) {
    draw_line(*out_, lay.header_start_row() + i, frame.header[i], sz.cols);
  }

  // Body: clamp the scroll offset and emit body_rows() lines, padding
  // with empty lines so the trailing tail of a long previous message
  // does not leak through.
  const int body_rows = lay.body_rows();
  const std::size_t total_body = frame.body.size();
  const std::size_t max_scroll = total_body > static_cast<std::size_t>(body_rows)
                                   ? total_body - static_cast<std::size_t>(body_rows)
                                   : 0;
  if (scroll_ > max_scroll) {
    scroll_ = max_scroll;
  }
  for (int i = 0; i < body_rows; ++i) {
    const std::size_t src_idx = scroll_ + static_cast<std::size_t>(i);
    const std::string & line = (src_idx < total_body) ? frame.body[src_idx] : std::string{};
    draw_line(*out_, lay.body_start_row() + i, line, sz.cols);
  }

  for (int i = 0; i < footer_rows; ++i) {
    draw_line(*out_, lay.footer_start_row() + i, frame.footer[i], sz.cols);
  }

  // Erase any rows still showing content from a taller previous frame
  // (terminal shrank, or header / footer wrapped to fewer lines).
  const int drawn_rows = header_rows + body_rows + footer_rows;
  for (int row = drawn_rows + 1; row <= last_drawn_rows_; ++row) {
    draw_line(*out_, row, std::string{}, sz.cols);
  }
  last_drawn_rows_ = drawn_rows;

  out_->flush();
}

int ScrollablePager::run(
  const BuildFrame & build_frame, const OnNav & on_nav, const OnAppKey & on_app_key)
{
  TerminalRawMode raw_mode;
  if (!raw_mode.active()) {
    return 1;
  }
  ActiveRawModeScope active_raw_mode_scope(&raw_mode);

  ScreenScopeConfig sc{};
  sc.hide_cursor = cfg_.hide_cursor;
  sc.disable_autowrap = cfg_.disable_autowrap;
  sc.use_alt_screen = cfg_.use_alt_screen;
  sc.out = out_;
  ScreenScope screen(sc);
  internal::SigwinchScope sigwinch;

  needs_redraw_ = true;
  last_drawn_rows_ = 0;

  while (true) {
    if (needs_redraw_) {
      const Size sz = query_terminal_size();
      Frame frame = build_frame(scroll_, sz);
      render_frame(frame);
      needs_redraw_ = false;
    }

    const KeyEvent ev = read_key_event();
    const NavKey nav = to_nav_key(ev);

    // Generic scroll keys handled by the pager itself.
    if (nav == NavKey::kScrollUp) {
      if (scroll_ > 0) {
        --scroll_;
      }
      needs_redraw_ = true;
      continue;
    }
    if (nav == NavKey::kScrollDown) {
      ++scroll_;  // clamped during render
      needs_redraw_ = true;
      continue;
    }
    if (nav == NavKey::kScrollHead) {
      scroll_ = 0;
      needs_redraw_ = true;
      continue;
    }
    if (nav == NavKey::kScrollTail) {
      scroll_ = static_cast<std::size_t>(-1);  // clamped during render
      needs_redraw_ = true;
      continue;
    }

    // Resize: always redraw, and let the app observe it.
    if (nav == NavKey::kResize) {
      needs_redraw_ = true;
      if (on_nav) {
        const AppKeyResult r = on_nav(nav);
        if (r == AppKeyResult::kQuit) {
          break;
        }
      }
      continue;
    }

    if (nav == NavKey::kQuit) {
      break;
    }

    // Item navigation forwarded to the app.
    if (nav != NavKey::kNone) {
      if (on_nav) {
        const AppKeyResult r = on_nav(nav);
        if (r == AppKeyResult::kQuit) {
          break;
        }
        if (r == AppKeyResult::kHandled) {
          needs_redraw_ = true;
        }
      }
      continue;
    }

    // App-specific event.
    if (on_app_key) {
      const AppKeyResult r = on_app_key(ev);
      if (r == AppKeyResult::kQuit) {
        break;
      }
      if (r == AppKeyResult::kHandled) {
        needs_redraw_ = true;
      }
    }
  }

  return 0;
}

void ScrollablePager::with_line_input(
  const std::function<void(std::istream &, std::ostream &)> & body)
{
  if (g_active_raw_mode == nullptr) {
    // Called outside run(); just invoke the body with no terminal
    // manipulation rather than corrupting state.
    if (body) {
      body(std::cin, *out_);
    }
    needs_redraw_ = true;
    return;
  }

  // Park the cursor below the rendered viewport so the prompt shows
  // beneath the footer. A full \x1B[2J + redraw follows on return.
  const Size sz = query_terminal_size();
  move_cursor(*out_, sz.rows, 1);
  *out_ << '\n';
  show_cursor(*out_);
  set_autowrap(*out_, true);
  out_->flush();
  g_active_raw_mode->suspend_for_line_input();

  if (body) {
    body(std::cin, *out_);
  }

  g_active_raw_mode->resume_after_line_input();
  // Force a full-screen redraw on the next iteration so the prompt
  // and any echoed text vanish cleanly.
  *out_ << "\x1B[2J";
  if (cfg_.hide_cursor) {
    hide_cursor(*out_);
  }
  if (cfg_.disable_autowrap) {
    set_autowrap(*out_, false);
  }
  out_->flush();
  needs_redraw_ = true;
}

}  // namespace bagwiz::core::tui
