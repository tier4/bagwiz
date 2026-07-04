// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/terminal_input.hpp"

#include "bagwiz/core/tui/internal/signal_handler.hpp"

#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <string_view>

namespace bagwiz::core
{

namespace
{

termios make_raw_settings(const termios & saved)
{
  termios raw = saved;
  raw.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO | ISIG);
  raw.c_cc[VMIN] = 1;
  raw.c_cc[VTIME] = 0;
  return raw;
}

// Milliseconds to wait for a continuation byte after reading ESC. Short
// enough that a lone ESC press feels immediate, long enough that a
// terminal-emitted "ESC [ C" arrives as one sequence on a slow TTY.
constexpr int kEscFollowupPollMs = 50;

// Blocking read of a single byte from stdin. Returns -1 on EOF or
// genuine error; returns -2 when a resize was observed via SIGWINCH so
// the caller can surface kResize instead of treating it as quit.
constexpr int kReadEofOrError = -1;
constexpr int kReadResizeInterrupt = -2;

int read_byte()
{
  while (true) {
    // Surface a pending resize immediately instead of waiting for read() to be
    // interrupted by SIGWINCH. This catches resizes that occurred while we were
    // busy (e.g. rendering) as well as those that interrupted the previous read.
    if (tui::internal::consume_resize_flag()) {
      return kReadResizeInterrupt;
    }

    char c = 0;
    const ssize_t n = ::read(STDIN_FILENO, &c, 1);
    if (n > 0) {
      return static_cast<unsigned char>(c);
    }
    if (n == 0) {
      return kReadEofOrError;  // EOF
    }
    // n < 0
    if (errno == EINTR) {
      // Loop and let the consume_resize_flag() check at the top handle it.
      continue;
    }
    return kReadEofOrError;
  }
}

// Non-blocking read of up to n bytes with a poll() timeout. Returns the
// number of bytes actually read (0 on timeout, -1 on error).
ssize_t read_available(char * buf, size_t n, int timeout_ms)
{
  pollfd pfd{};
  pfd.fd = STDIN_FILENO;
  pfd.events = POLLIN;
  const int r = ::poll(&pfd, 1, timeout_ms);
  if (r <= 0) {
    return r;
  }
  return ::read(STDIN_FILENO, buf, n);
}

}  // namespace

KeyEvent classify_key(std::string_view bytes)
{
  if (bytes.empty()) {
    return KeyEvent::kUnknown;
  }

  if (bytes.size() == 1) {
    const unsigned char c = static_cast<unsigned char>(bytes[0]);
    switch (c) {
      case 0x1B:  // lone ESC
      case 0x03:  // Ctrl-C
      case 0x04:  // Ctrl-D
      case 'q':
      case 'Q':
        return KeyEvent::kQuit;
      case ' ':
        return KeyEvent::kNext;
      case '\r':
      case '\n':
        return KeyEvent::kConfirm;
      case 'b':
        return KeyEvent::kPrev;
      case 'g':
        return KeyEvent::kFirst;
      case 'G':
        return KeyEvent::kLast;
      case '.':
        return KeyEvent::kStepForward1s;
      case ',':
        return KeyEvent::kStepBackward1s;
      case 'k':
        return KeyEvent::kScrollUp;
      case 'j':
        return KeyEvent::kScrollDown;
      case 'H':
        return KeyEvent::kScrollHead;
      case 'T':
        return KeyEvent::kScrollTail;
      case 's':
        return KeyEvent::kSaveYaml;
      case 'a':
        return KeyEvent::kToggleArrayExpand;
      case 'i':
        return KeyEvent::kTogglePreview;
      case 'u':
        return KeyEvent::kToggleUndistort;
      case 'p':
        return KeyEvent::kToggleProjectPcd;
      case 't':
        return KeyEvent::kSelectPcdTopic;
      case 'f':
        return KeyEvent::kCyclePcdProperty;
      case 'c':
        return KeyEvent::kCyclePcdScheme;
      case 'r':
        return KeyEvent::kTogglePcdRange;
      case '=':
      case '+':
        return KeyEvent::kPcdPointSizeUp;
      case '-':
        return KeyEvent::kPcdPointSizeDown;
      case ']':
        return KeyEvent::kPcdAlphaUp;
      case '[':
        return KeyEvent::kPcdAlphaDown;
      default:
        return KeyEvent::kUnknown;
    }
  }

  // Three-byte CSI sequences: ESC [ <final>
  if (bytes.size() == 3 && static_cast<unsigned char>(bytes[0]) == 0x1B && bytes[1] == '[') {
    switch (bytes[2]) {
      case 'A':
        return KeyEvent::kScrollUp;  // Up arrow
      case 'B':
        return KeyEvent::kScrollDown;  // Down arrow
      case 'C':
        return KeyEvent::kNext;  // Right arrow
      case 'D':
        return KeyEvent::kPrev;  // Left arrow
      case 'H':
        return KeyEvent::kScrollHead;  // Home
      case 'F':
        return KeyEvent::kScrollTail;  // End
      default:
        return KeyEvent::kUnknown;
    }
  }

  return KeyEvent::kUnknown;
}

TerminalRawMode::TerminalRawMode()
{
  if (!::isatty(STDIN_FILENO)) {
    return;
  }
  if (::tcgetattr(STDIN_FILENO, &saved_) != 0) {
    return;
  }
  // Disable canonical mode, echo, and signal generation. With ISIG off,
  // Ctrl-C arrives as byte 0x03 which classify_key() maps to kQuit; the
  // RAII restore runs on scope exit instead of the process being killed
  // mid-render.
  const termios raw = make_raw_settings(saved_);
  if (::tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
    return;
  }
  active_ = true;
}

void TerminalRawMode::suspend_for_line_input()
{
  if (!active_) {
    return;
  }
  ::tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
}

void TerminalRawMode::resume_after_line_input()
{
  if (!active_) {
    return;
  }
  const termios raw = make_raw_settings(saved_);
  ::tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}

TerminalRawMode::~TerminalRawMode()
{
  if (active_) {
    ::tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
  }
}

namespace
{

// Raw single-event read with no double-tap tracking. read_key_event()
// layers '.'/',' double-tap detection on top of this.
KeyEvent read_key_event_raw()
{
  const int first = read_byte();
  if (first == kReadResizeInterrupt) {
    return KeyEvent::kResize;
  }
  if (first == kReadEofOrError) {
    return KeyEvent::kQuit;
  }

  // Non-ESC: single-byte classification covers everything.
  if (first != 0x1B) {
    const char ch = static_cast<char>(first);
    return classify_key(std::string_view(&ch, 1));
  }

  // ESC may be standalone or the start of a CSI sequence. Peek briefly for
  // a '[' follower.
  //
  // A SIGWINCH can interrupt the poll() inside read_available(). Without
  // handling that case we would treat a partial ESC sequence as a lone ESC
  // and quit the preview unexpectedly. Retry on EINTR so the sequence can be
  // assembled; the pending resize flag is left set and will be surfaced by
  // the next read_byte() call.
  char follow[2] = {0, 0};
  std::size_t got = 0;
  while (got < sizeof(follow)) {
    const ssize_t n = read_available(follow + got, sizeof(follow) - got, kEscFollowupPollMs);
    if (n > 0) {
      got += static_cast<std::size_t>(n);
    } else if (n == 0) {
      break;  // timeout
    } else if (errno != EINTR) {
      break;  // genuine read error
    }
  }
  if (got < sizeof(follow) || follow[0] != '[') {
    return KeyEvent::kQuit;  // lone ESC or unrecognized sequence
  }
  const char seq[3] = {'\x1B', follow[0], follow[1]};
  return classify_key(std::string_view(seq, 3));
}

}  // namespace

KeyEvent upgrade_step_double_tap(
  KeyEvent ev, KeyEvent prev, std::chrono::steady_clock::duration since_prev,
  KeyEvent & remember) noexcept
{
  const bool is_step = ev == KeyEvent::kStepForward1s || ev == KeyEvent::kStepBackward1s;
  if (!is_step) {
    remember = KeyEvent::kUnknown;  // any other event breaks a pending double-tap
    return ev;
  }
  if (ev == prev && since_prev <= kStepDoubleTapWindow) {
    // Second rapid press of the same step key: upgrade to its 10s variant and
    // clear the memory so a third press starts a fresh single step.
    remember = KeyEvent::kUnknown;
    return ev == KeyEvent::kStepForward1s ? KeyEvent::kStepForward10s : KeyEvent::kStepBackward10s;
  }
  remember = ev;  // first press: remember it as a possible double-tap opener
  return ev;
}

KeyEvent read_key_event()
{
  const KeyEvent ev = read_key_event_raw();

  // Double-tap memory persists across calls for the lifetime of the process.
  // The interactive loops are single-threaded, so plain function-local statics
  // are safe. prev_time starts at the clock epoch, so the first press is always
  // outside the window and treated as a single step.
  static KeyEvent prev_step = KeyEvent::kUnknown;
  static std::chrono::steady_clock::time_point prev_time{};

  const auto now = std::chrono::steady_clock::now();
  KeyEvent remember = KeyEvent::kUnknown;
  const KeyEvent out = upgrade_step_double_tap(ev, prev_step, now - prev_time, remember);
  prev_step = remember;
  prev_time = now;
  return out;
}

}  // namespace bagwiz::core
