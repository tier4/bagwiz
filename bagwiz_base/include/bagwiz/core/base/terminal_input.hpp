// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__BASE__TERMINAL_INPUT_HPP_
#define BAGWIZ__CORE__BASE__TERMINAL_INPUT_HPP_

#include <termios.h>

#include <string_view>

namespace bagwiz::core
{

// High-level events the interactive UI cares about. Raw key bytes (single
// chars or escape sequences) are collapsed into these buckets by
// classify_key().
enum class KeyEvent {
  kNext,               // next message
  kPrev,               // previous message
  kFirst,              // jump to first
  kLast,               // jump to last (may force a full scan in the caller)
  kStepForward1s,      // jump to the next message at least one second ahead
  kStepBackward1s,     // jump to the previous message at least one second behind
  kStepForward10s,     // jump ~10 seconds ahead ('>')
  kStepBackward10s,    // jump ~10 seconds behind ('<')
  kScrollUp,           // scroll the current message's rendered body up by one line
  kScrollDown,         // scroll the current message's rendered body down by one line
  kScrollHead,         // jump to the top of the current message's body
  kScrollTail,         // jump to the bottom of the current message's body
  kSaveYaml,           // save current message body as YAML (walk command)
  kToggleArrayExpand,  // toggle full-expansion of long primitive arrays (walk command)
  kTogglePreview,      // toggle in-terminal image preview (walk command)
  kToggleUndistort,    // toggle undistort in image preview (walk command)
  kToggleProjectPcd,   // toggle point-cloud projection overlay in image preview (walk command)
  kSelectPcdTopic,     // choose a PointCloud2 topic for the projection overlay
  kCyclePcdProperty,   // cycle point-cloud visualization property (walk command)
  kCyclePcdScheme,     // cycle point-cloud color scheme (walk command)
  kTogglePcdRange,     // toggle point-cloud value range auto/manual (walk command)
  kPcdPointSizeUp,     // increase projected point size (walk command)
  kPcdPointSizeDown,   // decrease projected point size (walk command)
  kPcdAlphaUp,         // increase point overlay opacity (walk command)
  kPcdAlphaDown,       // decrease point overlay opacity (walk command)
  kConfirm,            // confirm the current prompt/selection
  kQuit,               // exit the interactive loop
  kResize,             // terminal was resized (synthesised by read_key_event
                       // from a SIGWINCH flag set by the signal_handler
                       // module; never produced by classify_key)
  kUnknown,            // unrecognized input; caller should ignore or beep
};

// Pure classifier for an already-captured byte sequence. Exposed so unit
// tests can exercise every key mapping without touching /dev/tty.
//
// Accepted input:
//   * single bytes: Space (next), 'b' (prev), 'g' (first), 'G' (last),
//     '.' (step forward one second), ',' (step backward one second),
//     '>' (step forward ~10 seconds), '<' (step backward ~10 seconds),
//     'k' (scroll up), 'j' (scroll down), 'H' (scroll head), 'T' (scroll
//     tail), 's' (save as yaml — walk), 'a' (toggle array expand — walk),
//     'i' (toggle image preview — walk), 'u' (toggle undistort — walk),
//     'q'/'Q' (quit), plus control
//     chars (^C, ^D) and a lone ESC (0x1B) for quit
//   * three-byte ANSI sequences "ESC [ C" (Right -> next), "ESC [ D"
//     (Left -> prev), "ESC [ A" (Up -> scroll up), "ESC [ B" (Down ->
//     scroll down), "ESC [ H" (Home -> scroll head), "ESC [ F" (End ->
//     scroll tail)
// Anything else -> kUnknown.
KeyEvent classify_key(std::string_view bytes);

// RAII guard that puts STDIN_FILENO into a minimal "cbreak" mode (no echo,
// no line buffering, VMIN=1 VTIME=0). On destruction the previous termios
// is restored. Construction is a no-op when stdin is not a TTY, in which
// case active() returns false.
class TerminalRawMode
{
public:
  TerminalRawMode();
  ~TerminalRawMode();

  TerminalRawMode(const TerminalRawMode &) = delete;
  TerminalRawMode & operator=(const TerminalRawMode &) = delete;
  TerminalRawMode(TerminalRawMode &&) = delete;
  TerminalRawMode & operator=(TerminalRawMode &&) = delete;

  bool active() const { return active_; }

  // Temporarily restore canonical stdin so callers can use line-oriented
  // reads (e.g. std::getline). Pair every call with resume_after_line_input()
  // when raw mode should resume; otherwise the destructor still restores
  // the saved (cooked) settings safely.
  void suspend_for_line_input();
  void resume_after_line_input();

private:
  bool active_ = false;
  termios saved_{};
};

// Blocks on stdin until a single KeyEvent can be produced. Handles the ESC
// prefetch needed to distinguish a lone ESC from an arrow-key sequence.
// Returns kQuit when the read is interrupted (e.g. SIGINT) or on EOF.
// Undefined unless stdin is a TTY.
KeyEvent read_key_event();

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__BASE__TERMINAL_INPUT_HPP_
