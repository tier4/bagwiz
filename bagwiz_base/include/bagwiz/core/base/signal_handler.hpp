// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__BASE__SIGNAL_HANDLER_HPP_
#define BAGWIZ__CORE__BASE__SIGNAL_HANDLER_HPP_

#include <atomic>

namespace bagwiz::core
{

// Lifetime-managed SIGWINCH handler. On construction installs an
// async-signal-safe handler that flips a single atomic flag; on
// destruction the previous disposition is restored.
//
// Construction is a no-op when sigaction() fails. Only one instance
// may exist at a time; constructing a second instance overwrites the
// global flag pointer and the destructor of the older instance will
// silently observe inactive state.
class SigwinchScope
{
public:
  SigwinchScope();
  ~SigwinchScope();
  SigwinchScope(const SigwinchScope &) = delete;
  SigwinchScope & operator=(const SigwinchScope &) = delete;
  SigwinchScope(SigwinchScope &&) = delete;
  SigwinchScope & operator=(SigwinchScope &&) = delete;

  // Returns true if SIGWINCH has been observed since the last call to
  // consume(). The flag is cleared atomically.
  bool consume();

private:
  bool active_ = false;
};

// Tests and the terminal_input layer can call this to inspect / clear
// the SIGWINCH flag without owning a SigwinchScope. Returns false when
// no scope is active (the flag is always-false in that case).
bool consume_resize_flag() noexcept;

// Test seam: forcibly raise the flag from non-signal context. Used by
// unit tests that exercise read_key_event's EINTR handling. Returns
// false when no scope is active.
bool raise_resize_flag_for_test() noexcept;

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__BASE__SIGNAL_HANDLER_HPP_
