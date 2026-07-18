// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/base/signal_handler.hpp"

// POSIX sigaction is not exposed by the C++ <csignal> wrapper, so the
// C header is required here.
#include <signal.h>  // NOLINT(modernize-deprecated-headers)

#include <atomic>

namespace bagwiz::core
{

namespace
{

// Lock-free flag flipped by the SIGWINCH handler. A nullptr value means
// no SigwinchScope is currently active.
std::atomic<bool> * g_flag = nullptr;

// Saved disposition so the destructor can restore exactly what was in
// place before our scope was entered.
struct sigaction g_prev{};
bool g_installed = false;

extern "C" void handle_sigwinch(int signum)
{
  (void)signum;
  // Async-signal-safe: a relaxed store to an atomic bool is allowed.
  if (g_flag != nullptr) {
    g_flag->store(true, std::memory_order_relaxed);
  }
}

}  // namespace

SigwinchScope::SigwinchScope()
{
  static std::atomic<bool> s_flag{false};
  s_flag.store(false, std::memory_order_relaxed);
  g_flag = &s_flag;

  struct sigaction sa{};
  sa.sa_handler = &handle_sigwinch;
  sigemptyset(&sa.sa_mask);
  // SA_RESTART intentionally NOT set: we want read() to return EINTR so
  // the input loop can observe the resize flag synchronously.
  sa.sa_flags = 0;
  if (sigaction(SIGWINCH, &sa, &g_prev) == 0) {
    g_installed = true;
    active_ = true;
  } else {
    g_flag = nullptr;
  }
}

SigwinchScope::~SigwinchScope()
{
  if (active_ && g_installed) {
    sigaction(SIGWINCH, &g_prev, nullptr);
    g_installed = false;
  }
  g_flag = nullptr;
}

bool SigwinchScope::consume()
{
  return consume_resize_flag();
}

bool consume_resize_flag() noexcept
{
  if (g_flag == nullptr) {
    return false;
  }
  return g_flag->exchange(false, std::memory_order_acq_rel);
}

bool raise_resize_flag_for_test() noexcept
{
  if (g_flag == nullptr) {
    return false;
  }
  g_flag->store(true, std::memory_order_release);
  return true;
}

}  // namespace bagwiz::core
