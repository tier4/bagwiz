// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/progress_bar.hpp"

#include <indicators/indeterminate_progress_bar.hpp>
#include <indicators/progress_bar.hpp>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

namespace bagwiz::core::slam
{

bool progress_enabled(bool stderr_is_tty, bool no_color_env_set, bool no_progress_flag)
{
  return stderr_is_tty && !no_color_env_set && !no_progress_flag;
}

std::int64_t progress_total(const io::BagReader::Stats & stats, std::span<const std::string> topics)
{
  std::int64_t total = 0;
  for (const auto & topic : topics) {
    if (topic.empty()) {
      continue;
    }
    const auto it = stats.per_topic.find(topic);
    if (it != stats.per_topic.end() && it->second > 0) {
      total += it->second;
    }
  }
  return total;
}

std::int64_t progress_total(
  const std::unordered_map<std::string, std::int64_t> & counts, std::span<const std::string> topics)
{
  std::int64_t total = 0;
  for (const auto & topic : topics) {
    if (topic.empty()) {
      continue;
    }
    const auto it = counts.find(topic);
    if (it != counts.end() && it->second > 0) {
      total += it->second;
    }
  }
  return total;
}

namespace
{
// Short alias for indicators' option types (cpplint forbids `using namespace`).
namespace opt = indicators::option;

// Cap redraw frequency: repaint only when the integer tenths-of-a-percent
// value advances or ~50 ms have passed, so a high-rate IMU topic cannot flood
// stderr.
constexpr auto kMinRedrawInterval = std::chrono::milliseconds(50);
constexpr int kBarWidth = 40;
constexpr auto kSpinnerTickInterval = std::chrono::milliseconds(100);

std::size_t clamp_to_total(std::int64_t value, std::int64_t total)
{
  if (value <= 0) {
    return 0;
  }
  if (value >= total) {
    return static_cast<std::size_t>(total);
  }
  return static_cast<std::size_t>(value);
}
}  // namespace

// ---------------------------------------------------------------------------
// ScanProgress
// ---------------------------------------------------------------------------

struct ScanProgress::Impl
{
  bool determinate = false;
  bool finished = false;
  std::int64_t total = 0;
  int last_tenths = -1;
  std::chrono::steady_clock::time_point last_draw{};
  std::unique_ptr<indicators::ProgressBar> bar;
  std::unique_ptr<indicators::IndeterminateProgressBar> spinner;
};

ScanProgress::ScanProgress(std::int64_t total, bool enabled)
{
  if (!enabled) {
    return;  // impl_ stays null -> every method is a no-op
  }
  impl_ = std::make_unique<Impl>();
  impl_->total = total;
  impl_->determinate = total > 0;

  if (impl_->determinate) {
    impl_->bar = std::make_unique<indicators::ProgressBar>(
      opt::BarWidth{kBarWidth}, opt::Start{"["}, opt::Fill{"="}, opt::Lead{">"},
      opt::Remainder{" "}, opt::End{"]"}, opt::PrefixText{"Reading "},
      opt::ForegroundColor{indicators::Color::cyan}, opt::ShowPercentage{true},
      opt::ShowElapsedTime{true}, opt::ShowRemainingTime{true},
      opt::MaxProgress{static_cast<std::size_t>(total)}, opt::Stream{std::cerr});
  } else {
    impl_->spinner = std::make_unique<indicators::IndeterminateProgressBar>(
      opt::BarWidth{kBarWidth}, opt::Start{"["}, opt::Fill{"."}, opt::Lead{"<=>"}, opt::End{"]"},
      opt::PrefixText{"Reading "}, opt::ForegroundColor{indicators::Color::cyan},
      opt::Stream{std::cerr});
  }
}

void ScanProgress::update(std::int64_t processed, std::int64_t scans)
{
  if (!impl_ || impl_->finished) {
    return;
  }
  const auto now = std::chrono::steady_clock::now();

  if (impl_->determinate) {
    const int tenths = impl_->total > 0 ? static_cast<int>((processed * 1000) / impl_->total) : 0;
    const bool advanced = tenths != impl_->last_tenths;
    const bool elapsed = (now - impl_->last_draw) >= kMinRedrawInterval;
    if (!advanced && !elapsed) {
      return;
    }
    impl_->last_tenths = tenths;
    impl_->last_draw = now;
    impl_->bar->set_option(indicators::option::PostfixText{std::to_string(scans) + " scans"});
    impl_->bar->set_progress(clamp_to_total(processed, impl_->total));
    return;
  }

  // Indeterminate: total unknown, so just animate as messages arrive.
  if ((now - impl_->last_draw) < kMinRedrawInterval) {
    return;
  }
  impl_->last_draw = now;
  impl_->spinner->set_option(indicators::option::PostfixText{std::to_string(scans) + " scans"});
  impl_->spinner->tick();
}

void ScanProgress::done()
{
  if (!impl_ || impl_->finished) {
    return;
  }
  impl_->finished = true;
  if (impl_->determinate) {
    impl_->bar->set_progress(static_cast<std::size_t>(impl_->total));
    impl_->bar->mark_as_completed();
  } else {
    impl_->spinner->mark_as_completed();
  }
}

ScanProgress::~ScanProgress()
{
  // An early return (error path) can drop the reporter mid-bar; move the cursor
  // off the partially drawn line so later output is not appended to it.
  if (impl_ && !impl_->finished) {
    std::cerr << std::endl;
  }
}

// ---------------------------------------------------------------------------
// FinalizeSpinner
// ---------------------------------------------------------------------------

struct FinalizeSpinner::Impl
{
  std::atomic<bool> stop{false};
  bool stopped = false;
  std::unique_ptr<indicators::IndeterminateProgressBar> bar;
  std::thread worker;
};

FinalizeSpinner::FinalizeSpinner(std::string label, bool enabled)
{
  if (!enabled) {
    return;  // impl_ stays null -> no thread, no output
  }
  impl_ = std::make_unique<Impl>();

  impl_->bar = std::make_unique<indicators::IndeterminateProgressBar>(
    opt::BarWidth{kBarWidth}, opt::Start{"["}, opt::Fill{"."}, opt::Lead{"<=>"}, opt::End{"]"},
    opt::PostfixText{std::move(label)}, opt::ForegroundColor{indicators::Color::yellow},
    opt::Stream{std::cerr});

  impl_->worker = std::thread([impl = impl_.get()]() {
    while (!impl->stop.load(std::memory_order_relaxed)) {
      impl->bar->tick();
      std::this_thread::sleep_for(kSpinnerTickInterval);
    }
  });
}

void FinalizeSpinner::stop()
{
  if (!impl_ || impl_->stopped) {
    return;
  }
  impl_->stopped = true;
  impl_->stop.store(true, std::memory_order_relaxed);
  if (impl_->worker.joinable()) {
    impl_->worker.join();
  }
  // Safe to touch the bar now: the worker has joined, so no concurrent tick().
  impl_->bar->mark_as_completed();
}

FinalizeSpinner::~FinalizeSpinner()
{
  stop();
}

}  // namespace bagwiz::core::slam
