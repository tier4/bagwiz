// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/bag/bag_inplace.hpp"

#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

namespace bagwiz::core
{

namespace
{

// Generate a sibling path with a unique suffix. Uses pid + steady_clock
// ticks to avoid colliding with concurrent invocations from the same
// or different processes operating on the same parent directory.
std::filesystem::path make_tmp_sibling(const std::filesystem::path & final_path)
{
  const auto parent =
    final_path.has_parent_path() ? final_path.parent_path() : std::filesystem::current_path();
  const auto stem = final_path.filename().string();
  const auto pid = static_cast<std::uint64_t>(::getpid());
  const auto ticks =
    static_cast<std::uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
  return parent /
         (stem + ".bagwiz-inplace-tmp-" + std::to_string(pid) + "-" + std::to_string(ticks));
}

// RAII guard that removes the tmp path on destruction unless commit()
// is called. Lets us treat writer_fn exceptions and rename failures
// uniformly: the tmp is always cleaned up on the unhappy path.
class TmpPathGuard
{
public:
  explicit TmpPathGuard(std::filesystem::path path) : path_(std::move(path)) {}

  ~TmpPathGuard()
  {
    if (!committed_) {
      std::error_code ec;
      std::filesystem::remove_all(path_, ec);
    }
  }

  TmpPathGuard(const TmpPathGuard &) = delete;
  TmpPathGuard & operator=(const TmpPathGuard &) = delete;
  TmpPathGuard(TmpPathGuard &&) = delete;
  TmpPathGuard & operator=(TmpPathGuard &&) = delete;

  void commit() { committed_ = true; }
  const std::filesystem::path & path() const { return path_; }

private:
  std::filesystem::path path_;
  bool committed_ = false;
};

}  // namespace

void write_bag_inplace(
  const std::filesystem::path & final_path,
  const std::function<void(const std::filesystem::path & tmp_path)> & writer_fn)
{
  if (!std::filesystem::exists(final_path)) {
    throw std::runtime_error(
      "write_bag_inplace: final_path does not exist: " + final_path.string());
  }

  TmpPathGuard guard(make_tmp_sibling(final_path));

  // Hand off to the caller. Any exception escapes; the guard wipes the
  // tmp path on the way out.
  writer_fn(guard.path());

  // Sanity check: writer_fn must have materialised something at the
  // tmp path. Without this check, a no-op writer would silently delete
  // final_path on the next step.
  if (!std::filesystem::exists(guard.path())) {
    throw std::runtime_error(
      "write_bag_inplace: writer_fn returned successfully but produced no output at " +
      guard.path().string());
  }

  // Simple one-shot swap. There is a brief window between remove_all
  // and rename during which a process crash leaves no bag at
  // final_path; the helper's contract documents this trade-off.
  std::error_code ec;
  std::filesystem::remove_all(final_path, ec);
  if (ec) {
    throw std::runtime_error(
      "write_bag_inplace: failed to remove final_path '" + final_path.string() +
      "': " + ec.message());
  }
  std::filesystem::rename(guard.path(), final_path, ec);
  if (ec) {
    // remove_all already wiped final_path; we cannot recover it from
    // here. Surface the situation explicitly so the caller can log a
    // recovery hint.
    throw std::runtime_error(
      "write_bag_inplace: failed to rename tmp '" + guard.path().string() + "' to '" +
      final_path.string() + "' (original bag has been deleted): " + ec.message());
  }
  guard.commit();
}

}  // namespace bagwiz::core
