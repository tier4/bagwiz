// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__BAG_INPLACE_HPP_
#define BAGWIZ__CORE__BAG_INPLACE_HPP_

#include <filesystem>
#include <functional>

// Replace a bag in place by writing a fresh bag into a sibling tmp
// path and then swapping it into the final location. Used by
// rewrite-style commands (e.g. `bagwiz traj join` without `-o`) where
// the user wants the input bag to be replaced atomically.
//
// The swap is "simple one-shot": after `writer_fn` returns successfully
// the helper deletes `final_path` and renames the tmp into place. If
// the process crashes between the delete and the rename the original
// bag is lost — callers that cannot tolerate that should expose a
// separate `--output` mode and direct the user there.
namespace bagwiz::core
{

// Allocate a tmp path next to `final_path` (same parent, unique name)
// and invoke `writer_fn(tmp_path)` to materialise the new bag there.
// On success, replace `final_path` with the tmp:
//
//   1. `writer_fn(tmp)` — caller owns reader / writer construction
//      and must call writer->close() before returning.
//   2. remove_all(final_path)
//   3. rename(tmp, final_path)
//
// If `writer_fn` throws, the tmp path is removed and the exception is
// rethrown; `final_path` is left untouched. RAII guards ensure the tmp
// is cleaned up on any failure prior to step 3.
//
// `final_path` must already exist (this is an in-place replace, not a
// create). The helper does not inspect or preserve the layout of the
// original bag — the caller's `writer_fn` is responsible for picking a
// matching Format / Layout so the in-place semantics are preserved.
void write_bag_inplace(
  const std::filesystem::path & final_path,
  const std::function<void(const std::filesystem::path & tmp_path)> & writer_fn);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__BAG_INPLACE_HPP_
