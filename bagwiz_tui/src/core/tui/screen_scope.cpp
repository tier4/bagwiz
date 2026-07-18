// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tui/screen_scope.hpp"

#include "bagwiz/core/tui/layout.hpp"
#include "bagwiz/core/tui/renderer.hpp"

#include <iostream>

namespace bagwiz::core::tui
{

ScreenScope::ScreenScope(ScreenScopeConfig cfg)
: cfg_(cfg), out_(cfg.out != nullptr ? cfg.out : &std::cout)
{
  apply();
}

ScreenScope::~ScreenScope()
{
  if (active_) {
    restore();
  }
}

void ScreenScope::apply()
{
  if (cfg_.use_alt_screen) {
    enter_alt_screen(*out_);
  }
  if (cfg_.disable_autowrap) {
    set_autowrap(*out_, false);
  }
  if (cfg_.hide_cursor) {
    bagwiz::core::tui::hide_cursor(*out_);
  }
  out_->flush();
  active_ = true;
}

void ScreenScope::restore()
{
  if (cfg_.hide_cursor) {
    bagwiz::core::tui::show_cursor(*out_);
  }
  if (cfg_.disable_autowrap) {
    set_autowrap(*out_, true);
  }
  if (cfg_.use_alt_screen) {
    leave_alt_screen(*out_);
  } else {
    // Park the cursor below the rendered viewport so the next shell
    // prompt does not overwrite the footer. The terminal's current
    // size is used so the parking row is always in-bounds.
    const Size sz = query_terminal_size();
    move_cursor(*out_, sz.rows, 1);
    *out_ << '\n';
  }
  out_->flush();
  active_ = false;
}

void ScreenScope::suspend()
{
  if (active_) {
    restore();
  }
}

void ScreenScope::resume()
{
  if (!active_) {
    apply();
  }
}

}  // namespace bagwiz::core::tui
