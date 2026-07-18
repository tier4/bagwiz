// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/map_viewer_open.hpp"

#include <string>

namespace bagwiz::core::slam
{

std::string browser_open_command(const std::string & url)
{
#if defined(_WIN32)
  // cmd's `start` treats the first quoted token as the window title, so pass an
  // empty title before the URL. It returns immediately, so no backgrounding.
  return "start \"\" \"" + url + "\"";
#elif defined(__APPLE__)
  return "open \"" + url + "\" >/dev/null 2>&1 &";
#else
  return "xdg-open \"" + url + "\" >/dev/null 2>&1 &";
#endif
}

}  // namespace bagwiz::core::slam
