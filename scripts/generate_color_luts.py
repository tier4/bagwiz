#!/usr/bin/env python3
# Copyright 2026 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Generate 256-entry BGR lookup tables from matplotlib colormaps.

Run this script from the repository root and redirect the output to a temporary
file, for example:

    pixi run python scripts/generate_color_luts.py > /tmp/luts.txt

Then paste the generated C++ arrays into the anonymous namespace of
src/core/pointcloud/color_mapper.cpp, replacing the existing LUTs. Finally,
reformat the file with clang-format:

    pixi run clang-format -i src/core/pointcloud/color_mapper.cpp

The output is ordered {B, G, R} so the tables can be consumed directly by
OpenCV as BGR pixel values.
"""

import numpy as np
from matplotlib import colormaps


def main() -> None:
    schemes = ["viridis", "turbo", "jet", "plasma", "inferno", "magma", "rainbow"]
    for name in schemes:
        cmap = colormaps[name]
        rgba = (cmap(np.linspace(0, 1, 256))[:, :3] * 255).astype(np.uint8)
        # Convert RGB to BGR for OpenCV.
        bgr = rgba[:, ::-1]
        rows = ["{" + ",".join(str(v) for v in row) + "}" for row in bgr]
        print(f"constexpr std::array<BgrColor, 256> k{name.title()}Lut = {{")
        print("  " + ",\n  ".join(rows) + "\n}};")


if __name__ == "__main__":
    main()
