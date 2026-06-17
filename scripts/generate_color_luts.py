#!/usr/bin/env python3
# Copyright 2026 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Generate 256-entry BGR lookup tables from matplotlib colormaps."""

import numpy as np
from matplotlib import colormaps


def main() -> None:
    schemes = ["viridis", "turbo", "jet", "plasma", "inferno", "magma", "rainbow"]
    for name in schemes:
        cmap = colormaps.get_cmap(name)
        rgba = (cmap(np.linspace(0, 1, 256))[:, :3] * 255).astype(np.uint8)
        # Convert RGB to BGR for OpenCV.
        bgr = rgba[:, ::-1]
        rows = ["{" + ",".join(str(v) for v in row) + "}" for row in bgr]
        print(f"constexpr std::array<BgrColor, 256> k{name.title().replace('Jet', 'Jet')}Lut = {{{{")
        print("  " + ",\n  ".join(rows) + "\n}};")


if __name__ == "__main__":
    main()
