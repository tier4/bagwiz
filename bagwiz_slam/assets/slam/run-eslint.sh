#!/usr/bin/env bash
# Run ESLint over the map viewer's TypeScript using its own toolchain
# (bagwiz_slam/assets/slam/node_modules, the same one the build uses for tsc).
# Lints the whole viewer so the type-aware rules see every source (the project
# is tiny), and installs the toolchain on first use so the hook also works on
# a fresh checkout. Invoked by the `eslint-map-viewer` pre-commit hook.
set -euo pipefail

viewer_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ ! -x "${viewer_dir}/node_modules/.bin/eslint" ]; then
    echo "[eslint] installing map-viewer toolchain (npm ci)…" >&2
    (cd "${viewer_dir}" && npm ci --no-audit --no-fund)
fi

exec "${viewer_dir}/node_modules/.bin/eslint" \
    --config "${viewer_dir}/eslint.config.mjs" "${viewer_dir}"
