#!/usr/bin/env bash
# build-glim-deps.sh - build the GLIM SLAM stack into a vendored prefix.
#
# `bagwiz slam` links GLIM (koide3/glim) plus its heavy dependencies GTSAM and
# gtsam_points. None of the three are packaged on conda-forge/robostack at the
# versions GLIM needs (GTSAM ships only as alpha git tags, and gtsam_points
# bundles patched iSAM2 sources that must match), so they are built from source.
#
# Rather than rebuild this multi-ten-minute stack on every `colcon build`, it is
# built ONCE into install/<distro>/glim-deps and then `find_package`d by bagwiz
# (Approach B - a vendored prefix). This script is meant to be run inside the
# pixi environment via `pixi run -e <distro> build-glim`, so the conda toolchain
# (gcc, cmake, ninja) and the env-provided Boost / Eigen / fmt / spdlog / OpenMP
# are already on the relevant paths.
#
# Re-running is cheap: if the prefix already holds libglim.so for the current
# pinned versions it exits immediately. Pass --force to rebuild from scratch.
set -euo pipefail

# Pinned, mutually-validated versions (see .claude/prds/bagwiz-slam.prd.md,
# Milestone 0). conda-forge GTSAM (4.2 stable) does NOT work - gtsam_points
# bundles iSAM2 sources matching this alpha tag. glim + gtsam_points are pinned
# to exact commit SHAs that build together: the v1.2.x *tags* are NOT mutually
# compatible (glim's sub_mapping.cpp references a gtsam_points API that only
# exists on the branch these SHAs come from).
GTSAM_REF="4.3a1"
GTSAM_POINTS_REF="17578c8e8c1dc9bd55b0b8c31d2656230098977c"
GLIM_REF="88b3833229a9c3308e95065719a40acdd5f64c33"

# Repository root: this script lives at <repo>/scripts/build-glim-deps.sh.
_self="$(readlink -f -- "${BASH_SOURCE[0]}")"
REPO="$(cd -- "$(dirname -- "${_self}")/.." && pwd)"

# The active pixi environment (humble/jazzy/lyrical/...). Falls back to humble
# for a bare invocation outside `pixi run`.
ENV_NAME="${PIXI_ENVIRONMENT_NAME:-humble}"

PREFIX="${REPO}/install/${ENV_NAME}/glim-deps"
SRC="${REPO}/build/${ENV_NAME}/glim-src"
STAMP="${PREFIX}/.glim-deps.stamp"
WANT="gtsam=${GTSAM_REF} gtsam_points=${GTSAM_POINTS_REF} glim=${GLIM_REF}"

force=0
[ "${1:-}" = "--force" ] && force=1

if [ "${force}" -eq 0 ] && [ -f "${PREFIX}/lib/libglim.so" ] && [ -f "${STAMP}" ] &&
    [ "$(cat "${STAMP}")" = "${WANT}" ]; then
    echo "build-glim-deps: up to date (${WANT}) at ${PREFIX}"
    exit 0
fi

# Pick the conda toolchain explicitly so the build matches how bagwiz itself is
# compiled (conda gcc - it excludes /usr/include, hence Boost must come from the
# env, supplied via pixi's libboost-devel).
CXX_BIN="$(command -v x86_64-conda-linux-gnu-g++ || command -v g++)"
CC_BIN="$(command -v x86_64-conda-linux-gnu-gcc || command -v gcc)"
export CXX="${CXX_BIN}" CC="${CC_BIN}"
export CMAKE_PREFIX_PATH="${CONDA_PREFIX:-}:${PREFIX}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"

mkdir -p "${SRC}" "${PREFIX}"
jobs="$(nproc)"

# clone <url> <ref> <dir>: shallow-checkout a tag/branch OR an exact commit SHA.
clone() {
    local url="$1" ref="$2" dst="${SRC}/$3"
    [ -d "${dst}/.git" ] && return 0
    if printf '%s' "${ref}" | grep -qE '^[0-9a-f]{40}$'; then
        git init -q "${dst}"
        git -C "${dst}" remote add origin "${url}"
        git -C "${dst}" fetch -q --depth 1 origin "${ref}"
        git -C "${dst}" checkout -q FETCH_HEAD
    else
        git clone -q --depth 1 --branch "${ref}" "${url}" "${dst}"
    fi
}

echo "build-glim-deps: building ${WANT} into ${PREFIX} (env=${ENV_NAME})"

# 1) GTSAM - headless, deterministic (no TBB), use the env's Eigen.
clone https://github.com/borglab/gtsam "${GTSAM_REF}" gtsam
cmake -S "${SRC}/gtsam" -B "${SRC}/gtsam/build" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
    -DGTSAM_BUILD_UNSTABLE=ON -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_WITH_TBB=OFF \
    -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_PYTHON=OFF \
    -DGTSAM_BUILD_DOCS=OFF -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DBUILD_SHARED_LIBS=ON
cmake --build "${SRC}/gtsam/build" --target install -j"${jobs}"

# 2) gtsam_points - CPU only, OpenMP on.
clone https://github.com/koide3/gtsam_points "${GTSAM_POINTS_REF}" gtsam_points
cmake -S "${SRC}/gtsam_points" -B "${SRC}/gtsam_points/build" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
    -DBUILD_WITH_CUDA=OFF -DBUILD_WITH_OPENMP=ON -DBUILD_TESTS=OFF -DBUILD_DEMO=OFF \
    -DBUILD_EXAMPLE=OFF -DBUILD_TOOLS=OFF
cmake --build "${SRC}/gtsam_points/build" --target install -j"${jobs}"

# 3) glim - headless CPU. Apply the fmt-12 compat fix: the env ships fmt 12,
# whose fmt::ptr() rejects smart pointers (moved to <fmt/std.h>); pass the raw
# pointer instead. Idempotent string replacement, version-tolerant.
clone https://github.com/koide3/glim "${GLIM_REF}" glim
ct="${SRC}/glim/src/glim/odometry/odometry_estimation_ct.cpp"
sed -i \
    -e 's/fmt::ptr(frames\[last\])/fmt::ptr(frames[last].get())/' \
    -e 's/fmt::ptr(frames\[last - 1\])/fmt::ptr(frames[last - 1].get())/' \
    "${ct}"
cmake -S "${SRC}/glim" -B "${SRC}/glim/build" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
    -DBUILD_WITH_CUDA=OFF -DBUILD_WITH_VIEWER=OFF -DBUILD_WITH_OPENCV=OFF
cmake --build "${SRC}/glim/build" --target install -j"${jobs}"

echo "${WANT}" >"${STAMP}"
echo "build-glim-deps: done -> ${PREFIX}"
