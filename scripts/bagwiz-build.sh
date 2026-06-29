#!/usr/bin/env bash
# bagwiz-build.sh - build bagwiz via colcon, picking the SLAM / GPU profile from
# the active pixi environment. Invoked by `pixi run build` and `pixi run build-gpu`
# (see pixi.toml); not meant to be run on its own, since it relies on the pixi
# environment's activated ROS 2 + conda toolchain.
#
# SLAM is the default build: `bagwiz map slam` links the GLIM stack (GTSAM +
# gtsam_points + glim), which this script builds once into a vendored prefix via
# scripts/build-glim-deps.sh before compiling bagwiz against it. The three
# profiles, chosen automatically:
#
#   * CPU env (humble / jazzy)        -> CPU SLAM build (-DBAGWIZ_WITH_SLAM=ON),
#                                        after building the GLIM CPU deps.
#   * --gpu in a *-gpu env            -> CUDA SLAM build (-DBAGWIZ_WITH_SLAM_CUDA),
#     (humble-gpu / jazzy-gpu)          after building the GLIM CPU + CUDA deps.
#   * lyrical                         -> plain build, NO SLAM. lyrical cannot pin
#                                        Eigen 3.4 (ros-lyrical-tf2-eigen-kdl hard-
#                                        requires eigen-abi 5.0.1), so the GLIM
#                                        stack is unbuildable there; bagwiz still
#                                        builds, just without `map slam`.
#
# Each pixi environment builds into its OWN base (build/<env>, install/<env>) keyed
# on $PIXI_ENVIRONMENT_NAME, so switching `-e <env>` never reuses another env's
# colcon/CMake cache (which would silently link the wrong ROS or CUDA libraries).
# The trailing symlink keeps build/compile_commands.json pointing at the last-built
# env for editor tooling.
#
# Intentionally NOT pixi-input-cached: the bundled GLIM step is already idempotent
# via its own stamp (build-glim-deps.sh), and an `inputs` cache keyed on src/ alone
# would report a false "cache hit" and skip the whole task even when the GLIM prefix
# is missing (e.g. after `pixi run clean`), leaving the SLAM build to fail at
# find_package(glim). colcon's own incremental build keeps a no-change rebuild fast.
set -euo pipefail

# First positional (anything not starting with `-`) is the CMake build type; --gpu
# selects the CUDA SLAM profile. Accept them in any order.
build_type="Release"
gpu=0
for arg in "$@"; do
    case "${arg}" in
    --gpu) gpu=1 ;;
    -*)
        echo "bagwiz-build: unknown flag '${arg}' (expected --gpu)" >&2
        exit 2
        ;;
    *) build_type="${arg}" ;;
    esac
done

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO="${PIXI_PROJECT_ROOT:-$(cd -- "${SCRIPT_DIR}/.." && pwd)}"
ENV_NAME="${PIXI_ENVIRONMENT_NAME:-humble}"
# Distro = env name without the optional `-gpu` suffix (humble-gpu -> humble).
DISTRO="${ENV_NAME%-gpu}"

# SLAM is supported on every distro except lyrical (see header).
slam=1
if [ "${DISTRO}" = "lyrical" ]; then
    slam=0
fi

cd "${REPO}"

cmake_args=(-DCMAKE_BUILD_TYPE="${build_type}" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON)

if [ "${gpu}" -eq 1 ]; then
    if [ "${slam}" -eq 0 ]; then
        echo "bagwiz-build --gpu: SLAM (and thus the GPU build) is unsupported on '${DISTRO}'." >&2
        echo "  Use a SLAM distro: pixi run -e humble-gpu build-gpu   # or jazzy-gpu" >&2
        exit 1
    fi
    : "${CONDA_PREFIX:?bagwiz-build --gpu needs an activated pixi env (CONDA_PREFIX unset)}"
    : "${CXX:?bagwiz-build --gpu needs the conda C++ compiler on \$CXX}"
    # Fail fast (BEFORE the slow CPU GLIM build) when this env has no CUDA toolkit,
    # i.e. it is not a *-gpu env. nvcc comes from the `gpu` feature, which only the
    # *-gpu environments carry; a bare `pixi run build-gpu` lands in `default`
    # (= humble, no CUDA) and could otherwise waste a tens-of-minutes GLIM build
    # before build-glim-deps.sh --cuda rejects it.
    if [ ! -x "${CONDA_PREFIX}/bin/nvcc" ]; then
        echo "bagwiz-build --gpu: no CUDA toolkit in this environment ('${ENV_NAME}')." >&2
        echo "  The -gpu tasks (build-gpu/install-gpu/run-gpu/test-gpu) carry CUDA only in a" >&2
        echo "  *-gpu environment. Re-run the SAME task with -e, e.g.:" >&2
        echo "    pixi run -e humble-gpu install-gpu      # or build-gpu / run-gpu / test-gpu" >&2
        echo "    pixi run -e jazzy-gpu  install-gpu      # jazzy-gpu works too" >&2
        exit 1
    fi
    # GLIM CPU deps first (the CUDA stack reuses this prefix's GTSAM), then the
    # CUDA deps. Both are sub-second no-ops once their stamped prefixes exist.
    bash "${SCRIPT_DIR}/build-glim-deps.sh"
    bash "${SCRIPT_DIR}/build-glim-deps.sh" --cuda
    cmake_args+=(
        -DBAGWIZ_WITH_SLAM_CUDA=ON
        "-DCMAKE_PREFIX_PATH=${REPO}/install/${ENV_NAME}/glim-deps-cuda;${REPO}/install/${ENV_NAME}/glim-deps"
        -DCMAKE_CUDA_COMPILER="${CONDA_PREFIX}/bin/nvcc"
        "-DCMAKE_CUDA_HOST_COMPILER=${CXX}"
        -DCMAKE_CUDA_ARCHITECTURES=86
        -DCUDAToolkit_ROOT="${CONDA_PREFIX}"
    )
elif [ "${slam}" -eq 1 ]; then
    bash "${SCRIPT_DIR}/build-glim-deps.sh"
    cmake_args+=(
        -DBAGWIZ_WITH_SLAM=ON
        "-DCMAKE_PREFIX_PATH=${REPO}/install/${ENV_NAME}/glim-deps"
    )
fi

colcon build --symlink-install --packages-up-to bagwiz \
    --build-base "build/${ENV_NAME}" --install-base "install/${ENV_NAME}" \
    --cmake-args "${cmake_args[@]}"
ln -sfn "${ENV_NAME}/compile_commands.json" build/compile_commands.json
