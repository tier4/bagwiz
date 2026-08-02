#!/usr/bin/env bash
# bagwiz-build.sh - build bagwiz via colcon, picking the {core,full} profile from
# --core and the CPU/CUDA profile from the active pixi environment. Invoked by the
# pixi build tasks (build-core / build-full; see pixi.toml); not meant to be run on
# its own, since it relies on the pixi environment's activated ROS 2 + conda
# toolchain.
#
# "full" builds add the `map` command group (in-process GLIM SLAM); "core" builds
# omit it. `bagwiz map slam` links the GLIM stack (GTSAM + gtsam_points + glim),
# which the full builds compile once into a vendored prefix via
# scripts/build-glim-deps.sh before compiling bagwiz against it. The CPU/CUDA choice
# is derived from the environment name: a *-cuda env (humble-cuda/jazzy-cuda) builds
# CUDA, any other env builds CPU. The profiles:
#
#   * build-core (--core) in a CPU env   -> core bagwiz, NO `map`/SLAM, no GLIM
#                                           stack. The fast build. Any distro.
#   * build-core (--core) in a *-cuda env-> same core binary (core links no CUDA),
#                                           built into the *-cuda env's base; the
#                                           symmetric core entry of the cpu/cuda
#                                           matrix. humble-cuda/jazzy-cuda.
#   * build-full in a CPU env            -> CPU SLAM (-DBAGWIZ_WITH_SLAM=ON), after
#                                           building the GLIM CPU deps. humble/jazzy.
#   * build-full in a *-cuda env         -> CUDA SLAM (-DBAGWIZ_WITH_SLAM_CUDA=ON),
#                                           after building the GLIM CPU + CUDA deps.
#                                           humble-cuda/jazzy-cuda.
#
# Each pixi environment builds into its OWN base (build/<env>, install/<env>) keyed
# on $PIXI_ENVIRONMENT_NAME, so switching `-e <env>` never reuses another env's
# colcon/CMake cache (which would silently link the wrong ROS or CUDA libraries).
# build-core and build-full SHARE a base (e.g. install/humble), so every profile
# passes BAGWIZ_WITH_SLAM/_CUDA/_MAP_VIEWER explicitly: that forces the CMake cache
# to the intended profile and stops a prior full build from leaving `map` compiled
# into a later core build (or vice versa). The trailing symlink keeps
# build/compile_commands.json pointing at the last-built env for editor tooling.
#
# Intentionally NOT pixi-input-cached: the bundled GLIM step is already idempotent
# via its own stamp (build-glim-deps.sh), and an `inputs` cache keyed on src/ alone
# would report a false "cache hit" and skip the whole task even when the GLIM prefix
# is missing (e.g. after `pixi run clean`), leaving the SLAM build to fail at
# find_package(glim). colcon's own incremental build keeps a no-change rebuild fast.
set -euo pipefail

# First positional (anything not starting with `-`) is the CMake build type. The
# only flag is --core, which selects the core (no `map`/SLAM) profile; the CPU/CUDA
# profile comes from the active env, not a flag.
build_type="Release"
core=0
for arg in "$@"; do
    case "${arg}" in
    --core) core=1 ;;
    -*)
        echo "bagwiz-build: unknown flag '${arg}' (expected --core)" >&2
        exit 2
        ;;
    *) build_type="${arg}" ;;
    esac
done

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO="${PIXI_PROJECT_ROOT:-$(cd -- "${SCRIPT_DIR}/.." && pwd)}"
ENV_NAME="${PIXI_ENVIRONMENT_NAME:-humble}"

# CPU/CUDA profile is derived from the env name, not a flag: a *-cuda env builds
# CUDA, any other env builds CPU. Fail fast here (before the slow GLIM build) if a
# *-cuda env is missing its toolkit, since nvcc comes from the `gpu` feature that
# only the *-cuda environments carry.
cuda=0
if [ "${ENV_NAME%-cuda}" != "${ENV_NAME}" ]; then
    if [ ! -x "${CONDA_PREFIX:-/no-conda}/bin/nvcc" ]; then
        echo "bagwiz-build: environment '${ENV_NAME}' looks like a CUDA environment but nvcc was not found." >&2
        echo "  Make sure you are running in a *-cuda pixi environment, e.g.:" >&2
        echo "    pixi run -e humble-cuda build-full      # or jazzy-cuda" >&2
        exit 1
    fi
    cuda=1
fi

# Resolve whether to compile the `map`/SLAM command group:
#   slam=1 -> full build (map present);  slam=0 -> core build (no map).
# --core forces a core build anywhere; otherwise a full build compiles SLAM.
slam=1
if [ "${core}" -eq 1 ]; then
    slam=0
fi

cd "${REPO}"

cmake_args=(-DCMAKE_BUILD_TYPE="${build_type}" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON)

if [ "${cuda}" -eq 1 ] && [ "${core}" -eq 1 ]; then
    # build-core in a *-cuda env: a core build that targets the *-cuda env's base
    # (install/<env>), giving the cpu/cuda matrix a symmetric core entry. It links
    # NO CUDA (core has no `map`/SLAM), so it is byte-identical to build-core in a
    # CPU env -- the only difference is the install base. The *-cuda env and its
    # nvcc were already validated above.
    #
    # Core profile (no map/SLAM/CUDA), explicitly forced so a prior full build in
    # this same base cannot leave `map` compiled in.
    cmake_args+=(
        -DBAGWIZ_WITH_SLAM=OFF
        -DBAGWIZ_WITH_SLAM_CUDA=OFF
        -DBAGWIZ_WITH_MAP_VIEWER=OFF
    )
elif [ "${cuda}" -eq 1 ]; then
    # build-full in a *-cuda env: CUDA SLAM (humble-cuda/jazzy-cuda). The *-cuda env
    # and its nvcc were already validated above; here we just need the conda C++
    # compiler on $CXX (the CUDA host compiler).
    : "${CONDA_PREFIX:?build-full needs an activated pixi env (CONDA_PREFIX unset)}"
    : "${CXX:?build-full needs the conda C++ compiler on \$CXX}"
    # GLIM CPU deps first (the CUDA stack reuses this prefix's GTSAM), then the
    # CUDA deps. Both are sub-second no-ops once their stamped prefixes exist.
    bash "${SCRIPT_DIR}/build-glim-deps.sh"
    bash "${SCRIPT_DIR}/build-glim-deps.sh" --cuda
    # Pass the CUDA toolchain through environment variables instead of -D flags.
    # colcon forwards --cmake-args to every package, so -DCMAKE_CUDA_COMPILER and
    # friends would trigger "Manually-specified variables were not used" warnings
    # in packages that do not enable the CUDA language. CMake recognises CUDACXX,
    # CUDAHOSTCXX and CUDAToolkit_ROOT, and only the package that calls
    # enable_language(CUDA) / find_package(CUDAToolkit) consumes them.
    export CUDACXX="${CONDA_PREFIX}/bin/nvcc"
    export CUDAHOSTCXX="${CXX}"
    export CUDAToolkit_ROOT="${CONDA_PREFIX}"
    cmake_args+=(
        -DBAGWIZ_WITH_SLAM_CUDA=ON
        -DBAGWIZ_WITH_MAP_VIEWER=ON
        "-DCMAKE_PREFIX_PATH=${REPO}/install/${ENV_NAME}/glim-deps-cuda;${REPO}/install/${ENV_NAME}/glim-deps"
        -DCMAKE_CUDA_ARCHITECTURES=86
    )
elif [ "${slam}" -eq 1 ]; then
    # build-full (CPU SLAM).
    bash "${SCRIPT_DIR}/build-glim-deps.sh"
    cmake_args+=(
        -DBAGWIZ_WITH_SLAM=ON
        -DBAGWIZ_WITH_SLAM_CUDA=OFF
        -DBAGWIZ_WITH_MAP_VIEWER=ON
        "-DCMAKE_PREFIX_PATH=${REPO}/install/${ENV_NAME}/glim-deps"
    )
else
    # build-core (CPU): core profile, no `map`/SLAM. Force the toggles OFF so a
    # prior full build in this same base cannot leave `map` compiled into this
    # core build.
    cmake_args+=(
        -DBAGWIZ_WITH_SLAM=OFF
        -DBAGWIZ_WITH_SLAM_CUDA=OFF
        -DBAGWIZ_WITH_MAP_VIEWER=OFF
    )
fi

colcon build --symlink-install --packages-up-to bagwiz \
    --build-base "build/${ENV_NAME}" --install-base "install/${ENV_NAME}" \
    --cmake-args "${cmake_args[@]}"
ln -sfn "${ENV_NAME}/compile_commands.json" build/compile_commands.json
