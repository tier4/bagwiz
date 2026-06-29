#!/usr/bin/env bash
# bagwiz-build.sh - build bagwiz via colcon, picking the {core,full} x {cpu,cuda}
# profile from flags + the active pixi environment. Invoked by the pixi build tasks
# (build-core / build-full / build-core-cuda / build-full-cuda; see pixi.toml); not
# meant to be run on its own, since it relies on the pixi environment's activated
# ROS 2 + conda toolchain.
#
# "full" builds add the `map` command group (in-process GLIM SLAM); "core" builds
# omit it. `bagwiz map slam` links the GLIM stack (GTSAM + gtsam_points + glim),
# which the full builds compile once into a vendored prefix via
# scripts/build-glim-deps.sh before compiling bagwiz against it. The four profiles:
#
#   * build-core      (--core)         -> core bagwiz, NO `map`/SLAM, no GLIM stack.
#                                         The fast build. Any distro.
#   * build-full      (no flags)       -> CPU SLAM (-DBAGWIZ_WITH_SLAM=ON), after
#                                         building the GLIM CPU deps. humble/jazzy.
#                                         lyrical falls back to a core build (it
#                                         cannot pin Eigen 3.4, so the GLIM stack is
#                                         unbuildable there).
#   * build-core-cuda (--core --cuda)  -> core bagwiz built inside a *-gpu env
#                                         (install/<env>-gpu base). Same binary as
#                                         build-core (core links no CUDA); exists for
#                                         a symmetric cpu/cuda matrix. humble-gpu/
#                                         jazzy-gpu.
#   * build-full-cuda (--cuda)         -> CUDA SLAM (-DBAGWIZ_WITH_SLAM_CUDA=ON),
#                                         after building the GLIM CPU + CUDA deps.
#                                         humble-gpu/jazzy-gpu.
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

# First positional (anything not starting with `-`) is the CMake build type. Flags:
# --core selects the core (no `map`/SLAM) profile; --cuda selects the CUDA env/SLAM
# profile. Accept them in any order.
build_type="Release"
core=0
cuda=0
for arg in "$@"; do
    case "${arg}" in
    --core) core=1 ;;
    --cuda) cuda=1 ;;
    -*)
        echo "bagwiz-build: unknown flag '${arg}' (expected --core and/or --cuda)" >&2
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

# Resolve whether to compile the `map`/SLAM command group:
#   slam=1 -> full build (map present);  slam=0 -> core build (no map).
# --core forces a core build anywhere. A full build wants SLAM, but lyrical cannot
# build the GLIM stack (it cannot pin Eigen 3.4), so a full build there falls back
# to core with a notice.
slam=1
if [ "${core}" -eq 1 ]; then
    slam=0
elif [ "${DISTRO}" = "lyrical" ]; then
    slam=0
    echo "[bagwiz-build] lyrical cannot build the GLIM/SLAM stack (it cannot pin Eigen 3.4)," >&2
    echo "[bagwiz-build]   so this 'full' build produces a core (no 'map' command) bagwiz." >&2
fi

cd "${REPO}"

cmake_args=(-DCMAKE_BUILD_TYPE="${build_type}" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON)

if [ "${cuda}" -eq 1 ] && [ "${core}" -eq 1 ]; then
    # build-core-cuda: a core build that targets a *-gpu env's base (install/<env>),
    # giving the cpu/cuda matrix a symmetric core entry. It links NO CUDA (core has
    # no `map`/SLAM), so it needs no toolkit -- but it MUST run in a *-gpu env, else
    # it is byte-identical to build-core and the name is misleading.
    if [ "${ENV_NAME%-gpu}" = "${ENV_NAME}" ]; then
        echo "build-core-cuda must run in a *-gpu environment (it targets install/${ENV_NAME})." >&2
        echo "  For a core build in a CPU environment, use build-core instead:" >&2
        echo "    pixi run -e ${DISTRO} build-core" >&2
        echo "  Or run this in a *-gpu env:" >&2
        echo "    pixi run -e humble-gpu build-core-cuda      # or jazzy-gpu" >&2
        exit 1
    fi
    # Core profile (no map/SLAM/CUDA), explicitly forced so a prior full-cuda build
    # in this same base cannot leave `map` compiled in.
    cmake_args+=(
        -DBAGWIZ_WITH_SLAM=OFF
        -DBAGWIZ_WITH_SLAM_CUDA=OFF
        -DBAGWIZ_WITH_MAP_VIEWER=OFF
    )
elif [ "${cuda}" -eq 1 ]; then
    # build-full-cuda: CUDA SLAM. Unsupported wherever SLAM is (lyrical).
    if [ "${slam}" -eq 0 ]; then
        echo "build-full-cuda: SLAM (and thus the CUDA build) is unsupported on '${DISTRO}'." >&2
        echo "  Use a SLAM distro: pixi run -e humble-gpu build-full-cuda   # or jazzy-gpu" >&2
        exit 1
    fi
    : "${CONDA_PREFIX:?build-full-cuda needs an activated pixi env (CONDA_PREFIX unset)}"
    : "${CXX:?build-full-cuda needs the conda C++ compiler on \$CXX}"
    # Fail fast (BEFORE the slow CPU GLIM build) when this env has no CUDA toolkit,
    # i.e. it is not a *-gpu env. nvcc comes from the `gpu` feature, which only the
    # *-gpu environments carry; a bare `pixi run build-full-cuda` lands in `default`
    # (= humble, no CUDA) and could otherwise waste a tens-of-minutes GLIM build
    # before build-glim-deps.sh --cuda rejects it.
    if [ ! -x "${CONDA_PREFIX}/bin/nvcc" ]; then
        echo "build-full-cuda: no CUDA toolkit in this environment ('${ENV_NAME}')." >&2
        echo "  The -cuda tasks (build-full-cuda/install-cuda/run-cuda/test-cuda) carry CUDA" >&2
        echo "  only in a *-gpu environment. Re-run the SAME task with -e, e.g.:" >&2
        echo "    pixi run -e humble-gpu build-full-cuda      # or install-cuda / run-cuda / test-cuda" >&2
        echo "    pixi run -e jazzy-gpu  build-full-cuda      # jazzy-gpu works too" >&2
        exit 1
    fi
    # GLIM CPU deps first (the CUDA stack reuses this prefix's GTSAM), then the
    # CUDA deps. Both are sub-second no-ops once their stamped prefixes exist.
    bash "${SCRIPT_DIR}/build-glim-deps.sh"
    bash "${SCRIPT_DIR}/build-glim-deps.sh" --cuda
    cmake_args+=(
        -DBAGWIZ_WITH_SLAM_CUDA=ON
        -DBAGWIZ_WITH_MAP_VIEWER=ON
        "-DCMAKE_PREFIX_PATH=${REPO}/install/${ENV_NAME}/glim-deps-cuda;${REPO}/install/${ENV_NAME}/glim-deps"
        -DCMAKE_CUDA_COMPILER="${CONDA_PREFIX}/bin/nvcc"
        "-DCMAKE_CUDA_HOST_COMPILER=${CXX}"
        -DCMAKE_CUDA_ARCHITECTURES=86
        -DCUDAToolkit_ROOT="${CONDA_PREFIX}"
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
    # build-core (CPU), or a full build on lyrical: core profile, no `map`/SLAM.
    # Force the toggles OFF so a prior full build in this same base cannot leave
    # `map` compiled into this core build.
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
