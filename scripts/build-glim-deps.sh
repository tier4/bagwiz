#!/usr/bin/env bash
# build-glim-deps.sh - build the GLIM SLAM stack into a vendored prefix.
#
# `bagwiz map slam` links GLIM (koide3/glim) plus its heavy dependencies GTSAM and
# gtsam_points. None of the three are packaged on conda-forge/robostack at the
# versions GLIM needs (GTSAM ships only as alpha git tags, and gtsam_points
# bundles patched iSAM2 sources that must match), so they are built from source.
#
# Rather than rebuild this multi-ten-minute stack on every `colcon build`, it is
# built ONCE into install/<distro>/glim-deps and then `find_package`d by bagwiz
# (Approach B - a vendored prefix). This script is run inside the pixi environment
# by `pixi run -e <distro> build-full` (which in a *-cuda env also passes --cuda)
# via scripts/bagwiz-build.sh, so the conda toolchain (gcc, cmake, ninja)
# and the env-provided Boost / Eigen / fmt / spdlog / OpenMP are already on the
# relevant paths.
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

# The active pixi environment (humble/jazzy/...). Falls back to humble
# for a bare invocation outside `pixi run`.
ENV_NAME="${PIXI_ENVIRONMENT_NAME:-humble}"

# Args (any order): --force rebuilds from scratch; --cuda builds the optional
# GPU stack (gtsam_points + glim with CUDA) into a SEPARATE prefix so the default
# CPU prefix (built by `pixi run build-full`) stays byte-identical and GPU-free.
force=0
cuda=0
for arg in "$@"; do
    case "${arg}" in
    --force) force=1 ;;
    --cuda) cuda=1 ;;
    *)
        echo "build-glim-deps: unknown argument '${arg}' (expected --force and/or --cuda)" >&2
        exit 2
        ;;
    esac
done

if [ "${cuda}" -eq 1 ]; then
    # The GPU stack must build in a *-cuda pixi environment (humble-cuda / jazzy-cuda):
    # the env name is the install base (install/<env>), kept separate from the CPU
    # build, and it carries the conda CUDA toolkit. Running this under pixi in a CPU
    # env (e.g. `pixi run build-full` without `-e <distro>-cuda`) would share the
    # CPU build base + cache and fail later with a confusing "no CUDA target" CMake
    # error, so stop now with a clear message. A bare run outside pixi
    # (PIXI_ENVIRONMENT_NAME unset) is left to the caller + BAGWIZ_CUDA_HOME.
    if [ -n "${PIXI_ENVIRONMENT_NAME:-}" ] && [ "${PIXI_ENVIRONMENT_NAME%-cuda}" = "${PIXI_ENVIRONMENT_NAME}" ]; then
        echo "build-glim-deps --cuda must run in a *-cuda pixi environment" \
            "(it ran in '${PIXI_ENVIRONMENT_NAME}')." >&2
        echo "  Use:  pixi run -e humble-cuda build-full   # or: jazzy-cuda" >&2
        exit 1
    fi
    # CUDA toolkit root, by priority: an explicit BAGWIZ_CUDA_HOME > the active pixi
    # env's conda CUDA ($CONDA_PREFIX/bin/nvcc, installed by the `gpu` feature) > a
    # system /usr/local/cuda-12.8. So in a *-cuda pixi env CUDA is fully pixi-managed;
    # BAGWIZ_CUDA_HOME still lets a system toolkit be used. NOTE: do NOT trust the
    # env's CUDA_HOME — a user profile may point it at an incompatible system CUDA
    # (e.g. 12.6, which rejects gcc 14). nvcc 12.8 is required for the gcc 14.3 host.
    if [ -n "${BAGWIZ_CUDA_HOME:-}" ]; then
        cuda_root="${BAGWIZ_CUDA_HOME}"
    elif [ -x "${CONDA_PREFIX:-}/bin/nvcc" ]; then
        cuda_root="${CONDA_PREFIX}"
    else
        cuda_root="/usr/local/cuda-12.8"
    fi
    CUDA_ARCH="86" # RTX 3080 Ti = sm_86; single-arch (no multiarch fatbin) = smaller .so, faster build.
    PREFIX="${REPO}/install/${ENV_NAME}/glim-deps-cuda"
    SRC="${REPO}/build/${ENV_NAME}/glim-src-cuda"
    # The CUDA build REUSES this CPU prefix's GTSAM (GTSAM is CUDA-agnostic), skipping
    # the multi-ten-minute GTSAM cold build. `pixi run build-full` (CPU) makes it.
    CPU_PREFIX="${REPO}/install/${ENV_NAME}/glim-deps"
    WANT="gtsam=${GTSAM_REF}(reuse) gtsam_points=${GTSAM_POINTS_REF}+cuda${CUDA_ARCH} glim=${GLIM_REF}+gpu"
else
    PREFIX="${REPO}/install/${ENV_NAME}/glim-deps"
    SRC="${REPO}/build/${ENV_NAME}/glim-src"
    WANT="gtsam=${GTSAM_REF} gtsam_points=${GTSAM_POINTS_REF} glim=${GLIM_REF}"
fi
STAMP="${PREFIX}/.glim-deps.stamp"

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

# CUDA flag arrays consumed by the gtsam_points / glim cmake calls below. Empty of
# CUDA in the default (CPU) mode, so those builds are byte-for-byte unchanged.
gp_cuda=(-DBUILD_WITH_CUDA=OFF)
glim_cuda=(-DBUILD_WITH_CUDA=OFF)
if [ "${cuda}" -eq 1 ]; then
    # Reuse the CPU prefix's GTSAM (the long pole); fail clearly if it is absent.
    if [ ! -f "${CPU_PREFIX}/lib/cmake/GTSAM/GTSAMConfig.cmake" ]; then
        echo "build-glim-deps --cuda reuses GTSAM from ${CPU_PREFIX}, but it is missing." >&2
        echo "  Build the CPU deps first: pixi run -e ${ENV_NAME} build-full" >&2
        exit 1
    fi
    if [ ! -x "${cuda_root}/bin/nvcc" ]; then
        echo "build-glim-deps --cuda: nvcc not found at ${cuda_root}/bin/nvcc" >&2
        echo "  In a *-cuda pixi env nvcc comes from the conda env; otherwise set BAGWIZ_CUDA_HOME." >&2
        exit 1
    fi
    # Override any stale system CUDA_HOME/CUDA_PATH (a user profile may export an
    # incompatible toolkit) so nvcc and every tool agree on the chosen one.
    export PATH="${cuda_root}/bin:${PATH}" CUDACXX="${cuda_root}/bin/nvcc"
    export CUDA_HOME="${cuda_root}" CUDA_PATH="${cuda_root}"
    # Resolve the reused GTSAM at configure time (gtsam_points/glim find_package(GTSAM)).
    export CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}:${CPU_PREFIX}"
    # nvcc drives the conda g++ host pass; point it at the conda sysroot so the C++
    # stdlib headers resolve (the classic nvcc-in-a-conda-env failure surface).
    export CONDA_BUILD_SYSROOT="${CONDA_BUILD_SYSROOT:-${CONDA_PREFIX:-}/x86_64-conda-linux-gnu/sysroot}"
    cuda_common=(
        -DBUILD_WITH_CUDA=ON -DBUILD_WITH_CUDA_MULTIARCH=OFF
        -DCMAKE_CUDA_COMPILER="${cuda_root}/bin/nvcc"
        -DCMAKE_CUDA_HOST_COMPILER="${CXX_BIN}"
        -DCMAKE_CUDA_ARCHITECTURES="${CUDA_ARCH}"
        -DCUDAToolkit_ROOT="${cuda_root}"
        -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON
    )
    gp_cuda=("${cuda_common[@]}")
    glim_cuda=("${cuda_common[@]}")
fi

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

# 1) GTSAM - headless, deterministic (no TBB), use the env's Eigen. CUDA mode
#    reuses the CPU prefix's GTSAM (it is CUDA-agnostic) and skips this long build.
if [ "${cuda}" -eq 0 ]; then
    clone https://github.com/borglab/gtsam "${GTSAM_REF}" gtsam
    cmake -S "${SRC}/gtsam" -B "${SRC}/gtsam/build" -G Ninja \
        -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
        -DGTSAM_BUILD_UNSTABLE=ON -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_WITH_TBB=OFF \
        -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_PYTHON=OFF \
        -DGTSAM_BUILD_DOCS=OFF -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DBUILD_SHARED_LIBS=ON
    cmake --build "${SRC}/gtsam/build" --target install -j"${jobs}"
else
    echo "build-glim-deps --cuda: reusing GTSAM from ${CPU_PREFIX} (skipping the GTSAM build)"
fi

# 2) gtsam_points - CPU only, OpenMP on.
clone https://github.com/koide3/gtsam_points "${GTSAM_POINTS_REF}" gtsam_points
cmake -S "${SRC}/gtsam_points" -B "${SRC}/gtsam_points/build" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
    "${gp_cuda[@]}" -DBUILD_WITH_OPENMP=ON -DBUILD_TESTS=OFF -DBUILD_DEMO=OFF \
    -DBUILD_EXAMPLE=OFF -DBUILD_TOOLS=OFF
cmake --build "${SRC}/gtsam_points/build" --target install -j"${jobs}"

# 3) glim - headless CPU. Two idempotent, version-tolerant source patches:
#
#   a) fmt-12 compat: the env ships fmt 12, whose fmt::ptr() rejects smart
#      pointers (moved to <fmt/std.h>); pass the raw pointer instead.
#
#   b) carry intensities: OdometryEstimationCT rebuilds the estimation frame
#      from raw points and adds only times/normals/covs, dropping the per-point
#      intensities the preprocessor extracted. Without this, intensity never
#      reaches sub/global mapping and the exported map (GlobalMapping::
#      export_points) is xyz-only. Copy intensities onto the frame right after
#      the times so they propagate through to `bagwiz map slam`.
clone https://github.com/koide3/glim "${GLIM_REF}" glim
ct="${SRC}/glim/src/glim/odometry/odometry_estimation_ct.cpp"
sed -i \
    -e 's/fmt::ptr(frames\[last\])/fmt::ptr(frames[last].get())/' \
    -e 's/fmt::ptr(frames\[last - 1\])/fmt::ptr(frames[last - 1].get())/' \
    "${ct}"
grep -q 'add_intensities(raw_frame->intensities)' "${ct}" || sed -i \
    's#\(frame_cpu->add_times(raw_frame->times);\)#\1\n  if (!raw_frame->intensities.empty()) frame_cpu->add_intensities(raw_frame->intensities);#' \
    "${ct}"
cmake -S "${SRC}/glim" -B "${SRC}/glim/build" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
    "${glim_cuda[@]}" -DBUILD_WITH_VIEWER=OFF -DBUILD_WITH_OPENCV=OFF
cmake --build "${SRC}/glim/build" --target install -j"${jobs}"

# CUDA mode: confirm the GPU library was produced and links the CUDA runtime
# before stamping, so a silent CUDA-off fallback (glim auto-disables CUDA when
# gtsam_points lacks it) cannot masquerade as a successful GPU build.
if [ "${cuda}" -eq 1 ]; then
    cuda_lib="${PREFIX}/lib/libgtsam_points_cuda.so"
    if [ ! -f "${cuda_lib}" ]; then
        echo "build-glim-deps --cuda: expected ${cuda_lib} was not produced (CUDA likely off)" >&2
        exit 1
    fi
    echo "build-glim-deps --cuda: $(basename "${cuda_lib}") linkage (expect libcudart + a CUDA RUNPATH/RPATH):"
    if command -v readelf >/dev/null 2>&1; then
        readelf -d "${cuda_lib}" | grep -E 'NEEDED.*cudart|RUNPATH|RPATH' ||
            echo "  (no matching NEEDED/RPATH entries found)"
    else
        echo "  (readelf not available; skipping linkage check)"
    fi
fi

echo "${WANT}" >"${STAMP}"
echo "build-glim-deps: done -> ${PREFIX}"
