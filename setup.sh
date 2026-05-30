#!/usr/bin/env bash
# setup.sh - Install ROS package dependencies for building bagwiz.
#
# Run after cloning the repository. Does not import third-party message
# sources: for extra message types (e.g. domain-specific bags), install
# those packages via apt or build them in another workspace and source that
# overlay before running bagwiz.
#
# Version pinning: if deps/<ROS_DISTRO>.lock exists, its pinned versions are
# installed with a single `apt-get install name=version` call. That changes
# no system apt configuration (no sources.list / preferences / hold), so the
# pin applies to this install only and leaves no residue. Without a lock the
# script falls back to a plain `rosdep install` (latest available). See
# deps/README.md and regenerate locks with deps/lock-deps.sh.
#
# Despite the filename, this script is meant to be executed, not sourced.
#
# Requires:
#   - ROS_DISTRO sourced (e.g. `source /opt/ros/humble/setup.bash`)
#   - rosdep (initialised with `sudo rosdep init && rosdep update`)

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ -z ${ROS_DISTRO:-} ]]; then
    echo "[setup.sh] ROS_DISTRO is not set. Source your ROS environment first." >&2
    exit 1
fi

if ! command -v rosdep >/dev/null 2>&1; then
    echo "[setup.sh] 'rosdep' is not installed or not on PATH." >&2
    exit 1
fi

LOCK_FILE="${SCRIPT_DIR}/deps/${ROS_DISTRO}.lock"

if [[ -f ${LOCK_FILE} ]]; then
    echo "[setup.sh] Installing pinned dependency versions from deps/${ROS_DISTRO}.lock"
    # Read "name=version" entries, ignoring comment and blank lines.
    mapfile -t pinned < <(grep -vE '^[[:space:]]*(#|$)' "${LOCK_FILE}")
    if [[ ${#pinned[@]} -eq 0 ]]; then
        echo "[setup.sh] ${LOCK_FILE} has no package entries." >&2
        exit 1
    fi
    # Pinned install in a single apt-get call. This deliberately changes no
    # system apt configuration (no sources.list, preferences, or holds), so
    # the version pin applies to this install only and leaves no residue.
    apt_get=(apt-get)
    if [[ ${EUID} -ne 0 ]]; then
        apt_get=(sudo apt-get)
    fi
    "${apt_get[@]}" install -y --no-install-recommends --allow-downgrades "${pinned[@]}"
else
    echo "[setup.sh] No pinned lock for ${ROS_DISTRO}; installing latest available via rosdep"
    rosdep install \
        --from-paths "${SCRIPT_DIR}" \
        --ignore-src \
        --rosdistro "${ROS_DISTRO}" \
        -r -y
fi

echo "[setup.sh] Done. Build with ./build.sh"
