#!/usr/bin/env bash
# setup.sh - Install ROS package dependencies for building bagwiz.
#
# Run after cloning the repository. Resolves bagwiz's declared dependencies
# (package.xml) via rosdep and installs the latest versions available for the
# sourced ROS distro. Does not import third-party message sources: for extra
# message types (e.g. domain-specific bags), install those packages via apt or
# build them in another workspace and source that overlay before running
# bagwiz.
#
# Despite the filename, this script is meant to be executed, not sourced.
#
# ROS sourcing: if ROS is already sourced the active distro is used. Otherwise
# the installed distros are listed and you are prompted to choose one (see
# ros-source.sh). Install rosdep with `sudo apt install python3-rosdep` and
# initialise it once with `sudo rosdep init && rosdep update`.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=ros-source.sh
source "${SCRIPT_DIR}/ros-source.sh"
ensure_ros_sourced setup.sh

if ! command -v rosdep >/dev/null 2>&1; then
    echo "[setup.sh] 'rosdep' is not installed or not on PATH." >&2
    exit 1
fi

echo "[setup.sh] Installing latest available dependencies for ${ROS_DISTRO} via rosdep"
rosdep install \
    --from-paths "${SCRIPT_DIR}" \
    --ignore-src \
    --rosdistro "${ROS_DISTRO}" \
    -r -y

echo "[setup.sh] Done. Build with ./build.sh"
