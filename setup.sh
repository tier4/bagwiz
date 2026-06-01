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
# Requires a sourced ROS 2 environment (see below) and rosdep
# (`sudo apt install python3-rosdep`, then `sudo rosdep init && rosdep update`).

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Require a sourced ROS 2 environment. ROS_DISTRO alone is not enough (it can be
# exported without sourcing); AMENT_PREFIX_PATH is set when a distro's
# setup.bash is sourced, so require both. If not sourced, print the exact source
# command for each ROS 2 distro installed under /opt/ros (or, when none is
# found, tell the user to install ROS 2) and exit.
ros_install_root="${ROS_INSTALL_ROOT:-/opt/ros}"
if [[ -n ${ROS_DISTRO:-} && -n ${AMENT_PREFIX_PATH:-} ]]; then
    echo "[setup.sh] Using sourced ROS 2 distro: ${ROS_DISTRO}"
else
    installed_distros=()
    for entry in "${ros_install_root}"/*/; do
        [[ -f "${entry}setup.bash" ]] && installed_distros+=("$(basename "${entry}")")
    done
    if [[ ${#installed_distros[@]} -eq 0 ]]; then
        echo "[setup.sh] ROS 2 is not sourced and no installation was found under ${ros_install_root}." >&2
        echo "[setup.sh] Install ROS 2 first: https://docs.ros.org/en/rolling/Installation.html" >&2
        exit 1
    fi
    echo "[setup.sh] ROS 2 is not sourced. Source an installed distro, then re-run this command:" >&2
    for distro in "${installed_distros[@]}"; do
        echo "[setup.sh]     source ${ros_install_root}/${distro}/setup.bash" >&2
    done
    exit 1
fi

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
