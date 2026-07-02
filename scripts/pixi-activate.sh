# shellcheck shell=sh
# Sourced by pixi on environment activation (see [target.unix.activation] in
# pixi.toml), AFTER RoboStack's own conda activation has pointed ROS_DISTRO,
# AMENT_PREFIX_PATH, CMAKE_PREFIX_PATH and LD_LIBRARY_PATH inside the pixi
# environment ($CONDA_PREFIX). pixi keeps the environment variables this script
# exports.
#
# It does one optional, guarded thing (a no-op before the first build):
#   Source bagwiz's own colcon overlay (install/setup.sh) once built, so the
#   freshly built `bagwiz` binary lands on PATH inside the environment.
#
# To use custom message packages, source the workspace's install/setup.bash
# before running bagwiz.
#
# Written in POSIX shell (sourced setup.sh, not setup.bash) so it works whether
# pixi runs activation scripts under sh or bash.

# Workspace root: pixi exposes it under different names across versions.
_bagwiz_ws="${PIXI_PROJECT_ROOT:-${PIXI_WORKSPACE_ROOT:-$PWD}}"
# Each distro installs into its own base (install/<distro>); see [tasks.build-full].
_bagwiz_env="${PIXI_ENVIRONMENT_NAME:-default}"

# bagwiz's own overlay for this distro (exists only after `pixi run build-full`).
if [ -f "${_bagwiz_ws}/install/${_bagwiz_env}/setup.sh" ]; then
    # shellcheck disable=SC1090,SC1091
    . "${_bagwiz_ws}/install/${_bagwiz_env}/setup.sh"
fi

unset _bagwiz_ws _bagwiz_env
