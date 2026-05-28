#!/usr/bin/env bash
# Container entrypoint for bagwiz.
#
# Sources the ROS underlay (which provides the message packages baked into the
# image), then the bagwiz install space (which puts the bagwiz executable on
# PATH), then an optional user-supplied colcon overlay mounted at /overlay
# (which contributes additional, e.g. custom, message types). Finally it runs
# the requested command.
set -e

# Give ROS and colcon a writable HOME when the container runs with --user and
# no matching entry in /etc/passwd.
export HOME="${HOME:-/tmp}"

# shellcheck disable=SC1090,SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck disable=SC1091
source "/opt/bagwiz_ws/install/setup.bash"

if [ -f "/overlay/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "/overlay/install/setup.bash"
fi

exec "$@"
