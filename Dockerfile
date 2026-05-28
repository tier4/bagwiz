# Build and run bagwiz with ROS 2 and the message packages it depends on
# baked into the image, so the host needs no ROS installation. The ROS build
# toolchain is kept in the final image on purpose: it lets users build their
# own message packages into a colcon overlay at runtime (see the README) and
# have bagwiz decode bags that use those types.
#
# Select the ROS 2 distribution with --build-arg ROS_DISTRO=humble|jazzy.
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

# The base image already exports ROS_DISTRO at runtime; re-declaring the ARG
# here makes it available to the build instructions below.
ARG ROS_DISTRO

# Optional release compile-flag tweaks forwarded by "build.sh --docker"
# (e.g. "--native --unroll"). Empty by default so CI and a plain "docker build"
# produce a portable, release image. See build.sh for the flag definitions.
ARG BAGWIZ_BUILD_FLAGS=""

WORKDIR /opt/bagwiz_ws

# Place the sources in a colcon "src" layout.
COPY . src/bagwiz

# Install bagwiz's declared ROS dependencies (the message packages that ship
# in the image) plus bash-completion (so interactive shells load completions),
# then build bagwiz in release mode. Kept in a single layer so the apt package
# lists are removed without lingering in an earlier layer.
#
# The rosdep install step is delegated to the repository's setup.sh so the
# image and a native checkout resolve dependencies the exact same way. setup.sh
# assumes rosdep is already updated, so "rosdep update" stays here.
#
# The colcon build is likewise delegated to build.sh so the image and a native
# checkout compile bagwiz identically. build.sh expects ROS to be sourced (done
# on the preceding line) and writes the install tree to the current WORKDIR,
# which the entrypoint sources. --native/--unroll passed to "build.sh --docker"
# arrive here through the BAGWIZ_BUILD_FLAGS build arg.
#
# The completion script is written to the system-wide bash-completion directory
# rather than through "bagwiz complete bash --install": --install targets a
# per-user $HOME path, but the container runs as an arbitrary host uid with no
# fixed HOME, so a system path is what makes completion work for every user
# from the first interactive shell. The install space is sourced first so the
# bagwiz executable is on PATH and can emit its own completion script.
#
# The base image ships bash-completion but leaves the loader commented out in
# /etc/bash.bashrc, so the loader block is appended there to make interactive
# shells source the framework (which in turn sources the script installed
# below).
#
# SC2086 is ignored because ${BAGWIZ_BUILD_FLAGS} is intentionally left unquoted
# so that a multi-flag value like "--native --unroll" word-splits into separate
# arguments for build.sh.
# hadolint ignore=SC1091,DL3008,SC2086
RUN apt-get update \
    && apt-get install -y --no-install-recommends bash-completion \
    && rosdep update --rosdistro "${ROS_DISTRO}" \
    && bash src/bagwiz/setup.sh \
    && rm -rf /var/lib/apt/lists/* \
    && . "/opt/ros/${ROS_DISTRO}/setup.sh" \
    && bash src/bagwiz/build.sh ${BAGWIZ_BUILD_FLAGS} \
    && . /opt/bagwiz_ws/install/setup.sh \
    && mkdir -p /etc/bash_completion.d \
    && bagwiz complete bash >/etc/bash_completion.d/bagwiz \
    && printf '%s\n' \
        '# Load programmable completion for interactive shells (bagwiz image).' \
        'if ! shopt -oq posix && [ -f /usr/share/bash-completion/bash_completion ]; then' \
        '  . /usr/share/bash-completion/bash_completion' \
        'fi' >>/etc/bash.bashrc

COPY --chmod=0755 docker/entrypoint.sh /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
