# Pinned dependencies (`deps/`)

These lock files pin the exact apt versions of bagwiz's declared build
dependencies (from [`../package.xml`](../package.xml)), one per supported ROS
distro:

- `humble.lock`
- `jazzy.lock`

They are generated and committed by maintainers and shipped to everyone, so
every `./setup.sh` installs the same versions. **Users do nothing extra** —
`setup.sh` picks up the lock for the active `ROS_DISTRO` automatically.

Each line is `name=version`; `#` lines are comments.

## How `setup.sh` uses a lock

`setup.sh` installs the pinned versions with a single
`apt-get install name=version ...` call. It changes **no** system apt
configuration (no `sources.list`, no `preferences.d`, no `apt-mark hold`), so
the pin applies to that install only and leaves no residue. Only the directly
declared packages are pinned — never base system libraries — so there is no
risk of unsafe downgrades. If no lock exists for the active distro, `setup.sh`
falls back to a plain `rosdep install`. CI runs the same `setup.sh`, so the
build/test workflow exercises the pinned versions users actually install.

## Regenerating the locks (maintainers)

> **Maintainers only.** `deps/lock-deps.sh` is a maintainer tool — end users
> never run it. Users only run `./setup.sh`, which consumes the committed
> locks. Generating locks is the project's responsibility so that everyone
> installs the same versions.

Run `deps/lock-deps.sh` against each distro's apt repository. The canonical,
machine-independent way is the official ROS images, from the repo root:

```bash
for distro in humble jazzy; do
  docker run --rm -e HOST_UID="$(id -u)" -e HOST_GID="$(id -g)" \
    -v "$PWD":/ws -w /ws "ros:${distro}" bash -c '
      apt-get update -qq && rosdep update --rosdistro "$ROS_DISTRO" \
      && ./deps/lock-deps.sh \
      && chown "$HOST_UID:$HOST_GID" "deps/$ROS_DISTRO.lock"'
done
git add deps/*.lock
git commit -m "chore(deps): bump pinned dependency versions"
```

`lock-deps.sh` reads the dependency keys from `package.xml`, resolves each to
its apt package via `rosdep`, and pins it to the version the repository
currently offers (apt "Candidate"). Bump whenever you want to move the pins
forward.

## Upstream retention

`packages.ros.org` keeps only the latest version of each package, so a pin can
eventually disappear upstream and `setup.sh` then fails to install it.

**When that happens it is a maintainer's job to fix it, not the user's.** For a
single removed version, edit the affected line in `deps/<distro>.lock`
directly: replace the unavailable version with one the repository currently
offers (`apt-cache policy <name>` prints the candidate), then commit. Reach for
`deps/lock-deps.sh` when you want to refresh the whole set rather than patch one
package.

If you need pins that survive upstream changes entirely, point that single
`apt-get` call at a dated `snapshots.ros.org` repo via
`apt-get -o Dir::Etc::SourceList=...`, still without touching system apt
configuration.
