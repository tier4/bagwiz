#!/usr/bin/env bash
# Verify that bags bagwiz writes can be opened by the ROS 2 distro it was built
# against, in both layouts.
#
# This guards a failure mode unit tests cannot see. rosbag2 selects its metadata
# YAML decoder from the declared `version`, and jazzy+ parses that metadata while
# probing whether a storage plugin can open the file. A document whose structure
# disagrees with its version therefore does not merely report wrong numbers — the
# bag stops being openable at all:
#
#   [ERROR] [rosbag2_storage]: No storage id specified, and no plugin found
#   that could open URI: '...'
#
# bagwiz pins `version: 5` (the humble baseline) on every distro so one output
# shape stays readable everywhere: rosbag2 reads older metadata forward but not
# newer metadata backward, and bagwiz hands its output to consumers whose distro
# it does not control.
#
# The seed bag is built with plain sqlite3 rather than `ros2 bag record` so the
# check needs no ROS graph, no DDS and no publisher.
set -euo pipefail

BAGWIZ_BIN="${BAGWIZ_BIN:-./build/${PIXI_ENVIRONMENT_NAME:-default}/bagwiz/bagwiz}"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

if [[ ! -x $BAGWIZ_BIN ]]; then
    echo "check-rosbag2-compat: bagwiz binary not found at $BAGWIZ_BIN" >&2
    exit 1
fi

SEED="$WORK/seed.db3"
python3 - "$SEED" <<'PY'
import sqlite3, sys

db = sqlite3.connect(sys.argv[1])
db.executescript("""
CREATE TABLE schema(schema_version INTEGER PRIMARY KEY, ros_distro TEXT NOT NULL);
CREATE TABLE metadata(id INTEGER PRIMARY KEY, metadata_version INTEGER NOT NULL,
                      metadata TEXT NOT NULL);
CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT NOT NULL, type TEXT NOT NULL,
                    serialization_format TEXT NOT NULL, offered_qos_profiles TEXT NOT NULL);
CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL,
                      timestamp INTEGER NOT NULL, data BLOB NOT NULL);
CREATE INDEX timestamp_idx ON messages (timestamp ASC);
INSERT INTO schema VALUES (3, 'humble');
INSERT INTO topics VALUES (1, '/chatter', 'std_msgs/msg/String', 'cdr', '');
""")
# CDR-encapsulated std_msgs/String carrying "hi".
payload = b"\x00\x01\x00\x00" + b"\x03\x00\x00\x00" + b"hi\x00"
db.executemany(
    "INSERT INTO messages(topic_id, timestamp, data) VALUES (1, ?, ?)",
    [(1_700_000_000_000_000_000 + i * 1_000_000, payload) for i in range(5)])
db.commit()
db.close()
PY

fail=0

# Indent a captured block onto stderr so failure detail reads as sub-output.
indent() { echo "      ${1//$'\n'/$'\n'      }" >&2; }

check() { # <label> <path>
    local label="$1" path="$2" out detail
    if ! out="$(ros2 bag info "$path" 2>&1)"; then
        echo "FAIL  $label: ros2 bag info exited non-zero" >&2
        indent "$out"
        fail=1
        return
    fi
    if grep -qE "No plugin detected|Exception on parsing|Error opening" <<<"$out"; then
        echo "FAIL  $label: ros2 bag info could not read the bag" >&2
        detail="$(grep -iE "error|exception|no plugin" <<<"$out" || true)"
        indent "$detail"
        fail=1
        return
    fi
    if ! grep -qE "Messages: +5( |$)" <<<"$out"; then
        echo "FAIL  $label: expected 5 messages" >&2
        indent "$out"
        fail=1
        return
    fi
    echo "ok    $label"
}

# `convert format` refuses a no-op (same storage AND same layout), so reach the
# directory bag by changing layout, then the single file by changing it back.
# Both hops are genuine bagwiz writes.
"$BAGWIZ_BIN" convert format -i "$SEED" -o "$WORK/dir" >/dev/null
check "directory bag" "$WORK/dir"

"$BAGWIZ_BIN" convert format -i "$WORK/dir" -o "$WORK/single.db3" >/dev/null
check "single-file .db3" "$WORK/single.db3"

"$BAGWIZ_BIN" convert format -i "$SEED" -o "$WORK/single.mcap" >/dev/null
# humble ships no mcap storage plugin at all. That is a distro limitation
# rather than a bagwiz regression, so probe for it and skip. Capture the output
# first: `ros2 bag info` exits non-zero here, and under `pipefail` piping it
# straight into grep would report the pipeline as failed even on a match.
mcap_probe="$(ros2 bag info "$WORK/single.mcap" 2>&1 || true)"
# Distinguish "this distro has no mcap plugin" from "the mcap plugin exists but
# rejected our file" — only the latter is a bagwiz regression. When rosbag2
# cannot match a plugin it also logs the ones it does have; humble's list has no
# mcap entry, jazzy's does.
if grep -q "No plugin detected" <<<"$mcap_probe" &&
    ! grep -q "Available storage plugins:.*mcap" <<<"$mcap_probe"; then
    echo "skip  single-file .mcap (no mcap storage plugin on this distro)"
else
    check "single-file .mcap" "$WORK/single.mcap"
fi

exit "$fail"
