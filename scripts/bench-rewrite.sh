#!/usr/bin/env bash
# Reproducible benchmark for bagwiz's read -> process -> write rewrite loop.
#
# Decomposes wall-clock into read+decompress vs write for the pure-copy trio
# (topic drop / keep / rename) plus a read-only baseline and a sparse-extraction
# case, on a real (multi-GB) MCAP bag. The read/write split is taken from the
# env-gated BAGWIZ_PROFILE per-stage report (authoritative, in-process timing);
# /usr/bin/time -v supplies CPU% and peak RSS. This establishes the single-core
# baseline that the pipeline acceleration backends are measured against.
#
# Usage:
#   scripts/bench-rewrite.sh [BAG_PATH]
#
# Environment overrides:
#   BAG    input bag — required; single-file or rosbag2 directory; pass as $1 or
#          set BAG (a multi-GB bag gives meaningful read/write numbers)
#   ENV    pixi environment / build to use (default: default)
#   RUNS   warm timed runs per scenario for the median (default: 3)
#   OUTDIR scratch dir for rewrite outputs (default: /tmp/bagwiz-bench)
#   TOPIC  small latched topic used by the drop/keep cases (default: /tf_static)
#
# The drop/keep scenarios assume $TOPIC exists in the bag; bagwiz errors out
# clearly if it does not.
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO" || exit 1

BAG="${1:-${BAG:?set BAG or pass an input bag path as the first argument}}"
ENV="${ENV:-default}"
RUNS="${RUNS:-3}"
OUTDIR="${OUTDIR:-/tmp/bagwiz-bench}"
TOPIC="${TOPIC:-/tf_static}"
BIN="install/$ENV/bagwiz/bin/bagwiz"
TIMEV=/usr/bin/time

[ -f "$BAG" ] || [ -d "$BAG" ] || {
    echo "bench: bag not found: $BAG" >&2
    exit 1
}
[ -x "$BIN" ] || {
    echo "bench: binary not found ($BIN); run: pixi run -e $ENV build" >&2
    exit 1
}
mkdir -p "$OUTDIR"

inbytes=$(du -sb "$BAG" | cut -f1) # total bytes; handles single-file and directory bags
gitrev=$(git rev-parse --short HEAD 2>/dev/null || echo '?')

# Describe the input bag's recorded compression for the banner. rosbag2 stores it
# in metadata.yaml (directory bags); a standalone bag file carries no sidecar, so
# fall back to a storage-level note rather than guessing.
meta=""
{ [ -d "$BAG" ] && [ -f "$BAG/metadata.yaml" ]; } && meta="$BAG/metadata.yaml"
[ -f "$BAG.metadata.yaml" ] && meta="$BAG.metadata.yaml"
if [ -n "$meta" ]; then
    format=$(grep -m1 'compression_format:' "$meta" | sed -E 's/.*compression_format:[[:space:]]*"?([^"]*)"?.*/\1/')
    mode=$(grep -m1 'compression_mode:' "$meta" | sed -E 's/.*compression_mode:[[:space:]]*"?([^"]*)"?.*/\1/')
    if [ -z "$format" ] || [ "$mode" = "NONE" ]; then
        comp="uncompressed"
    else
        comp="$format-compressed ($mode mode)"
    fi
else
    comp="compression per storage format"
fi

echo "================================================================"
echo "bagwiz rewrite benchmark   $(date)"
echo "host : $(nproc) cores   git $gitrev   env $ENV"
echo "bag  : $BAG"
echo "size : $(du -h "$BAG" | cut -f1)  ($inbytes bytes, $comp)"
echo "runs : $RUNS warm timed run(s) per scenario; median reported"
echo "================================================================"

# Median wall-clock (seconds) over $RUNS executions of the command string "$*".
# Note: each run pays pixi activation overhead, identical across scenarios, so
# the A/B decomposition is a cross-check; the BAGWIZ_PROFILE split below is the
# authoritative in-process read/write breakdown.
median_wall() {
    local times=() t0 t1 i
    for ((i = 0; i < RUNS; i++)); do
        t0=$(date +%s.%N)
        pixi run -e "$ENV" bash -c "$*" >/dev/null 2>&1
        t1=$(date +%s.%N)
        times+=("$(awk -v a="$t0" -v b="$t1" 'BEGIN{print b-a}')")
    done
    printf '%s\n' "${times[@]}" | sort -g |
        awk '{v[NR]=$1} END{print (NR%2)?v[(NR+1)/2]:(v[NR/2]+v[NR/2+1])/2}'
}

# One BAGWIZ_PROFILE run; echoes the 5-line per-stage report.
profile_run() {
    pixi run -e "$ENV" bash -c "BAGWIZ_PROFILE=1 $*" 2>&1 | sed -n '/profile \[/,+4p'
}

# CPU% and peak RSS from one /usr/bin/time -v run.
resource_run() {
    pixi run -e "$ENV" bash -c "$TIMEV -v $* 2>'$OUTDIR/.time' >/dev/null" >/dev/null 2>&1
    grep -E 'Percent of CPU|Maximum resident' "$OUTDIR/.time" | sed 's/^[[:space:]]*/  /'
}

echo
echo "### warming OS page cache ..."
pixi run -e "$ENV" bash -c "$BIN check broken --deep '$BAG'" >/dev/null 2>&1

# [A] read-only baseline: how much of the time is just read + decompress.
echo
echo "===== [A] READ+DECOMPRESS only  (check broken --deep) ====="
A=$(median_wall "$BIN check broken --deep '$BAG'")
echo "  median wall: ${A}s"

# [B] full rewrite (pure copy): drop a tiny latched topic -> near-full passthrough.
DROP_OUT="$OUTDIR/drop.mcap"
echo
echo "===== [B] FULL REWRITE  (topic drop $TOPIC -o) ====="
B=$(median_wall "$BIN topic drop '$BAG' -t '$TOPIC' -o '$DROP_OUT' --overwrite")
echo "  median wall: ${B}s"
profile_run "$BIN topic drop '$BAG' -t '$TOPIC' -o '$DROP_OUT' --overwrite"
resource_run "$BIN topic drop '$BAG' -t '$TOPIC' -o '$DROP_OUT' --overwrite"
outbytes=$(stat -c %s "$DROP_OUT" 2>/dev/null || echo 0)
echo "  output: $(du -h "$DROP_OUT" 2>/dev/null | cut -f1) ($outbytes bytes, uncompressed)  expansion=$(awk -v i="$inbytes" -v o="$outbytes" 'BEGIN{if(i>0)printf "%.2fx",o/i; else print "n/a"}')"

# [B'] backend comparison: the same full rewrite under each selectable backend
# (BAGWIZ_BACKEND). Proves the threaded PipelinedBackend's output is byte
# -identical to the SequentialBackend oracle (md5) and quantifies the speedup.
echo
echo "===== [B'] BACKEND COMPARISON  (full rewrite per BAGWIZ_BACKEND) ====="
declare -A BK_WALL BK_MD5
for bk in sequential pipelined; do
    BK_OUT="$OUTDIR/drop.$bk.mcap"
    BK_WALL[$bk]=$(median_wall "env BAGWIZ_BACKEND=$bk $BIN topic drop '$BAG' -t '$TOPIC' -o '$BK_OUT' --overwrite")
    BK_MD5[$bk]=$(md5sum "$BK_OUT" 2>/dev/null | awk '{print $1}')
    echo "  --- $bk: median wall ${BK_WALL[$bk]}s   md5 ${BK_MD5[$bk]}"
    profile_run "env BAGWIZ_BACKEND=$bk $BIN topic drop '$BAG' -t '$TOPIC' -o '$BK_OUT' --overwrite"
    resource_run "env BAGWIZ_BACKEND=$bk $BIN topic drop '$BAG' -t '$TOPIC' -o '$BK_OUT' --overwrite"
done
if [ -n "${BK_MD5[sequential]}" ] && [ "${BK_MD5[sequential]}" = "${BK_MD5[pipelined]}" ]; then
    echo "  byte-identical output across backends (md5 match): OK"
else
    echo "  OUTPUT DIFFERS across backends (md5 mismatch) -- correctness bug!" >&2
fi
awk -v s="${BK_WALL[sequential]}" -v p="${BK_WALL[pipelined]}" \
    'BEGIN{ if(p>0) printf "  speedup (sequential/pipelined): %.2fx\n", s/p }'
rm -f "$OUTDIR/drop.sequential.mcap" "$OUTDIR/drop.pipelined.mcap"

# [C] sparse extraction: keep only the tiny latched topic -> read-bound floor.
KEEP_OUT="$OUTDIR/keep.mcap"
echo
echo "===== [C] SPARSE EXTRACT  (topic keep $TOPIC -o) ====="
C=$(median_wall "$BIN topic keep '$BAG' -t '$TOPIC' -o '$KEEP_OUT' --overwrite")
echo "  median wall: ${C}s"
profile_run "$BIN topic keep '$BAG' -t '$TOPIC' -o '$KEEP_OUT' --overwrite"

echo
echo "===== DECOMPOSITION (wall-clock cross-check) ====="
awk -v A="$A" -v B="$B" 'BEGIN{
  w=B-A; if(w<0)w=0;
  printf "  read+decompress (A)   : %.2f s\n", A;
  printf "  write+overhead  (B-A) : %.2f s\n", w;
  printf "  full rewrite    (B)   : %.2f s\n", B;
  if(B>0){printf "  read share            : %.0f%%\n",100*A/B; printf "  write share           : %.0f%%\n",100*w/B}
}'

rm -f "$DROP_OUT" "$KEEP_OUT" "$OUTDIR/.time"
echo
echo "### done $(date)"
