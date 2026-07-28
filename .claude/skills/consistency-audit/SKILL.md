---
name: consistency-audit
description: >-
  Audit and fix drift between bagwiz source-code behavior (the source of truth)
  and its user-facing text: per-command and global --help, docs/commands/*.md,
  the README command table, in-source behavior comments, and the hardcoded
  tables in bagwiz/src/commands/completion.cpp. Use when asked to check or fix
  help/doc/comment inconsistencies for one command, a set of commands, or the
  whole CLI. Fans work out to one read-only subagent per command so the main
  context stays lean.
---

# bagwiz consistency-audit

Bring bagwiz's documentation, help text, and behavior comments back in line with
what the code actually does. The implementation is the ground truth; prose is
corrected to match it, never the other way around.

## Ground-truth principle

- The authoritative behavior of a command is its compiled C++: the CLI11 wiring
  in `configure*()` (flag names, positionals, `->default_val(...)`,
  `->check(CLI::IsMember(...))`, `->required()`) plus the `run*()` logic and the
  `bagwiz_*/src/core/` / `bagwiz_io/src/io/` functions it delegates to.
- Only prose may change: docs, help strings, footers, and comments. Never edit
  executable logic, flag definitions, defaults, or validators to "make the docs
  right" — if the code looks wrong, stop and report it instead of editing it.
- Comments themselves are a target, not a source. Do not trust an existing
  comment over the code it describes.

## The four drift surfaces

For any command, behavior is restated in up to four places that fall out of sync:

1. Help text — string literals inside `bagwiz/src/commands/<cmd>.cpp` `configure*()`:
   the `add_subcommand(name, DESC)` 2nd arg, the `add_option/add_flag(..., HELP)`
   3rd arg, and `sub->footer("...")`. Top-level program description is at
   `bagwiz/src/main.cpp` (`CLI::App app{"bagwiz - ..."}`) and the subcommand list is
   built from each command's `description()`.
2. Docs — `docs/commands/<cmd>.md` (note: `cam-info.md` is hyphenated) and the
   command table + examples in `README.md`.
3. Comments — class-level prose above each `Command` and rationale inside
   `run*()` in the `.cpp` files, plus contract comments in
   `include/bagwiz/commands/*.hpp` and `include/bagwiz/core/**`.
4. Completion tables — `bagwiz/src/commands/completion.cpp` re-declares each command's
   flags and supported message types as static C++ arrays, independent of
   `configure()`. This is the easiest surface to forget; always include it.

`docs/CODEMAPS/*` is generated and gitignored — never hand-edit it. If it is
stale, regenerate with the `update-codemaps` skill instead.

## Command → source / doc map

Registered commands: `cam-info`, `check`, `complete`, `convert`, `generate`,
`joke` (hidden, intentionally undocumented), `ls`, `map`, `tf`, `topic`,
`traj`, `walk`. Re-derive this list from `bagwiz/src/commands/registry.cpp` /
`name()` declarations before a full run, since it grows over time.

| Command  | Source of truth (.cpp / core)                                                                                                  | Doc                         |
| -------- | ------------------------------------------------------------------------------------------------------------------------------ | --------------------------- |
| cam-info | `cam_info.cpp`, `cam_info_replace.cpp`, `bagwiz_image/src/core/image/camera_info_resolver.cpp`                                 | `docs/commands/cam-info.md` |
| check    | `check.cpp`                                                                                                                    | `docs/commands/check.md`    |
| complete | `completion.cpp`                                                                                                               | `docs/commands/complete.md` |
| convert  | `convert.cpp`                                                                                                                  | `docs/commands/convert.md`  |
| generate | `generate.cpp`, `generate_video.cpp`                                                                                           | `docs/commands/generate.md` |
| ls       | `ls.cpp`                                                                                                                       | `docs/commands/ls.md`       |
| map      | `map.cpp`, `map_slam.cpp`, `map_viewer.cpp`                                                                                    | `docs/commands/map.md`      |
| tf       | `tf.cpp`, `tf_static_cp.cpp`, `bagwiz_tf/src/core/tf/*.cpp`                                                                    | `docs/commands/tf.md`       |
| topic    | `topic.cpp`, `topic_drop/keep/rename.cpp`, `bagwiz_base/src/core/base/topic_match.cpp`, `bagwiz_bag/src/core/bag/bag_copy.cpp` | `docs/commands/topic.md`    |
| traj     | `traj.cpp`                                                                                                                     | `docs/commands/traj.md`     |
| walk     | `walk.cpp`                                                                                                                     | `docs/commands/walk.md`     |

(All `.cpp` paths are under `bagwiz/src/commands/`.)

## Workflow

### Phase 0 — Scope

Determine the target set from the request:

- A single command (`/consistency-audit topic`) → audit just that command.
- A list (`topic, tf, map`) → those commands.
- `all` or no argument → every registered command plus the global/README surface.

A build is optional. The audit runs from source alone. Only if you want to diff
live help do you need a build: `pixi run -e humble build-core`, then
`scripts/bagwiz-run.sh <cmd> [<sub>] --help`. Treat live `--help` as a
convenience cross-check, not the ground truth — the `configure*()` source is.

### Phase 1 — Fan out one read-only audit subagent per command

Spawn the audit agents in parallel (one Agent tool block, multiple calls). Use
the `Explore` or `general-purpose` subagent type. Each agent owns exactly one
command and is told:

> Source code behavior is ground truth; do not modify anything. Read
> `bagwiz/src/commands/<cmd>.cpp` (`configure*()` + `run*()`), the delegated
> `bagwiz/src/commands/<cmd>_*.cpp` and the `bagwiz_*/src/core/` libraries it calls, and the relevant
> `include/bagwiz/**` headers. Then compare against the four drift surfaces:
> `docs/commands/<cmd>.md`, this command's row/examples in `README.md`,
> the in-source comments, and this command's tables in
> `bagwiz/src/commands/completion.cpp`. Return ONLY a findings list — no file contents.

Context discipline (this is the whole point of the skill): the main agent must
not read the full command sources itself. It consumes only the compact findings
each subagent returns. Each finding uses this shape:

```text
- surface: doc | help | comment | completion | readme
  location: <file>:<line or section>
  discrepancy: <what the prose currently claims>
  truth: <what the code actually does, with src file:line evidence>
  fix: <the exact corrected wording, or a tight description of the edit>
  severity: factual-error | omission | stale-example | wording
```

Agents return an empty list when a command is already consistent — that is a
valid, useful result.

### Phase 2 — Consolidate and dispatch fixes

1. Merge all findings. Drop anything that is style-only unless asked to include
   it; prioritize `factual-error` and `omission`.
2. If the run is non-trivial, show the consolidated findings to the developer
   before editing, grouped by command.
3. Apply fixes with writer subagents (or directly for a small set). Partition by
   FILE to avoid write conflicts:
   - Per-command files (`docs/commands/<cmd>.md` and that command's `.cpp`
     comments) are independent → safe to fix in parallel, one agent per command.
   - Shared files (`README.md`, `bagwiz/src/commands/completion.cpp`) are touched by
     many commands → batch all their edits into a single agent so two agents
     never write the same file at once.

Match existing Markdown and comment style. Keep edits minimal and factual.

### Phase 3 — Verify

- `pixi run -e humble build-core` — comment edits live in compiled sources, so this
  both proves nothing broke and catches any accidental logic edit. (Use the same
  `-e <distro>` the developer is working in.)
- If live help was used: re-run `scripts/bagwiz-run.sh <cmd> --help` and confirm
  it now matches the doc.
- Markdown hygiene on changed docs (the repo runs markdownlint, cspell,
  markdown-link-check, prettier via pre-commit) — let the pre-commit hooks run;
  do not bypass them.

### Phase 4 — Report and ship

Summarize per command: files changed and the nature of each inconsistency fixed.
Then follow the repo's git rules in `AGENTS.md`:

- Never commit to `main`. Branch first (e.g. `docs/<scope>-consistency`).
- Conventional Commits: `docs:` for doc/comment fixes, `fix:` if a
  completion-table flag list was wrong. No AI attribution.
- Open a PR; do not push to the remote or merge without explicit approval.

## Constraints checklist

- [ ] Source behavior treated as ground truth; only prose changed.
- [ ] No edits to logic, flag names, defaults, or validators.
- [ ] All four surfaces checked, including `completion.cpp`.
- [ ] `docs/CODEMAPS/*` left alone (regenerate via `update-codemaps`, not edited).
- [ ] Main context fed structured findings only, never raw command sources.
- [ ] Shared files (README, completion.cpp) edited by a single agent.
- [ ] `pixi run -e <distro> build-core` passes; pre-commit hooks not bypassed.
- [ ] English-only prose; AGENTS.md git/PR workflow followed.

## Scaling notes

- One or a few commands: fan out a handful of audit agents, fix inline. Cheap.
- `all`: that is ~11 audit agents plus a global/README pass. If the developer
  has opted into multi-agent orchestration, a Workflow that pipelines
  audit → fix per command (with a single shared-file fix stage at the end) keeps
  it deterministic; otherwise drive it with parallel Agent calls in batches.
