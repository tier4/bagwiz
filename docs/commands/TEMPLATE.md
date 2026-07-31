# Command documentation template

Every command under `bagwiz/src/commands/` has a page at
`docs/commands/<cmd>.md` (hyphenated command name, e.g. `cam-info.md`). This
file is the single source of truth for how those pages are structured and
written. Follow it when writing a new page or revising an existing one.

Exemplars to mirror: [`topic.md`](topic.md) (command group) and
[`trim.md`](trim.md) (single-action command).

## Ground rules

- The compiled behavior is the ground truth. Read the command's `configure*()`
  (flag names, defaults, `->required()`, `->check(...)`) and `run*()` before
  writing, and verify every claim against the code — never document from
  memory or from the existing prose.
- Describe only current behavior. No migration notes, upgrade guides, or
  "renamed from" callouts — bagwiz has no formal releases, so there is nothing
  to migrate from.
- Match the CLI11 help text, the command's row in `README.md`, and the flag
  tables in `bagwiz/src/commands/completion.cpp`. When behavior changes, all
  four surfaces change together (the `consistency-audit` skill checks this).
- English only. Bold and emoji sparingly (see `AGENTS.md`).

## Page skeleton

Single-action command (`ls`, `trim`, `walk`, ...):

````markdown
# `bagwiz <cmd>`

One or two sentences: what the command does, in plain terms.

## Usage

```text
bagwiz <cmd> -i <input> [OPTIONS]
```

## Examples

```bash
# One comment line per invocation, describing the scenario.
bagwiz <cmd> -i drive.mcap -o out/
```

## Options

| Flag                    | Description                            |
| ----------------------- | -------------------------------------- |
| `-i`, `--input <input>` | Input ROS 2 rosbag. Must exist.        |
| `-o`, `--output <p>`    | **Required.** Output path. Default: -. |

## <command-specific semantics> (optional; e.g. "Time window semantics")

## Behavior

- One bullet per guaranteed behavior, edge case, or failure mode.

## Exit status

| Code | Meaning |
| ---- | ------- |
| `0`  | ...     |
| `1`  | ...     |
````

Command group (`topic`, `tf`, `map`, ...):

```markdown
# `bagwiz <cmd>`

One sentence, then the subcommand table:

| Subcommand                     | What it does          |
| ------------------------------ | --------------------- |
| [`<sub>`](#bagwiz-<cmd>-<sub>) | One-line description. |

---

## `bagwiz <cmd> <sub>`

Intro paragraph, then the same sections as a single-action command, one
heading level deeper: `### Usage`, `### Options`, ..., with a `---` rule
between subcommand sections. A single shared `## Exit status` table at the
end may cover all subcommands when they share the same codes.
```

## Section rules

- `Usage` — a `text` fenced block, one line, exactly the flag spellings from
  `configure()`. Required flags listed explicitly, optional ones collapsed
  into `[OPTIONS]`. No positional operands (bagwiz takes all operands as
  flags).
- `Examples` — always plural, even for a single invocation. A `bash` fenced
  block where every invocation gets one `#` comment line above it describing
  the scenario. Quote globs (`'/sensing/*'`) and say so once below the block
  when globs appear. Placed right after `Usage`, so a reader sees a working
  invocation before the full flag reference.
- `Options` — a two-column table, `| Flag | Description |`. The Flag cell
  lists the short and long forms with a value placeholder: `` `-i`,
`--input <input>` ``; note "Long-form only." when there is no short form.
  Start the Description with `**Required.**` for required flags. Give
  defaults inline as `Default: <value>.` — do not add a Default column.
- Command-specific semantics — optional sections between Options and Behavior
  for anything the user must understand before the behavior list (selector
  syntax, window semantics, frame conventions). Name them after the concept,
  not "Notes".
- `Behavior` — bullets (or a numbered list for an ordered pipeline) covering
  guarantees, edge cases, and failure modes, each verifiable in the code.
- `Exit status` — a `| Code | Meaning |` table. `0` is success; `1` covers
  runtime failures of the command itself. CLI parse errors (missing required
  flags, failed validation) do not reach the command — CLI11 exits with its
  own codes — so do not list them under `1`; add one sentence saying so when
  the distinction matters.
- Optional trailing sections as needed: `Output` / `Outputs`, `Errors`
  (situation → outcome table), `Performance`, `Environment` (env vars),
  `Requirements`. Place them after `Behavior`, before `Exit status`.
