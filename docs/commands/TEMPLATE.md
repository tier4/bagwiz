# Command documentation template

Every command under `bagwiz/src/commands/` has a page at
`docs/commands/<cmd>.md` (hyphenated command name, e.g. `cam-info.md`). This
file is the single source of truth for how those pages are structured and
written. Follow it when writing a new page or revising an existing one.

Exemplars to mirror: [`topic.md`](topic.md) (command group) and
[`trim.md`](trim.md) (single-action command).

## Reading flow

Pages are written for two passes: a reader first skims `Examples` to pick up
how the command is used in practice, then consults the `Options` table for
details on the specific flags they need. Write to that flow:

- `Examples` is the teaching surface. Cover the realistic scenarios — the
  common case first, then the variations a user is likely to want — rather
  than one invocation per flag.
- The `Options` table is the reference surface. Each Description must stand
  on its own: constraints, defaults, interactions with other flags, and
  failure modes belong there, so a reader who jumps straight to a flag gets
  the full story without hunting through prose sections.
- Do not hide information a user needs in long prose sections. If a detail
  belongs to one flag, put it in that flag's Description; reserve prose
  sections for concepts that span several flags.

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

## Exit status

| Code | Meaning                              |
| ---- | ------------------------------------ |
| `0`  | Success.                             |
| `1`  | Failed — check stderr for the cause. |
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
  when globs appear. When a flag accepts several values per occurrence
  (e.g. `--pcd`, `-t/--topics`, `--color`), pass them in a single occurrence
  (`--pcd a b c`), not as repeated flag-value pairs (`--pcd a --pcd b`).
  Placed right after `Usage`, so a reader sees a working
  invocation before the full flag reference.
- `Options` — a two-column table, `| Flag | Description |`. The Flag cell
  lists the short and long forms with a value placeholder: `` `-i`,
`--input <input>` ``; note "Long-form only." when there is no short form.
  Start the Description with `**Required.**` for required flags. Give
  defaults inline as `Default: <value>.` — do not add a Default column.
  Keep each Description to the key points a user needs to pick and use the
  flag — at most ~800 characters even for the most involved flag. If a
  description grows past that, the detail belongs in a semantics section
  (linked from the Description), not in the table cell.
- Command-specific semantics — optional sections after Options for anything
  the user must understand beyond the flag reference: selector syntax, window
  semantics, frame conventions, guarantees, edge cases, and failure modes,
  each verifiable in the code. Name them after the concept, not "Notes" or
  "Behavior".
- `Exit status` — a short `| Code | Meaning |` table and nothing more:
  `0` = success, `1` = failed (stderr names the cause). Do not enumerate
  individual failure paths — the detail lives in the error messages and the
  sections above.
- Optional trailing sections as needed: `Output` / `Outputs`, `Errors`
  (situation → outcome table), `Performance`, `Environment` (env vars),
  `Requirements`. Place them before `Exit status`.

## Figures

Some concepts are easier to see than to read — 3D geometry (frames, axes,
trajectories, deskew relationships) above all. When an explanation needs
more than a sentence or two of spatial prose, generate an SVG figure and
embed it instead of writing the prose out:

- Commit the SVG under `docs/commands/assets/` named `<cmd>-<topic>.svg`
  and embed it with a relative link: `![<alt>](assets/<cmd>-<topic>.svg)`.
- Write plain, hand-readable SVG: vector shapes and real `<text>` only — no
  scripts, no embedded raster images — kept small and legible at the size it
  renders on GitHub, on both light and dark backgrounds (favor `currentColor`
  or mid-tone colors over pure black/white).
- A figure replaces the prose it explains; keep the surrounding text to a
  caption-level sentence. Do not duplicate the figure's content as text.
- Figures remain the exception, not the rule: process and architecture
  diagrams stay Mermaid (see `AGENTS.md`), and anything that explains well
  in one or two sentences stays text.
