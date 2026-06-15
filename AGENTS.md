# AGENTS.md

Guidelines for AI agents contributing to this repository.

The rules below are split into two top-level groups:

- **General Rules** apply to any contribution and are not specific
  to bagwiz's domain or codebase. Most would carry over unchanged
  to another repository.
- **Project-Specific Rules** apply only to this repository —
  bagwiz's build workflow, its CLI surface, and similar choices
  that would not generalize.

The final section ("Maintaining These Guidelines") describes how
to keep this file itself consistent over time.

## General Rules

### 1. Code & Documentation

- Always write source code and documentation in English.
- When modifying source code, update the corresponding documentation
  (e.g. `README.md`, command-line help text) to reflect the changes.
- When writing documentation, always consult the corresponding source
  code so that the description does not drift from the actual
  behavior. Verify claims against the implementation rather than
  relying on memory or assumptions.
- Use bold (`**...**`) and emoji sparingly in Markdown
  (documentation, PR descriptions, commit message bodies). Reserve
  bold for genuinely critical warnings or terms that must stand out
  from surrounding prose; do not bold ordinary keywords, type names,
  or section labels. Use emoji only when it adds clear meaning or
  improves readability, and avoid decorative emoji in source comments,
  generated documentation, commit messages, and pull request
  descriptions.
- Never write phrases that only make sense within the context of an
  AI–developer conversation. A future contributor or user reading the
  source must be able to understand the reasoning from the code and
  documentation alone. Concretely, do not embed shorthand references
  to design-discussion artifacts that are not in-tree — e.g. "plan A",
  "option B", "decision 3/D", "project decision 12/A", "Q6 decision",
  "per the design doc", or numbered selections from a chat-mode
  question/answer. Such labels are meaningless to anyone who did not
  participate in the conversation. Instead, spell out the rationale
  inline — what the rule is and why — so the comment, log message, or
  documentation stands on its own. If a deeper write-up exists, link
  to a tracked, in-repo document (e.g. an ADR file under `docs/`),
  not to a private conversation.

### 2. Attribution

- Do NOT include AI agent signatures (e.g. `Co-Authored-By: <agent name> ...`)
  in any generated code, commit messages, pull request descriptions,
  documentation, or other output.

### 3. Commit Messages & Branch Names

- Follow the [Conventional Commits](https://www.conventionalcommits.org/)
  specification for every commit message. Use one of the standard types
  (`feat`, `fix`, `refactor`, `test`, `docs`, `chore`, `perf`, `ci`,
  `style`, `build`, `revert`) with an optional scope, e.g.
  `feat(bag): add multi-topic inspect`.
- Use the same type as the branch prefix when creating a branch for a
  pull request (e.g. `feat/multi-topic-inspect`, `fix/hesai-sop-order`).
  Do not use tool- or author-specific prefixes such as `claude/*`.

### 4. Pre-commit Hooks

- Before committing, inspect staged and unstaged changes and ensure the
  commit does not include secrets, credentials, private keys, tokens,
  `.env` files, large generated or binary artifacts, personal
  information, or other files that should not be published to GitHub. If
  any such file is present or ambiguous, stop and warn the developer
  instead of adding it to the commit.
- Do not bypass pre-commit hooks (e.g. `--no-verify`) and do not
  disable a check globally. When a hook reports an error, fix the
  underlying issue and re-commit rather than skipping the hook to
  push work through. The only exception is a genuine false positive:
  suppress it narrowly at the offending site (e.g. an inline
  `// NOLINT(...)`, `// cppcheck-suppress`, or equivalent
  hook-specific directive scoped to the smallest unit possible).

### 5. GitHub Actions / CI

- Be mindful of the GitHub Actions workflows configured in this
  repository; ensure changes do not cause them to fail.
- When investigating workflow failures, use the `gh` command
  (e.g. `gh run view`, `gh run view --log-failed`) to retrieve and read
  the actual logs. Base bug fixes on evidence from those logs, not on
  assumptions.

### 6. Remote Repository Operations

- Always obtain explicit developer approval before making any changes
  to the remote repository — pushing commits, creating/closing pull
  requests or issues, commenting on PRs, and so on.
- Every modification to the existing codebase, no matter how trivial,
  must go through a pull request. Do not push directly to `main` or
  any other shared branch — open a PR first, even for small fixes
  such as typo corrections, formatting, or one-line changes.
- Never merge a pull request unless every required CI check has
  completed successfully. If any CI job is failing, pending, or
  skipped in a way that bypasses required checks, investigate and fix
  the underlying issue before merging — do not merge to "unblock" the
  branch or rely on follow-up PRs to clean up red CI.
- Write PR descriptions that are comprehensive and detailed, yet
  concise: cover the problem, the solution, and the test plan without
  unnecessary verbosity.

### 7. Resource Management

- When writing code that acquires or releases a resource (memory, file
  handles, sockets, mutex locks, terminal modes, ROS handles, etc.),
  follow the RAII principle as much as possible: tie the resource's
  lifetime to a stack object whose constructor acquires it and whose
  destructor releases it. Prefer standard wrappers (`std::unique_ptr`,
  `std::lock_guard`, `std::fstream`, ...) or a small purpose-built
  guard type over manual `new` / `delete`, paired `open` / `close`
  calls scattered through the body, or `try` / `catch` blocks whose
  only job is cleanup.

## Project-Specific Rules

Each subsection below scopes its rules to a specific surface of
the repository. A rule applies only to the surface named by its
subsection — for example, the conventions under "bagwiz CLI"
govern only the CLI surface and do not extend to internal C++
APIs, build scripts, or any other code that is not part of the
CLI.

### 1. Build Workflow

Applies when building bagwiz from this repository. Does not
govern source code or CLI behavior.

- bagwiz is built and run through [pixi](https://pixi.sh); no system ROS 2
  install is required. pixi provisions ROS 2 from RoboStack (one conda channel
  per distro) and the C/C++ toolchain from conda-forge. Build with
  `pixi run -e <distro> build` from the repository root, where `<distro>` is one
  of `humble`, `jazzy`, or `lyrical`; a bare `pixi run build` targets
  the default environment (Jazzy). Each distro builds into its own
  `build/<distro>` and `install/<distro>`, so switching distros never reuses
  another distro's CMake cache.
- Run the tests with `pixi run -e <distro> test` and the built binary with
  `pixi run -e <distro> run -- <args>` (or `pixi shell -e <distro>` then
  `bagwiz`). `./install.sh` installs an optional `bagwiz` launcher on `PATH`
  that delegates to pixi (`./install.sh --help` covers its options).
- Prefer the `pixi run` tasks over ad-hoc `colcon build` invocations when
  verifying changes, unless you are reproducing a CI or tooling issue that
  requires a different command line. The task definitions live in `pixi.toml`.

### 2. bagwiz CLI

Applies only to the `bagwiz` executable and the subcommands defined
under `src/commands/`. The rules below govern the user-facing CLI
surface — argument naming, ordering, help text, and similar concerns
— and do not apply to internal C++ APIs, library headers under
`include/`, build scripts, tests, or any other code that is not
part of the CLI itself.

- Order positional arguments on every `bagwiz` subcommand to follow
  common Unix utility conventions: read-side / source operands come
  first and the write-side / destination operand comes last,
  mirroring `cp src dst`, `mv src dst`, and `ln target name`.
  POSIX.1-2017 Base Definitions, Chapter 12 ("Utility Conventions",
  XBD §12) defines the surrounding option/operand syntax (option
  placement, the `--` separator, and so on); the
  source-before-destination ordering itself is not a separately
  published standard but the de facto convention codified by the
  POSIX utility specifications (`cp`, `mv`, `ln`, `install`, ...)
  and reinforced by the GNU Coding Standards. When a bagwiz
  subcommand must deviate — for example, when a single operand
  acts as both source and destination, or when the read-side
  selector is more naturally placed last — state the reasoning in
  that subcommand's help text so users do not have to infer it
  from the signature.

## Maintaining These Guidelines

- Keep the rules in this file free of duplication. Each topic should
  have a single source of truth: if two or more bullets or sections
  cover the same ground, consolidate them into one authoritative rule
  rather than restating the same guidance in multiple places.
- Before adding a new rule, scan the existing sections first. If the
  new guidance fits an existing rule, extend that rule in place;
  otherwise, place it under the section whose scope it most clearly
  belongs to instead of creating a parallel rule elsewhere. When the
  new rule does not yet fit any section, decide first whether it is
  general or bagwiz-specific and add it under the matching top-level
  group.
- When editing this file, also check whether the change makes any
  pre-existing rule redundant. If it does, update or remove the
  now-overlapping text in the same change so the ruleset stays
  minimal and non-redundant.
- Keep the rules in this file mutually consistent. Before adding or
  modifying a rule, read the surrounding sections to make sure the
  new wording does not contradict any existing rule. If a genuine
  conflict is unavoidable (for example, the new rule is meant to
  supersede an older one), resolve it in the same change by updating
  or removing the conflicting rule rather than leaving both in place.
