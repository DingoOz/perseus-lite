---
name: log-error
description: Maintain the ERRORS.md defect log for perseus-lite — check existing prevention rules before editing a file, and append a correctly-formatted entry after fixing a non-trivial bug. Use after fixing a compilation/runtime/config bug or a mis-applied pattern, or before modifying a file that may carry a logged prevention rule.
---

# Working with ERRORS.md

The repo requires logging defects to `ERRORS.md` before a fix is "done", and
reading prevention rules before touching a file. This skill is both halves.

## Before editing (prevention check)

When about to modify a file, scan `ERRORS.md` for entries whose **File(s)** or
**Pattern** touch it, and follow their **Prevention rule**. There are 15+ entries;
common load-bearing ones:

- New ament_python package → `-p no:launch_testing` + `extras_require['test']`
  (2026-07-07); see `new-ros-package` skill.
- New committed dotfile (`.env`, etc.) → confirm `git ls-files` actually tracks
  it; broad `.gitignore` rules silently drop it (2026-07-07).
- Pixi GUI tool segfault → env contamination, not a code bug (2026-07-03/05);
  see `triage-gui-segfault` skill.
- New cross-package `exec_depend` → check every env's `--packages-skip` list and
  verify with a fully clean rebuild (2026-07-06).
- Rename/delete migration → `grep -rn` the exact old identifier across the WHOLE
  repo, not just the directory you're in (2026-07-04).

## After fixing (append an entry)

Prepend under the header (newest first) using this exact format:

```markdown
### [SHORT_TITLE] — [YYYY-MM-DD]

- **Severity:** Critical | High | Medium | Low
- **Category:** Build | Logic | Type System | Concurrency | Memory | Configuration | API Misuse | Convention | Other
- **File(s):** `path/a`, `path/b`
- **Pattern:** The *general* error pattern, written to match FUTURE code (not
  "fixed line 42").
- **Root cause:** 1–2 sentences on why it happened.
- **Fix applied:** 1–2 sentences on what changed.
- **Prevention rule:** A concrete, actionable rule to avoid this class next time.
```

### What to log vs skip

- **Log**: compile error from wrong code (not a typo in a line you just wrote), a
  runtime/logic bug found in testing, a mis-applied convention, a non-obvious
  dependency/config fix, a regression.
- **Skip**: trivial same-line typos, throwaway/scratch code.

### Severity guide

Critical = crash/data-loss/security · High = wrong behaviour reaching prod ·
Medium = build/test failure with a non-obvious fix · Low = style/minor.

### Upkeep

When ERRORS.md passes ~20 entries, add/refresh a `## Summary` at the top
grouping recurring categories and root causes.
