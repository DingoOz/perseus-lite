---
name: verify-perseus
description: Run the full pre-PR verification for the perseus-lite workspace — clean rebuild, the env-matrix build/test, formatting, and docs — before committing or opening a PR. Use before opening a PR, when asked "is this ready to merge", or after a change that could affect builds/tests across the Pixi environments.
---

# Pre-PR verification for perseus-lite

Run the checks that mirror CI (`.github/workflows/all.yaml`) so a PR goes up
green the first time. Scale to what changed — you don't need the ML env for a
docs-only change.

## 1. Clean rebuild (do NOT skip on a real change)

```bash
cd software/ros_ws
rm -rf build install log
pixi run -e default build-test
```

The clean wipe is a rule, not a nicety: a stale `install/` from prior env
switching masks missing-dependency bugs (ERRORS.md 2026-07-06 — a package with a
cross-package `exec_depend` built only because an earlier env had left the dep in
`install/`). CI runs on a fresh checkout and will catch what an incremental
build hides.

## 2. Tests (the env matrix)

```bash
pixi run -e default test          # ctest subset + ament_python pytest suites
```

If you touched simulation or vision, also:

```bash
pixi run -e simulation build-test && pixi run -e simulation test   # linux-64 only
```

For a new ament_python package, confirm its test COUNT is non-zero
(`colcon test-result --verbose`) — see the `new-ros-package` skill; a green
summary can hide a silent unittest fallback.

## 3. Formatting (CI gate)

```bash
pixi run -e format fmt-check      # treefmt --ci: ruff, clang-format, yamlfmt,
                                  # shellcheck, taplo, prettier, actionlint
```

If it flags files, `pixi run -e format fmt` to fix, then re-check.

## 4. Docs (if you added/changed a page)

```bash
pixi run -e docs docs             # slow; new pages auto-appear via glob toctrees
```

Local Sphinx can fail on stale `docs/build/doxygen` state (exhale XML parse
error) that CI doesn't hit — if it errors there and you didn't touch C++/doxygen,
`rm -rf docs/build` and rebuild, or just trust CI's `Build docs` job.

## 5. Housekeeping before the PR

- **ERRORS.md**: if you fixed a non-trivial bug, log it (see `log-error` skill).
- **Branch**: work on `feature/<name>` off `main`, not on the fork's other
  in-flight branches (rebase onto `origin/main` if you branched off one).
- **PR command**: `gh pr create --repo DingoOz/perseus-lite ...` — the `upstream`
  remote hijacks the default repo, so ALWAYS pass `--repo`.
- **No AI attribution** in commit messages / PR titles / bodies (global rule).
