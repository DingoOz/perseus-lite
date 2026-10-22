---
name: cherry-pick-upstream
description: Safely pull specific commits from the upstream ROAR-QUTRC/perseus-v2 repo into this hard-diverged perseus-lite fork WITHOUT merging (a merge resurrects the v2-only packages Phase 4 deleted). Use when bringing an upstream fix or feature into the fork, or whenever tempted to run `git merge upstream/...`.
---

# Pulling from upstream (perseus-v2) — cherry-pick, never merge

perseus-lite has **hard-diverged** from `ROAR-QUTRC/perseus-v2`: v2-only ROS
packages, firmware, hardware designs, and challenge docs were **deleted**, not
disabled. `git merge upstream/main` would re-introduce all of it. So:

## The rule

**Never `git merge upstream/main` (or rebase onto it).** Cherry-pick individual
commits instead.

## Procedure

```bash
git fetch upstream
git log --oneline HEAD..upstream/main          # see what's new upstream
git checkout -b feature/<name>                 # work on a branch off main
git cherry-pick <sha> [<sha> ...]              # pull the specific commit(s)
```

## Conflict resolution

If a cherry-picked commit touches paths this fork **deleted**, you'll get
add/delete or content conflicts. **Keep the deletions** — `git rm` the paths
during resolution rather than restoring them:

```bash
git rm <resurrected-path>
git cherry-pick --continue
```

Cross-check §4 "REMOVED" in `CLAUDE.md` for the full list of deleted paths
(perseus, perseus_hardware, perseus_can_if, perseus_payloads, perseus_simulation,
perseus_description, all of firmware/, hi-can libs, Livox/fast-lio, etc.).

## After

- Verify with the `verify-perseus` skill (clean rebuild + tests) — an upstream
  commit may assume a package or dependency this fork no longer has.
- If the remote is still named `upstream`, consider
  `git remote rename upstream upstream-archive` locally so a reflexive
  `git pull upstream main` can't undo the divergence.

## Related

The always-on guardrail lives in `CLAUDE.md` §6; this skill is the detailed
procedure.
