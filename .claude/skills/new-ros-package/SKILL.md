---
name: new-ros-package
description: Scaffold a new ROS 2 package in the perseus-lite Pixi/RoboStack workspace with the env-specific gotchas pre-solved — a pytest suite that actually runs under colcon, ruff/ament_flake8 quote reconciliation, ament_copyright headers, the lint trio, and CI wiring. Use when adding a package under software/ros_ws/src/ (especially ament_python), or when `colcon test` reports "NO TESTS RAN" / a new package's tests silently don't run.
---

# Scaffolding a ROS 2 package in this workspace

The RoboStack Jazzy env has several traps that make a "working" package silently
skip its tests or fail CI. This skill encodes the fixes. Canonical live example:
`software/ros_ws/src/perseus_lite_tui/`. Bugs behind these rules are in
`ERRORS.md` (2026-07-03, 2026-07-07).

## Decide the build type

- **ament_python** — Python nodes/tools (mirror `perseus_lite_tui`,
  `input_devices`). Most of the traps below apply here.
- **ament_cmake** — C++ (mirror `perseus_sensors`, `perseus_lite_hardware`).
  C++20, `ament_lint_auto` + gtest; the pytest traps don't apply.

The repo default/preferred language is C++ (`docs/.../ros-templates.md`); use
Python where it clearly fits.

## ament_python checklist (the trap-laden path)

Create `software/ros_ws/src/<pkg>/` with:

1. **`package.xml`** (format 3): `<exec_depend>` only what you import at
   runtime. **Do NOT depend on packages skipped in the default/ML build envs**
   (perseus_lite_simulation, perseus_vision, perseus_lite_missions) — colcon's
   ament_python hook sources the dep's install space and the build fails on a
   clean tree (ERRORS.md 2026-07-06). Test deps: ament_copyright, ament_flake8,
   ament_pep257, python3-pytest.

2. **`setup.py`**: **declare the test dep via `extras_require={'test': ['pytest']}`,
   NEVER `tests_require=`.** `tests_require` is removed by setuptools ≥68 (stderr
   warning) AND colcon only selects its pytest runner when a `test` extra/dep is
   present — without it colcon falls back to unittest and reports
   `NO TESTS RAN` (exit 5) while looking green-ish.

3. **`setup.cfg`**: `script_dir`/`install_scripts` → `$base/lib/<pkg>`.

4. **`pytest.ini`** (REQUIRED in this env):
   ```ini
   [pytest]
   addopts = -p no:launch_testing -p no:launch_ros
   ```
   The env's `launch_testing`/`launch_ros` pytest plugins are incompatible with
   its pytest 9 and abort EVERY pytest run before collection. These disable them
   (colcon honours addopts via rootdir discovery).

5. **`ruff.toml`** (only if the package's tests run `ament_flake8`):
   ```toml
   [format]
   quote-style = "single"
   ```
   `ament_flake8` enforces flake8-quotes Q000 (single quotes); the repo formatter
   `ruff format` defaults to double. Without this they fight over every string.
   Docstrings stay `"""` — flake8-quotes wants that too. **Consequence: write the
   package's code single-quoted.** (Packages built only outside the host colcon —
   e.g. an in-container package — have no ament_flake8 test, so they use ruff's
   default double quotes; don't add this file there.)

6. **`resource/<pkg>`** — empty ament index marker.

7. **Copyright header** on every `.py` (ament_copyright accepts this MIT variant):
   ```python
   # Copyright (c) 2026 Nigel Hungerford-Symes
   #
   # Use of this source code is governed by an MIT-style
   # license that can be found in the LICENSE file or at
   # https://opensource.org/licenses/MIT.
   ```

8. **`test/test_{copyright,flake8,pep257}.py`** — copy structurally from
   `perseus_lite_tui/test/`. Note ament imports come before `pytest` (google
   import order). Both linters ignore D100–D107, so missing docstrings are fine,
   but any docstring you write must obey D213 (multi-line summary NOT on line 1 —
   simplest is single-line summaries, extra prose in `#` comments).

## Wire it in

- **pixi test task**: add `<pkg>` to the default `test` task's
  `--packages-select` in `pixi.toml` (the `-R 'Test\.'` ctest filter doesn't
  affect ament_python — pytest runs unfiltered). CI (`all.yaml` →
  `pixi run -e default test`) then covers it on x86 + ARM.
- **CLAUDE.md**: add a KEEP-table row (§4) and bump the `colcon list` count in
  the first-time checklist (§8).

## Verify (the exit criteria)

```bash
cd software/ros_ws
colcon build --symlink-install --packages-select <pkg>
colcon test --packages-select <pkg> --return-code-on-test-failure
colcon test-result --verbose --test-result-base build/<pkg>
```

Confirm the count is **non-zero** and the pytest output shows `configfile:
pytest.ini` with neither launch plugin loaded — a green summary alone is NOT
proof the tests ran (see ERRORS.md 2026-07-07). Then `pixi run -e format fmt`.
