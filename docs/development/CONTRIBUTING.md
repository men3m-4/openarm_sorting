# Contributing

Thank you for improving the OpenArm cup-sorting project. Please read
[DEVELOPMENT_GUIDE.md](DEVELOPMENT_GUIDE.md) first.

## Before you change something

* Know which package owns the behaviour ([../architecture/PACKAGE_ARCHITECTURE.md](../architecture/PACKAGE_ARCHITECTURE.md)).
  Perception logic belongs in `openarm_sorting_vision`, task and motion logic in
  `openarm_sorting_manipulation`, startup in `openarm_sorting_bringup`, the simulated robot in
  `openarm_sorting_description`.
* Do not modify `src/upstream/`. Upstream updates are done by changing the commit in
  `upstream.repos`, re-importing and re-running all tests.
* The demonstrated application is fixed: one cup, both cubes into it, LEFT arm only, red then
  blue, OpenCV colour + depth perception, cup pose from configuration. Changing any of these
  is a feature change and must be proposed and documented as such.

## Checklist for a change

1. Build: `colcon build --symlink-install` without new warnings in project packages.
2. Unit tests pass: `colcon test --packages-select openarm_sorting_description openarm_sorting_vision openarm_sorting_manipulation openarm_sorting_bringup && colcon test-result --verbose`.
3. Add or update unit tests for new pure-Python logic.
4. If the change can affect the robot (perception results, planning, grasping, placement,
   timing, launch order): run `python3 tests/integration/e2e_cup_sorting.py --answer y` and
   `--answer n`, and record the result in
   [../architecture/VALIDATION_REPORT.md](../architecture/VALIDATION_REPORT.md).
5. Parameters: new tuning values go to the package's YAML with an identical code default.
6. Documentation: update the affected document(s) in `docs/` and, for user-visible changes,
   the README.
7. Style: `python3 -m flake8 --max-line-length 120 --select E,W,F <changed files>`.

## Commit messages

Imperative mood, one subject line (<= 72 characters), body explaining *why*. Mention the
validation that was run.
