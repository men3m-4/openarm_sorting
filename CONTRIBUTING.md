# Contributing

Contribution rules, the change checklist and the test commands are in
[docs/development/CONTRIBUTING.md](docs/development/CONTRIBUTING.md); the development workflow
is in [docs/development/DEVELOPMENT_GUIDE.md](docs/development/DEVELOPMENT_GUIDE.md).

In short:

* never modify `src/upstream/` (upstream OpenArm, pinned in `upstream.repos`);
* keep the demonstrated application unchanged unless a change is explicitly proposed:
  one cup, LEFT arm only, red cube then blue cube, OpenCV colour + depth perception;
* every change builds, passes `colcon test` (42 unit tests), and changes that can affect the
  robot are re-validated with `python3 tests/integration/e2e_cup_sorting.py --answer y`;
* update the documentation in `docs/` together with the code.
