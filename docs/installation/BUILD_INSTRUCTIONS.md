# Build instructions

Prerequisites: [UBUNTU_SETUP.md](UBUNTU_SETUP.md) (ROS 2 Jazzy, rosdep, colcon).

## 1. Get the sources

The workspace already contains the upstream repositories under `src/upstream/`. On a fresh
machine (e.g. after copying only the project packages), restore them at the pinned commits:

```bash
cd ~/openarm_sorting_ws_clean
mkdir -p src/upstream
vcs import src/upstream < upstream.repos
```

## 2. Install dependencies

```bash
cd ~/openarm_sorting_ws_clean
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
rosdep check --from-paths src --ignore-src      # expect: All system dependencies have been satisfied
```

## 3. Build

Use a terminal in which **only** `/opt/ros/jazzy` is sourced (no other workspace).

```bash
cd ~/openarm_sorting_ws_clean
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

Expected: `Summary: 10 packages finished` (about 20 s on the validation machine). Two upstream
packages print compiler warnings (`openarm_can`: unused parameter, `openarm_hardware`:
deprecated `on_init` overload); they are upstream warnings and harmless.

`--symlink-install` links Python modules, launch files and configuration into `install/`,
so edits to them take effect without rebuilding. Rebuild after adding files, changing
`setup.py` / `package.xml`, or changing C++ code.

Only the project packages:

```bash
colcon build --symlink-install --packages-select \
    openarm_sorting_description openarm_sorting_vision openarm_sorting_manipulation openarm_sorting_bringup
```

## 4. Source

```bash
source ~/openarm_sorting_ws_clean/install/setup.bash
ros2 pkg list | grep openarm_sorting     # 4 packages
ros2 pkg prefix openarm_description      # must point into openarm_sorting_ws_clean/install
```

## 5. Test

```bash
cd ~/openarm_sorting_ws_clean
colcon test --packages-select openarm_sorting_description openarm_sorting_vision \
    openarm_sorting_manipulation openarm_sorting_bringup
colcon test-result --verbose              # expect: 42 tests, 0 errors, 0 failures
```

System tests (start Gazebo; see [../../tests/README.md](../../tests/README.md)):

```bash
python3 tests/integration/e2e_cup_sorting.py --answer n     # prompt "no" path, ~1 min
python3 tests/integration/e2e_cup_sorting.py --answer y     # full sort, ~2 min
python3 tests/integration/subsystem_launch_test.py          # individual launch files
```

## 6. Clean

```bash
cd ~/openarm_sorting_ws_clean
rm -rf build install log
```
