# Package architecture

Diagram: [diagrams/package_dependencies.md](diagrams/package_dependencies.md)

## 1. Workspace layout

```text
openarm_sorting_ws_clean/
├── src/
│   ├── upstream/                         upstream OpenArm repositories, UNMODIFIED (git clones,
│   │   ├── openarm_ros2/                 pinned in ../upstream.repos)
│   │   ├── openarm_description/
│   │   └── openarm_can/
│   ├── openarm_sorting_description/      simulated robot: Xacro wrapper, Gazebo preset,
│   │                                     controllers, finger collision patch
│   ├── openarm_sorting_vision/           perception: cube detection + 3D localisation
│   ├── openarm_sorting_manipulation/     task: coordinator, MoveIt/gripper clients, prompt
│   └── openarm_sorting_bringup/          launch files, world, models, RViz, MoveIt overrides
├── tests/integration/                    end-to-end and subsystem system tests
├── docs/                                 documentation (this folder)
├── upstream.repos                        upstream repositories + exact commits
├── README.md, CONTRIBUTING.md, LICENSE, .gitignore
```

## 2. Packages and responsibilities

### Upstream (src/upstream/, not modified)

| Package | Repository @ commit | Used for |
|---|---|---|
| `openarm_description` | enactic/openarm_description @ `8dbfcc9` | robot macros, meshes, convex decompositions used by the finger patch |
| `openarm_bimanual_moveit_config` | enactic/openarm_ros2 @ `8f23aea` | SRDF, kinematics, joint limits, MoveIt controller mapping (v2.0) |
| `openarm_bringup`, `openarm_hardware`, `openarm` | enactic/openarm_ros2 @ `8f23aea` | built (part of the repository), not used by the simulation |
| `openarm_can` | enactic/openarm_can @ `26b266a` | dependency of `openarm_hardware`, not used by the simulation |

The original workspace kept one project file **inside** the upstream clone
(`openarm_description/presets/v2/default_bimanual_gazebo.yaml`, untracked). In the new
workspace that file lives in `openarm_sorting_description/presets/`; the upstream clones have
no local changes (`git status` is clean).

### openarm_sorting_description (ament_python)

Responsibility: **what the simulated robot is.**

| Path | Content |
|---|---|
| `urdf/openarm_v20_gazebo.urdf.xacro` | wraps the upstream `openarm_robot` macro logic, loads the Gazebo preset from this package, adds the gz_ros2_control plugin (`controllers_file` argument), the RGB-D sensor and the camera link/optical frames |
| `presets/default_bimanual_gazebo.yaml` | Gazebo robot preset: `gz_ros2_control/GazeboSimSystem` hardware, joint4 limit override |
| `config/gazebo_controllers.yaml` | the 5 ros2_control controllers loaded inside Gazebo |
| `openarm_sorting_description/robot_description.py` | `gazebo_robot_description()`, `xacro_file()`, `xacro_mappings()` |
| `openarm_sorting_description/gripper_collision_patch.py` | Gazebo-only finger box collisions |
| `test/test_robot_description.py` | Xacro expands, frames exist, patch applied, controllers path |

### openarm_sorting_vision (ament_python)

Responsibility: **where the cubes are** (camera -> 3D poses in `world`).

| Path | Content |
|---|---|
| `openarm_sorting_vision/cube_detector.py` | node `cube_detector`: synchronisation, pipeline, stability filter, publishers |
| `openarm_sorting_vision/color_segmentation.py` | HSV masks and blob filter (pure OpenCV) |
| `openarm_sorting_vision/camera_geometry.py` | back-projection, quaternion / transform helpers (pure NumPy) |
| `openarm_sorting_vision/ground_truth.py` | Gazebo ground-truth poses via gz-transport (**test tools only**) |
| `openarm_sorting_vision/detection_validator.py` | node `detection_validator`: detection vs ground truth (**test tool**) |
| `config/cube_detector.yaml` | all detector parameters (validated values) |
| `test/test_perception_core.py` | segmentation, back-projection, frame conventions |

### openarm_sorting_manipulation (ament_python)

Responsibility: **doing the task** with the LEFT arm and LEFT gripper.

| Path | Content |
|---|---|
| `openarm_sorting_manipulation/coordinator.py` | node `cup_sorting_coordinator`: services, readiness, planning scene, pick, place, verification, recovery |
| `openarm_sorting_manipulation/moveit_interface.py` | thin client for move_group's standard ROS interfaces + `grasp_pose()` |
| `openarm_sorting_manipulation/gripper.py` | `GripperCommand` client + measured finger angle |
| `openarm_sorting_manipulation/planning_scene.py` | collision object / attach / detach builders |
| `openarm_sorting_manipulation/ros_utils.py` | blocking waits on futures from a worker thread |
| `openarm_sorting_manipulation/task_prompt.py` | node `task_prompt`: readiness wait + y/N confirmation on the terminal |
| `openarm_sorting_manipulation/check_cup_contents.py` | ground-truth placement check (**test tool**) |
| `config/coordinator.yaml` | motion, gripper and verification parameters |
| `config/cup_sorting_scene.yaml` | table and cup geometry (must match the world) |
| `test/test_manipulation_core.py` | prompt answers, grasp pose math, scene builders, scene file |

### openarm_sorting_bringup (ament_python)

Responsibility: **starting the system.** No application logic.

| Path | Content |
|---|---|
| `launch/*.launch.py` | 9 launch files, see [LAUNCH_ARCHITECTURE.md](LAUNCH_ARCHITECTURE.md) |
| `openarm_sorting_bringup/launch_support.py` | controller names from the YAML, MoveIt config builder, process environment, pre-flight check, Gazebo leftover cleanup |
| `openarm_sorting_bringup/wait_for_controllers.py` | executable `wait_for_controllers`: readiness gate for event-driven startup |
| `config/moveit_overrides.yaml` | KDL timeout and execution-tolerance overrides on top of the upstream MoveIt config |
| `worlds/openarm_cup_sorting_world.sdf` | the Gazebo world |
| `models/{cup_sorting_table,coffee_cup,red_cube,blue_cube}/` | Gazebo models (`model://` URIs) |
| `rviz/cup_sorting.rviz`, `rviz/camera_view.rviz` | RViz configurations (full / camera only) |
| `scripts/generate_coffee_cup.py` | regenerates the coffee cup model in the source tree (development tool, not installed) |
| `test/test_bringup.py` | launch files load, controllers YAML, world/models installed, world cup = scene file |

## 3. Dependency rules

* `bringup` depends on every project package; no project package depends on `bringup`.
* `vision` and `manipulation` do not import each other at run time. The only link is the
  `check_cup_contents` **test tool**, which reuses `openarm_sorting_vision.ground_truth`.
* The coordinator never uses Gazebo ground truth; only the two test tools do.
* Configuration that a node owns lives in that node's package (`cube_detector.yaml`,
  `coordinator.yaml`, `cup_sorting_scene.yaml`); configuration of the system as a whole
  lives in `bringup` (MoveIt overrides, world, RViz).
* The controller list has a single source: `openarm_sorting_description/config/gazebo_controllers.yaml`.
  The spawner, the readiness gate and the coordinator's `required_controllers` are all
  derived from it by the launch files.
