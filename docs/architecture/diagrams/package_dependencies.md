# Diagram: package dependencies

Run-time dependencies (`exec_depend`) between the workspace packages. Arrows point from the
dependent package to its dependency. Only the most important ROS 2 / MoveIt / Gazebo system
packages are shown. Used by [PACKAGE_ARCHITECTURE.md](../PACKAGE_ARCHITECTURE.md).

```mermaid
flowchart TB
    subgraph PROJECT["project packages (src/)"]
        BRINGUP["openarm_sorting_bringup<br/>launch, world, models, RViz, MoveIt overrides"]
        MANIP["openarm_sorting_manipulation<br/>coordinator, task_prompt"]
        VISION["openarm_sorting_vision<br/>cube_detector"]
        DESC["openarm_sorting_description<br/>Gazebo robot Xacro, preset, controllers"]
    end

    subgraph UPSTREAM["upstream OpenArm (src/upstream/, unmodified)"]
        OADESC["openarm_description"]
        MOVEITCFG["openarm_bimanual_moveit_config"]
        OABRINGUP["openarm_bringup"]
        HW["openarm_hardware"]
        CAN["openarm_can"]
        META["openarm (metapackage)"]
    end

    subgraph SYSTEM["ROS 2 Jazzy / Gazebo Harmonic / MoveIt 2 (apt)"]
        GZ["ros_gz_sim, ros_gz_bridge,<br/>gz_ros2_control"]
        MOVEIT["moveit_ros_move_group,<br/>moveit_configs_utils"]
        ZED["zed_description"]
        CV["cv_bridge, OpenCV"]
    end

    BRINGUP --> MANIP
    BRINGUP --> VISION
    BRINGUP --> DESC
    BRINGUP --> MOVEITCFG
    BRINGUP --> GZ
    BRINGUP --> MOVEIT
    MANIP -. "check_cup_contents (test tool)<br/>reuses ground_truth" .-> VISION
    VISION --> CV
    DESC --> OADESC
    DESC --> GZ
    MOVEITCFG --> OADESC
    OADESC --> ZED
    META --> OABRINGUP
    META --> HW
    META --> MOVEITCFG
    OABRINGUP --> MOVEITCFG
    HW --> CAN
```

`openarm_bringup`, `openarm_hardware`, `openarm_can` and `openarm` are part of the upstream
repositories and are built, but the simulation does not use them at run time (the Gazebo
preset selects `gz_ros2_control/GazeboSimSystem` instead of the CAN hardware interface).
