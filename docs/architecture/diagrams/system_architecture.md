# Diagram: overall system architecture

Processes and the main ROS 2 interfaces between them while the complete application runs
(`ros2 launch openarm_sorting_bringup cup_sorting.launch.py`). Used by
[SYSTEM_ARCHITECTURE.md](../SYSTEM_ARCHITECTURE.md).

```mermaid
flowchart LR
    subgraph GZ["Gazebo Harmonic (gz sim, bullet-featherstone)"]
        WORLD["openarm_cup_sorting world<br/>table, coffee cup, red + blue cube"]
        ROBOT["OpenArm v2.0 (spawned from /robot_description)"]
        GZRC["gz_ros2_control plugin<br/>= /controller_manager"]
        CAM["RGB-D sensor openarm_zed_rgbd<br/>640x480, 15 Hz"]
        ROBOT --- GZRC
        ROBOT --- CAM
    end

    RSP["robot_state_publisher<br/>/robot_description, /tf"]
    BRIDGE["ros_gz_bridge<br/>cup_sorting_bridge"]
    CTRL["controllers<br/>joint_state_broadcaster<br/>left/right JTC<br/>left/right gripper"]
    MG["move_group<br/>(MoveIt 2, OMPL, KDL)"]
    DET["cube_detector<br/>(OpenCV HSV + depth)"]
    COORD["cup_sorting_coordinator<br/>(LEFT arm pick and place)"]
    PROMPT["task_prompt<br/>(y/N confirmation)"]
    RVIZ["RViz 2"]
    OP(["operator terminal"])

    RSP -- "/robot_description" --> ROBOT
    GZRC --- CTRL
    CTRL -- "/joint_states" --> RSP
    CAM -- "gz topics" --> BRIDGE
    BRIDGE -- "/clock<br/>/openarm_zed/image, depth_image,<br/>camera_info, points" --> DET
    DET -- "/cup_sorting/{red,blue}_cube/pose" --> COORD
    DET -- "debug_image, detection_markers" --> RVIZ
    COORD -- "/compute_ik, /compute_cartesian_path,<br/>/move_action, /execute_trajectory,<br/>/apply_planning_scene" --> MG
    MG -- "follow_joint_trajectory" --> CTRL
    COORD -- "/left_gripper_controller/gripper_cmd" --> CTRL
    PROMPT -- "/cup_sorting/ready<br/>/cup_sorting/run" --> COORD
    OP <-- "/dev/tty" --> PROMPT
    MG -- "planning scene, trajectories" --> RVIZ
```
