# Visual-manipulation framework – architecture

The coffee / portafilter task is built on five robot-, camera- and simulator-independent
packages (`visual_manipulation_*`) plus a Gazebo-only integration layer. Everything that is
specific to the OpenArm, its simulated camera and Gazebo is **configuration and launch files**
in `openarm_sorting_bringup` / `openarm_sorting_description`.

```mermaid
flowchart LR
  subgraph core["robot / simulator independent"]
    IF[visual_manipulation_interfaces<br/>msgs]
    CORE[visual_manipulation_core<br/>SE3, filter, grasp, bayonet,<br/>PBVS, state machine<br/><i>pure Python + NumPy</i>]
    PER[visual_manipulation_perception<br/>aruco_detector<br/>fiducial_object_tracker]
    MOT[visual_manipulation_motion<br/>MoveIt client, Arm (TCP),<br/>gripper client, scene shapes]
    TASK[visual_manipulation_tasks<br/>portafilter_task, task_prompt]
  end
  subgraph sim["Gazebo only"]
    GZ[visual_manipulation_gazebo<br/>grasp_assist, check_portafilter]
  end
  subgraph openarm["OpenArm + Gazebo specific"]
    BR[openarm_sorting_bringup<br/>world, models, config/coffee/*.yaml,<br/>launch]
    DE[openarm_sorting_description<br/>robot + camera xacro]
  end
  CORE --> PER & MOT & TASK
  IF --> PER & TASK
  MOT --> TASK
  CORE --> GZ
  BR -. launches / configures .-> PER & TASK & GZ
  DE -. robot_description .-> BR
```

No `visual_manipulation_*` core package imports Gazebo, MuJoCo or an OpenArm name
(verified: `grep -r "openarm\|gz\." src/visual_manipulation_{core,perception,motion,tasks}`
only finds comments).

## Responsibilities

| Responsibility (request §4) | Where | Interfaces |
|---|---|---|
| A. Perception | `visual_manipulation_perception/aruco_detector_node.py`, `aruco_pose.py` | in: `sensor_msgs/Image`, `CameraInfo`, optional depth `Image`; out: `FiducialDetectionArray`, debug image, TF `aruco_<id>` |
| B. Pose estimation + frames | `aruco_pose.py` (PnP IPPE + optional RGB-D refinement), `object_tracker_node.py` (TF2 camera→reference, marker→object, temporal filter) | out: `TrackedObjectArray`, `PoseStamped` per object, TF `<reference> → <object>` |
| C. Grasp pose generation | `visual_manipulation_core/grasp.py` | object pose + configured object→TCP transforms, approach / retreat motions (base / tool / object frame), flange↔TCP |
| D. Motion planning | `visual_manipulation_motion/moveit_client.py`, `arm.py`; insertion geometry in `visual_manipulation_core/bayonet.py` | standard move_group services/actions |
| E. Gripper | `visual_manipulation_motion/gripper_client.py` | `control_msgs/action/GripperCommand` + `JointState` (finger-position grasp verification) |
| F. Task coordinator | `visual_manipulation_tasks/portafilter_task.py` on `visual_manipulation_core/state_machine.py` | Trigger services, `TaskStatus` |
| Visual servoing | `visual_manipulation_core/servo.py` (PBVS on the held object, command-based) | – |
| Simulator specifics | `visual_manipulation_gazebo` (ground truth, grasp assist), world/models in bringup | gz-transport only there |

## Key design decisions

1. **Marker ≠ object ≠ grasp.** Three separate, configured transforms:
   camera→marker (measured) · marker→object (`objects.<name>.marker_to_object`, CAD data) ·
   object→TCP (`grasp.<candidate>.object_to_tcp`). The robot flange is reached through
   `arm.flange_to_tcp`. Changing the gripper / robot only changes the last two.
2. **Everything in object frames, placed at measured poses.** Collision shapes of the machine,
   portafilter and dock are defined in their own frames (`coffee_geometry.yaml`) and placed at
   the camera-measured poses; the bayonet goals are defined in the mount frame. Nothing in the
   task uses a world pose of an object (the only configured world poses are the ready pose of
   the arm and the fallback place pose for `~/remove` after a restart).
3. **Closed-loop alignment (PBVS) on the held object** with the camera watching both the
   portafilter marker and the machine marker. Corrections are applied to the *commanded* TCP
   (integral action), which removes arm sag under load and camera-to-robot calibration
   errors – both matter on the real robot. Measured improvement: the servo converges in 1–2
   steps to < 1.5 mm radial; without the command-based formulation it did not converge
   (sag of 0.015–0.022 rad on the loaded arm).
4. **Verification everywhere, not assumptions.** Every motion: joint tracking error
   (contact / blocking detection). Grasp: finger window **and** the camera sees the part move
   with the hand. Installation: camera classification of the bayonet state, then a release
   test (the gripper opens; a locked bayonet holds the part). Placement: camera.
5. **Standard interfaces.** Only `sensor_msgs`, `geometry_msgs`, TF2, move_group services /
   actions, `control_msgs/GripperCommand`, `std_srvs/Trigger`. Custom messages only where
   standard ones lack fields: fiducial detection with quality indicators, tracked object with
   stability, task status.
6. **RGB first, RGB-D optional.** The detector works with any calibrated camera; if a
   registered depth image is configured it refines the pose (plane fit + Kabsch on the corner
   rays). See the accuracy table in `COFFEE_TASK.md`.

## Porting

| Target | What changes | What stays |
|---|---|---|
| **Real OpenArm** | camera topics / optical frame / intrinsics in `perception.yaml` (+ hand-eye calibration in TF), `depth_topic: ""` if no depth, gripper window measured on the real handle, `required_controllers`, velocity scaling; no `grasp_assist`; real marker sizes | all node code, geometry file, state machine |
| **MuJoCo** | a MuJoCo bringup providing the same ROS interfaces (camera image + CameraInfo, `/joint_states`, JTC + GripperCommand controllers, robot_state_publisher TF, move_group); world built from the same object dimensions | everything in `visual_manipulation_*` |
| **Another arm** | new `task.yaml`: planning group, tip link, joints, `flange_to_tcp`, hand links, gripper action + joint + window, grasp candidates (object→TCP for that gripper), ready pose | perception, core, motion, task code |
| **Another object / mount** | a new geometry file (marker→object, shapes, bayonet spec) and grasp candidates | everything else |
