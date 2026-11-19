# Diagram: manipulation sequence (one cube)

`CupSortingCoordinator.pick()` + `place_in_cup()` in
`openarm_sorting_manipulation/coordinator.py`. Every motion is followed by a joint-error
check (`execute_verified`); any failed check raises `StepFailed`, aborts the sequence and runs
`safe_recover()`. Used by [MANIPULATION.md](../../packages/MANIPULATION.md).

```mermaid
sequenceDiagram
    autonumber
    participant C as coordinator
    participant D as cube_detector
    participant M as move_group
    participant J as left JTC (Gazebo)
    participant G as left gripper

    D->>C: stable cube pose (world)
    C->>M: /apply_planning_scene (table + edge guard, cup cylinder, cubes)
    C->>M: IK + OMPL to the READY pose above the table
    M->>J: execute
    C->>C: verify joint error < 0.03 rad
    C->>G: open to 0.45 rad, verify finger angle
    C->>M: IK (12 seeds, max joint-limit margin) + OMPL to pre-grasp<br/>10 cm above, 60 deg pitch, yaw along a cube face
    M->>J: execute, verify
    D->>C: re-measure the cube (if still visible)
    C->>M: remove target cube from the scene, Cartesian descent<br/>(TCP = cube centre + 13 mm)
    M->>J: execute, verify, finger must not move (contact check)
    C->>G: squeeze to 0.0 (position control), settle 3 s
    C->>C: GRASP VERIFIED if finger stops in 0.12-0.32 rad
    C->>M: attach cube to openarm_left_ee_base_link, Cartesian lift 12 cm
    M->>J: execute, verify, finger still in range
    C->>M: IK + OMPL above the cup (+6 cm), release offset per colour
    M->>J: execute, verify, still holding
    C->>M: Cartesian lowering: cube bottom 15 mm above the rim
    M->>J: execute, verify
    C->>G: open to 0.35 rad (release), verify, detach cube
    C->>M: Cartesian retreat 8 cm, then READY pose
    M->>J: execute, verify
    D->>C: fresh detection: inside the cup or no longer on the table
    C->>C: PLACEMENT VERIFIED
```
