# Diagram: launch sequence

Event-driven startup of `cup_sorting.launch.py`. Every arrow labelled "on exit" is an
`OnProcessExit` event handler. No step waits for a fixed time. Used by
[LAUNCH_ARCHITECTURE.md](../LAUNCH_ARCHITECTURE.md).

```mermaid
sequenceDiagram
    autonumber
    actor U as Operator
    participant L as cup_sorting.launch.py
    participant S as simulation.launch.py
    participant G as Gazebo (gz sim)
    participant R as robot.launch.py<br/>(robot_state_publisher)
    participant SP as ros_gz_sim create
    participant C as controllers.launch.py<br/>(spawner)
    participant W as wait_for_controllers
    participant A as moveit / rviz / perception /<br/>coordinator launch files
    participant P as task_prompt

    U->>L: ros2 launch openarm_sorting_bringup cup_sorting.launch.py
    L->>L: pre-flight: another Gazebo server running? -> error + shutdown
    L->>S: include
    L->>W: start readiness gate (polls /controller_manager/list_controllers)
    S->>S: pre-flight check, GZ_SIM_RESOURCE_PATH, run marker, GPU env
    S->>G: gz sim -r world.sdf (on_exit_shutdown)
    S->>R: include (Gazebo description with finger box patch)
    S->>SP: spawn openarm_v20 from /robot_description
    SP-->>S: on exit (code 0) -> include controllers.launch.py
    Note over SP,S: code != 0 -> shutdown "robot spawn failed"
    S->>C: one spawner loads + activates 5 controllers
    C->>G: /controller_manager services (spawner waits for them)
    W-->>L: on exit (code 0): all 5 controllers ACTIVE
    Note over W,L: code > 0 (timeout) -> shutdown "controllers not active"
    L->>A: include moveit.launch.py, rviz.launch.py,<br/>perception.launch.py, coordinator.launch.py
    A->>P: start task_prompt (prompt:=true, auto_start:=false)
    P->>P: wait for /cup_sorting/ready = success<br/>and fresh red + blue cube poses
    P->>U: banner + "Do you want to execute the sorting task? [y/N]:"
    U->>P: y / n / Enter
    Note over P: y = /cup_sorting/run, n or Enter = nothing is executed
    U->>L: Ctrl+C
    L->>G: SIGINT to all processes
    G-->>S: gz front-end exits -> stop any leftover gz process of this run
```
