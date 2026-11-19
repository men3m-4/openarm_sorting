# Diagram: task execution flow

From operator confirmation to the end of the task (`task_prompt` -> `/cup_sorting/run` ->
`CupSortingCoordinator.sort(['red', 'blue'])`). Used by
[EXECUTION_FLOW.md](../EXECUTION_FLOW.md) and [TASK_EXECUTION.md](../../execution/TASK_EXECUTION.md).

```mermaid
flowchart TD
    START(["all components ready<br/>(/cup_sorting/ready + both cubes detected)"]) --> ASK
    ASK{"Do you want to execute<br/>the sorting task? [y/N]"}
    ASK -- "invalid input" --> ASK
    ASK -- "n / N / Enter / EOF" --> NO["print 'NOT executed' + how to run later<br/>simulation keeps running"]
    ASK -- "y / Y" --> RUN["call /cup_sorting/run (std_srvs/Trigger)"]
    RUN --> BUSY{"coordinator idle and ready<br/>(within 5 s)?"}
    BUSY -- no --> FAILMSG["success = false<br/>'busy' or 'not ready'"]
    BUSY -- yes --> RED["pick_and_place(red)"]
    RED -- "StepFailed / exception" --> REC["safe_recover():<br/>lift 6 cm if possible, plan to READY<br/>(held cube is not dropped)"]
    RED -- "verified" --> BLUE["pick_and_place(blue)"]
    BLUE -- "StepFailed / exception" --> REC
    BLUE -- "verified" --> OK["success = true<br/>'red: placed in cup (verified) | blue: ...'"]
    REC --> FAILMSG
    OK --> REPORT["task_prompt prints<br/>TASK FINISHED SUCCESSFULLY"]
    FAILMSG --> REPORT2["task_prompt prints<br/>TASK DID NOT COMPLETE"]
    REPORT --> END(["simulation keeps running until Ctrl+C"])
    REPORT2 --> END
    NO --> END
```

With `auto_start:=true` the prompt is not started and the coordinator runs the same
`sort(['red', 'blue'])` by itself once its own readiness check passes.
