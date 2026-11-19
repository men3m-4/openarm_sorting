# Diagram: perception pipeline

Processing per synchronized RGB + depth frame in `cube_detector`
(`openarm_sorting_vision/cube_detector.py`). Used by
[PERCEPTION.md](../../packages/PERCEPTION.md).

```mermaid
flowchart TD
    IMG["/openarm_zed/image<br/>rgb8 640x480"] --> SYNC
    DEP["/openarm_zed/depth_image<br/>32FC1, metres, z-depth"] --> SYNC
    INFO["/openarm_zed/camera_info"] --> K["intrinsics K<br/>fx = fy = 320, cx = 320, cy = 240"]
    SYNC["ApproximateTimeSynchronizer<br/>queue 5, slop 0.03 s"] --> HSV["RGB -> HSV"]
    HSV --> SEG["colour mask per cube<br/>red H 0-8 or 172-179, blue H 100-128<br/>S >= 120, V >= 50, 3x3 open + close"]
    SEG --> BLOB["connected components<br/>area 60-20000 px, fill ratio >= 0.55<br/>keep the largest blob"]
    BLOB --> ERODE["erode blob by 2 px<br/>(drop mixed edge pixels)"]
    ERODE --> VALID["finite depth in 0.15-2.0 m<br/>>= 25 valid pixels"]
    VALID --> BP["back-project with K into<br/>openarm_zed_sim_camera_optical_frame"]
    K --> BP
    TF["TF world <- optical frame<br/>(robot_state_publisher)"] --> XF
    BP --> XF["transform points to world"]
    XF --> TOP["top face = points within 6 mm of the<br/>95th-percentile height"]
    TOP --> RECT["min-area rectangle of the top face<br/>centre x, y, z = top - size/2, yaw mod 90 deg"]
    RECT --> SIZE{"top face size<br/>plausible for a 4 cm cube?"}
    SIZE -- no --> REJ["reject frame<br/>(history ages out)"]
    SIZE -- yes --> RAW["/cup_sorting/COLOUR_cube/raw"]
    RAW --> STAB{"last 6 estimates within 1.5 s<br/>all within 4 mm of their median?"}
    STAB -- no --> WAIT["not published yet"]
    STAB -- yes --> POSE["/cup_sorting/COLOUR_cube/pose<br/>median centre + circular-median yaw (world)"]
    POSE --> MK["/cup_sorting/detection_markers"]
    SEG --> DBG["/cup_sorting/debug_image"]
```
