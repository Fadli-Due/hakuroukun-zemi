# hakuroukun_ws
## Code for Hakuroukun Cleaning Robot

### Setting up the environment

-----

* **Enable GUI within Docker containers**

  > **! Caution:** This method exposes PC to external source. Therefore, a more secure alternative way is expected for using GUI within Docker containers. This problem was raised in [Using GUI's with Docker](https://wiki.ros.org/es/docker/Tutorials/GUI#:~:text=%2D%2Dpulse.-,Using%20X%20server,-X%20server%20is)

```bash
# This command is required to run every time the PC is restarted
xhost +
```

Make a X authentication file with proper permissions for the container to use.

```bash
# If not working, try to run "sudo rm -rf /tmp/.docker.xauth" first
cd ./src/hakuroukun_dockerfiles/
chmod +x ./install/xauth.sh && ./install/xauth.sh
```

> **If xauth.sh says "data" instead of "X11 Xauthority data":** The session
> cookie is keyed to the hostname, not `:0`. Run:
> ```bash
> xauth nlist | sed -e 's/^..../ffff/' | xauth -f /tmp/.docker.xauth nmerge -
> chmod 644 /tmp/.docker.xauth
> ```
> If the container still fails to start with "not a directory" error, the
> container's internal rootfs has a stale directory at `/tmp/.docker.xauth`
> from a crashed previous run. Fix with:
> ```bash
> docker compose down
> docker compose up -d
> catkin_make  # required after down/up wipes build/ and devel/
> ```

> **Environment note (lab laptop):** The `src/` folder is bind-mounted into the
> container at `/root/catkin_ws/src`. Edits on the host sync automatically —
> no `docker cp` needed. `catkin_make` is still required after adding new nodes
> or changing `CMakeLists.txt`.
>
> **Docker lifecycle:** Use `docker compose stop` (not `down`) to preserve
> `build/` and `devel/` between sessions. `down` wipes build artifacts and
> forces a full `catkin_make` on next start.

---

### Simulation : pure pursuit controller
Change mode in hakuroukun_launch/launch/bringup.launch
```
    <arg name="simulation" default="true" />
```
To run gazebo with ekf localization
```
docker exec -it hakuroukun-robot bash
roslaunch hakuroukun_launch bringup.launch
```

On another terminal, run pure pursuit controller
```
docker exec -it hakuroukun-robot bash
roslaunch hakuroukun_control hakuroukun_control.launch
```

### Experiment : To run robot with any controller:
0. Check ```cat /dev/ttyACM*``` for GPS, Arduino, IMU

1. Change mode in hakuroukun_launch/launch/bringup.launch
```
    <arg name="simulation" default="false" />
```
2. GPS Rotation angle calibration:
    - `rotation_angle` is **hardcoded** in `hakuroukun_pose/hakuroukun_pose/hakuroukun_pose.py`
      inside `_get_xy_from_latlon()` as `math.radians(172.1)`. The launch file param
      `~rotation_angle` is dead — the node never reads it.
    - To recalibrate: drive a straight segment in manual mode, record GPS coordinates,
      compute the heading angle, and update the hardcoded value directly in
      `hakuroukun_pose.py`. Do **not** edit `src/hakuroukun_pose/hakuroukun_pose_node.py`
      — it is never imported.

3. Arduino firmware:
    - Upload `${hakuroukun_communication}/firmware/bcd_automode/bcd_automode.ino`
      for autonomous BCD runs (see Real Robot section for full firmware workflow).

4. Bringup execution:
In the 1st terminal
    ```
    docker exec -it hakuroukun-robot bash
    roslaunch hakuroukun_boustrophedon_with_cones bringup_hakuroukun_robot.launch
    ```

5. Controller execution:
In the 2nd terminal
    ```
    docker exec -it hakuroukun-robot bash
    roslaunch hakuroukun_boustrophedon_with_cones offline_path_planning_real.launch
    ```

---

## `hakuroukun_boustrophedon_with_cones` — Offline BCD coverage with online obstacle handling

This package is my master's thesis work (Due, 2026). It is an **offline
Boustrophedon Cell Decomposition (BCD)** coverage planner with an **online
local replanner** that modifies the path when obstacles are detected. It
replaces Tai's online TASP planner from the previous study.

### What it does

- **Offline planning**: A complete coverage path is computed once from a
  pre-built occupancy map (BCD decomposition + A\* transit segments + pure
  pursuit-following waypoints).
- **Online modification (two layers)**:
  - **Layer 1 — Reflex stop** (in `path_follower.py`): LiDAR `FORWARD VETO`
    at 0.70 m. Stops the robot in milliseconds for sudden intrusions
    (a child running into the path, etc.).
  - **Layer 2 — Local replanner** (`local_replanner.py`): Watches LiDAR
    for obstacles that **persist on the path for ≥ 7 seconds**, then splices
    an A\*-computed detour into the baseline. Yields to transient obstacles
    (pedestrians passing) but routes around static blockers. **Detours are
    permanent commitments** — once an obstacle is classified as static
    (persistence ≥ 7 s), the spliced detour stays in `current_path` for
    the rest of the run. The old "restore baseline when LiDAR loses sight"
    behavior was removed (2026-06-25): losing sight of a static obstacle
    doesn't mean it's gone, and reverting to the pristine baseline would
    contradict the persistence model.
- **Return-to-start** (`return_to_start.py`): After coverage completes
  (path_follower reports robot has held final pose for ≥ 2 s), an A\* path
  from current pose back to `baseline[0]` is computed and appended to
  `current_path`. The robot drives home automatically. Persistence-gated
  obstacle avoidance keeps running on the return leg — if someone walks
  into the corridor, a detour is spliced onto the return path the same
  way it would be on a coverage lane.
- **Restricted zones via cones**: Cones are pre-painted into the planning
  map (Phase A). Real-time ZED2-based detection (Phase B) is deferred.

### Topic flow

```
offline_coverage_planner ──/planned_path──┐
                                          ├──> local_replanner ──/desired_path──> path_follower
return_to_start ──────────/return_path────┘                                              │
        ↑                                  ↑                                             │
        │                              /scan_multi (LiDAR)                               │
        └──────/path_follower/done ◄──────────────────────────────────────────────────────┘
```

`/planned_path` is the latched baseline BCD path. `/desired_path` is what
the follower actually tracks — equal to baseline when no detour is active,
spliced detour when one is, with the return-to-home leg appended after
coverage completes. `/path_follower/done` is a latched Bool fired once
when the robot has held the final coverage pose for `done_dwell_time`
seconds; this triggers `return_to_start` to compute the A\* return path.

### Package layout

```
hakuroukun_boustrophedon_with_cones/
├── config/
│   ├── amcl_params.yaml              ← AMCL tuning (simulation only)
│   ├── boustrophedon_config.yaml     ← BCD parameters (lane spacing, robot radius)
│   ├── path_follower_config.yaml     ← pure pursuit (lookahead=0.8, search_ahead=25)
│   ├── local_replanner_config.yaml   ← persistence threshold, window size, etc.
│   ├── costmap_params.yaml
│   └── gmapping_params.yaml
├── launch/
│   ├── bringup_hakuroukun_conemap_sim.launch     ← Gazebo + robot (conemap world)
│   ├── bringup_hakuroukun_warehouse_sim.launch   ← Gazebo + robot (warehouse world)
│   ├── bringup_hakuroukun_robot.launch           ← Real robot bringup
│   ├── offline_path_planning_conemap.launch      ← Planner + replanner + follower (conemap)
│   ├── offline_path_planning.launch              ← Planner + replanner + follower (warehouse)
│   ├── offline_path_planning_real.launch         ← Planner + replanner + follower (real robot, no AMCL)
│   ├── mapping_warehouse.launch                  ← gmapping (only needed to rebuild maps)
│   ├── gazebo_cones.launch / gazebo_cones_warehouse.launch
├── maps/
│   ├── conemap_planning.{pgm,yaml}   ← main map used for thesis experiments
│   ├── warehouse_map.{pgm,yaml}
│   └── ...
├── scripts/
│   ├── planning/
│   │   ├── offline_coverage_planner.py   ← BCD; publishes /planned_path
│   │   ├── return_to_start.py            ← A* return-to-home leg after /done
│   │   └── simple_astar.py
│   ├── control/
│   │   ├── path_follower.py              ← pure pursuit + reflex stop
│   │   ├── local_replanner.py            ← online detour layer
│   │   ├── return_to_start.py            ← A* return to baseline[0] on /path_follower/done
│   │   ├── map_odom_calibrator.py        ← dynamic map→odom TF (RViz 2D Pose Estimate)
│   │   ├── odom_tf_broadcaster.py        ← real-robot odom→base_link TF
│   │   └── sim_teleop_key.py
│   ├── evaluation/
│   │   ├── cleaning_simulator.py         ← live coverage % (publishes /cleaned_map)
│   │   ├── calculate_coverage_from_image.py
│   │   └── caculate_average_error.py     ← Savg / Smax / RMS trajectory error
│   ├── obstacles/
│   │   ├── cone_zone_creator.py
│   │   └── cones_detector_simulation.py
│   └── tools/
│       ├── test_obstacle_spawner.py      ← spawn static box / pedestrian in Gazebo
│       ├── verify_map.py
│       ├── visualizer.py
│       └── health_check.py
├── worlds/
│   ├── 30x30area.world                   ← conemap world (main thesis sim)
│   ├── 10x10area.world
│   └── hakuroukun_warehouse_v1.world
├── rviz/
│   └── boustrophedon_with_cones.rviz
├── SIM_TEST_PROCEDURE.md                 ← detailed obstacle test scenarios A/B/C
├── package.xml
└── CMakeLists.txt
```

### Build

After cloning, build the workspace and make the new Python nodes executable:

```bash
docker exec -it hakuroukun-robot bash
cd /root/catkin_ws
catkin_make
source devel/setup.bash

find src/hakuroukun_boustrophedon_with_cones/scripts -name "*.py" -exec chmod +x {} \;
```

### Simulation — conemap (main thesis scenario)

The conemap uses pre-painted cone polygons as restricted zones. It is the
main environment for thesis quantitative results.

Use 5 terminals in this order. Each terminal needs its own
`docker exec -it hakuroukun-robot bash`.

**T1 — Gazebo + bringup:**
```bash
roslaunch hakuroukun_boustrophedon_with_cones bringup_hakuroukun_conemap_sim.launch
```

**T2 — Rosbag (start BEFORE the robot begins moving):**
```bash
mkdir -p /root/catkin_ws/bags && cd /root/catkin_ws/bags
rosbag record -a -O conemap_run_$(date +%Y%m%d_%H%M%S).bag \
    __name:=conemap_recorder
```
`-a` records all topics. The `__name` lets you stop the bag cleanly with
`rosnode kill /conemap_recorder` once the run finishes — this flushes
the bag properly so it doesn't get left in `.active` state.

**T3 — Cleaning simulator** (publishes `/cleaned_map` from the robot's pose):
```bash
rosrun hakuroukun_boustrophedon_with_cones cleaning_simulator.py
```

**T4 — Planner + follower + AMCL + RViz:**
```bash
roslaunch hakuroukun_boustrophedon_with_cones offline_path_planning_conemap.launch
```
Wait for the green coverage path to appear in RViz and for AMCL's particle
cloud to converge tightly around the robot (a few seconds). **Don't skip
this check** — if the cloud stays diffuse or is offset, abort and reset
before continuing.

Useful log lines to confirm a healthy start:
- `[BCD] N coverage cells`
- `[BCD] published /planned_path with N points`
- `[local_replanner] baseline path received: N points`
- `[return_to_start] ready. robot_radius=1.00 densify=0.20`
- `[return_to_start] baseline received: home = (X, Y)`
- `[return_to_start] static map ready (WxH, res=...m)`
- `[pf] mode=FORWARD v=0.25 steer=...`

At the **end of coverage**, the return leg sequence should look like this:
```
[path_follower] /path_follower/done published (dist_to_final=X.XXm, dwell=2.0s).
[return_to_start] computing return: (X, Y) -> (X0, Y0)
[return_to_start] return path published: N A* points, M densified points.
[local_replanner] return leg appended: M points, new current_path length = ...
[path_follower] new path (... pts): closest_i seeded at ...
```
If `/path_follower/done` fires but the next three lines don't appear, check
that `return_to_start` is actually running and the static map / robot pose
are both ready (see its startup logs above).

**T5 — Live coverage readout:**
```bash
rosrun hakuroukun_boustrophedon_with_cones calculate_coverage_from_image.py
```

**Stopping the run:** kill the rosbag first so it flushes properly, then
shut down the launches:
```bash
rosnode kill /conemap_recorder
```

#### RViz displays to add

- `/cleaned_map` (Map, alpha ~0.5) — coverage trail.
- `/planned_path` (Path) — blue baseline BCD path.
- `/desired_path` (Path) — what the follower is tracking *now*.
- `/return_path` (Path) — A\* return-to-home leg (appears at end of coverage).
- `/local_replanner/obstacle_grid` (Map) — persistent obstacle cells.
- `/local_replanner/markers` (MarkerArray) — blue baseline + orange active detour.
- `/particlecloud` (PoseArray) — AMCL convergence check.

### Simulation — warehouse

Same 5-terminal pattern, swap the bringup and planning launch files:
```bash
# T1
roslaunch hakuroukun_boustrophedon_with_cones bringup_hakuroukun_warehouse_sim.launch
# T4
roslaunch hakuroukun_boustrophedon_with_cones offline_path_planning.launch
```
T2 (rosbag), T3 (`cleaning_simulator.py`), and T5 (`calculate_coverage_from_image.py`)
are identical to the conemap flow.

### Testing the online replanner

Three obstacle scenarios verify the two-layer architecture:

- **Scenario A** — static box on the path → reflex stop, then detour.
- **Scenario B** — pedestrian passes briefly (3 s) → reflex stop, **no detour**.
- **Scenario C** — pedestrian stays > 7 s → reflex stop, then detour.

Detailed steps, expected logs, and pass criteria are in
[`SIM_TEST_PROCEDURE.md`](./hakuroukun_boustrophedon_with_cones/SIM_TEST_PROCEDURE.md).

Quick example (Scenario A):
```bash
# T3 — spawn a static box ~10 m ahead of the robot
docker exec -it hakuroukun-robot bash
rosrun hakuroukun_boustrophedon_with_cones test_obstacle_spawner.py \
    static --x 5.0 --y 2.0
```

### Real robot

This section follows the same structure as Tai's `hakuroukun_tasp_with_cones`
real-robot procedure (Setup → Robot setup → Run experiment), so it should
read side-by-side with the previous lab documentation.

Key differences from simulation:
- **No AMCL.** Localization comes from RTK-GPS + IMU via `hakuroukun_pose_node`.
- `map → odom` is published by `map_odom_calibrator.py` — it starts identity
  and is updated at runtime when you click in RViz with the "2D Pose
  Estimate" tool (same workflow as AMCL).
  No relaunch needed for re-calibration.
- `odom → base_link` is published by `odom_tf_broadcaster.py` from
  `/hakuroukun_pose/rear_wheel_odometry`.
- Same `conemap_planning.yaml` map; no fresh gmapping needed.
- **No live cone detection**. Cones are pre-painted
  into the planning map and physically placed at matching positions on the
  ground before the run.

#### 1. Set up experiment environment

Place the physical cones on the ground at positions matching the polygons
painted into `conemap_planning.pgm`. The cones define restricted zones the
planner must not cross.

- 10 cones total.
- Cone coordinates can be re-extracted from the planning map with
  `scripts/tools/` if needed.

> **obstacle_stop_range note:** `path_follower_config.yaml` sets
> `obstacle_stop_range: 0.70 m`. The robot will enter HOLD if the LiDAR
> sees anything within 0.70 m, including physical cones. If the cone
> positions on the ground don't perfectly match the painted polygons in the
> map, the robot may trigger unexpected HOLDs near cone boundaries. Watch
> for this on the first run.

#### 2. Robot setup

**a. Recreate device symlinks (every session — do not persist across reboots):**
```bash
sudo ln -sf /dev/ttyACM0 /dev/imu
```
Confirm all three symlinks exist before launching:
```bash
ls /dev/arduino /dev/imu /dev/gps
```

> **Symlink mapping (confirmed):**
> - `/dev/ttyACM0` → `/dev/imu`
> - `/dev/ttyACM2` → `/dev/arduino`
> - `/dev/gps` — confirm this symlink exists; it is required by `sensor_bringup.launch`
>   but is not created by the documented setup procedure.

**b. Connect the PC to all sensors and control devices** — RTK-GPS,
TSND151 IMU, dual RPLIDARs, Arduino. Confirm each device shows up:
```bash
cat /dev/ttyACM*
```

**c. GPS rotation angle — `rotation_angle` is hardcoded, not a param.**

> **Important:** The `~rotation_angle` param in `bringup_hakuroukun_robot.launch`
> is **dead** — `hakuroukun_pose.py` hardcodes `rotation_angle = math.radians(172.1)`
> directly in `_get_xy_from_latlon()` and never calls `rospy.get_param` for it.
> The launch file value is silently ignored.
>
> The 172.1° value was calibrated on 2026-06-30 via GPS straight-line drive at
> the D-F area. To recalibrate: drive a straight segment in manual mode, record
> GPS coordinates, compute the heading angle, and update the hardcoded value in
> `hakuroukun_pose/hakuroukun_pose/hakuroukun_pose.py` directly (in
> `_get_xy_from_latlon()`). Do **not** edit `src/hakuroukun_pose/hakuroukun_pose_node.py`
> — it is never imported.

**d. Steering neutral calibration (do this before every experiment day).**

The pot reading when the wheels are physically straight must be consistent
across both the Arduino firmware and the Python communication node. Duc's
procedure adapted for the current codestack:

1. Upload `keyboard_mode.ino` via Arduino IDE.
2. Run keyboard teleop (use `python3` explicitly):
   ```bash
   python3 /root/catkin_ws/src/hakuroukun_communication/scripts/keyboard_teleop.py
   ```
3. On a flat surface, drive slowly forward and use left/right keys to find
   the steering position where the robot tracks straight. Hold that position.
4. Read `pm_st=N` from the Arduino serial output (visible in the teleop
   terminal log). That N is your neutral pot reading.
5. Update **both** of the following with the measured value:
   ```python
   # hakuroukun_communication_node.py — _corrected_steering_command()
   c_cw  = <measured>       # neutral for CW/straight (was 525)
   c_ccw = <measured> + 85  # neutral for CCW (was 610; +85 is the mechanical asymmetry offset)
   ```
   ```cpp
   // bcd_automode.ino
   #define PM_ST_N  <measured>   // was 560; must match Python c_cw
   ```
6. Upload `bcd_automode.ino`. The robot is now ready for autonomous runs.

> **Why both files?** `PM_ST_N` in the Arduino is the center of its
> range-check window — commands outside `PM_ST_N ± limits` are silently
> rejected. `c_cw` in Python is what gets sent as "zero steering angle".
> If they differ, the robot drives with a constant drift and/or some
> commands are rejected at startup.

**e. Upload the Arduino firmware.**
Open the Arduino IDE
(`arduino-ide_2.3.4_Linux_64bit.AppImage` in the downloads folder) and
upload one of:

- `${hakuroukun_communication}/firmware/bcd_automode/bcd_automode.ino` —
  for autonomous BCD coverage runs. Purpose-built for this launch chain:
  proper `\n` terminator, motor watchdog (400 ms), 10 Hz serial rate,
  diagnostic pot-reading feedback. Use this for all thesis experiments.
- `${hakuroukun_communication}/firmware/manualmode/manualmode.ino` —
  for manual button control (useful during cone placement).
- `${hakuroukun_communication}/firmware/keyboardmode/keyboard_mode/keyboard_mode.ino` —
  for keyboard teleop via `keyboard_teleop.py` (driving to experiment area
  and steering neutral calibration).

> **Serial command format:** `bcd_automode.ino` reads with `readStringUntil('\n')`.
> `hakuroukun_communication_node.py` sends `"0{dir}{steer:03d}{accel:03d}00\r\n"`
> (10 data chars + `\r\n`). The Arduino strips any trailing `\r` before the
> length check, so the protocol is robust to both `\n` and `\r\n` line endings.
> Do not revert to `motor_control.ino` for autonomous runs — it relies on a
> fragile 2 ms serial timeout instead of a proper terminator, and its motor
> while-loops can hang indefinitely if the potentiometer cable is loose.

> **Steering range:** Arduino accepts steer values `PM_ST_N - 200` to
> `PM_ST_N + 290`. Python clamps to `390–850`. Ensure `PM_ST_N` is set
> consistently with `c_cw` (see calibration above) so the Arduino's
> acceptance window is centered on the value Python sends as neutral.

#### 3. Run experiment (5 terminals)

Each terminal needs its own `docker exec -it hakuroukun-robot bash`.

**T1 — Robot bringup** (sensors, LiDAR stack, `hakuroukun_pose_node`):
```bash
roslaunch hakuroukun_boustrophedon_with_cones bringup_hakuroukun_robot.launch
```
> **GPS timeout:** `hakuroukun_pose_node` calls `wait_for_message('/fix', timeout=10s)`
> at startup. If the GPS does not have a fix within 10 seconds, the node raises
> an exception and bringup fails. Always confirm the GPS LED is solid before
> launching T1.

**T2 — Rosbag (start BEFORE the robot begins moving):**
```bash
mkdir -p /root/catkin_ws/bags && cd /root/catkin_ws/bags
rosbag record -a -O real_run_$(date +%Y%m%d_%H%M%S).bag \
    __name:=real_recorder
```

**T3 — Cleaning simulator** (marks cells under the robot as cleaned, publishes `/cleaned_map`):
```bash
rosrun hakuroukun_boustrophedon_with_cones cleaning_simulator.py
```

**T4 — Planner + replanner + follower + return-to-start + RViz:**
```bash
roslaunch hakuroukun_boustrophedon_with_cones offline_path_planning_real.launch
```

**T5 — Live coverage readout:**
```bash
rosrun hakuroukun_boustrophedon_with_cones calculate_coverage_from_image.py
```

**Stopping:** `rosnode kill /real_recorder` first to flush the bag, then shut
down the launches.

#### Calibrating `map → odom` with RViz (do this every experiment day)

> **Critical — read before touching RViz:**
> Before clicking "2D Pose Estimate", confirm that **Global Options → Fixed Frame
> is set to `map`** (not `odom`). Setting the pose estimate while Fixed Frame is
> `odom` applies the correction in the wrong coordinate frame, producing a large
> persistent heading error (alpha ~0.72–1.57 rad) that prevents path following
> even when GPS and IMU are working correctly. This was the root cause of the
> failed outdoor run on 2026-06-30.
>
> Note: `map_odom_calibrator.py` will warn in the log if `/initialpose` arrives
> with `frame_id != map`, but it **will still apply the transform**. The warning
> alone will not save you — check Fixed Frame yourself first.

Even with `rotation_angle` correctly calibrated, two coordinate frames still
need to be aligned for the robot icon in RViz to land on the correct map pixel:

- **`map` frame** — the painted PGM. Origin at `[-25, -25]` m (map covers a
  50×50 m area centered on world origin). Fixed when the map was made.
- **`odom` frame** — where `hakuroukun_pose_node` anchored its local frame
  on first power-up. Decided by where the robot happens to be at startup.

These almost never coincide by chance, so on every new experiment session
you'll see the robot icon sitting in the wrong place. The
`map_odom_calibrator` node fixes this without requiring a relaunch:

1. T1 and T4 are running. The robot icon is in the wrong place. Expected.
2. In RViz, confirm **Global Options → Fixed Frame = `map`**.
3. Click the **"2D Pose Estimate"** tool in the top toolbar.
4. Click on the map where the robot actually is, and drag in the direction
   it's actually facing.
5. The node receives `/initialpose`, computes the offset, and the icon
   snaps to the correct position. Log line in T4:
   `[map_odom_calibrator] map->odom updated: x=… y=… yaw=…`
6. If the icon drifts later, click again. No relaunch needed.

> **Guard:** `map_odom_calibrator` will not apply `/initialpose` until it has
> received at least one message on `/hakuroukun_pose/rear_wheel_odometry`. If
> you click 2D Pose Estimate too early (before T1 is fully up), the click is
> silently discarded. Wait for `[pf] mode=FORWARD` in T4 before calibrating.

For best results, calibrate while the robot is **stationary** and verify
against a second known waypoint before starting a coverage run.

**Sanity check before moving:** spin the robot in place manually and watch
`/hakuroukun_pose/rear_wheel_odometry` — the XY position should stay roughly
fixed while yaw changes. If the position walks in a circle, the GPS-to-rear-axle
heading correction in `hakuroukun_pose.py` is not working correctly.

> **LiDAR range note:** The laser merger has `range_min: 0.3 m`. Obstacles
> closer than 0.3 m are invisible to `/scan_multi`. The path follower's
> `obstacle_stop_range` is 0.70 m, well within this window.

---

### Known bugs fixed (2026-07)

| Date | File | Bug | Fix |
|------|------|-----|-----|
| 2026-07-02 | `hakuroukun_pose.py` | `self.orientation` frozen at 0 — GPS-to-rear-axle correction always applied along heading=0 regardless of actual robot heading, causing up to 0.6 m directional position error on any turn | Replace `self.orientation` with `self.yaw` in `_gps_callback`; initialize `self.yaw = 0.0` in `_register_parameters` |
| 2026-07-02 | `hakuroukun_communication_node.py` | Serial command was 8 chars; Arduino `length() != 10` check silently rejected every command — robot received no commands at all | Pad format string to `f"0{dir}{steer:03d}{accel:03d}00"` (10 chars) |
| 2026-07-02 | `hakuroukun_communication_node.py` | `self.direction` reset to 0 at top of every timer tick — caused relay chatter during REVERSE as direction flickered between ticks | Remove unconditional reset; only update direction inside the `cmd_controller_flag` block |
| 2026-07-02 | `hakuroukun_communication_node.py` | `cmd_controller_flag` never reset to False — flag stayed True permanently after first message | Flag intentionally kept True so last command persists across ticks; direction re-derived each tick from stale message |
| 2026-07-02 | `local_replanner.py` | `obs_first` array not shifted in lockstep with `obs_grid` during recenters — persistence timestamps desync after any recenter, preventing detour from ever firing | Shift `obs_first` together with `obs_grid` on every recenter |
| 2026-07-02 | `path_follower.py` | `closest_i` seeded by global argmin on new path arrival during HOLD, jumping past detour arc to geometrically closer post-rejoin baseline tail — robot skipped entire detour | Preserve `closest_i` when new path arrives during HOLD mode |
| 2026-07-03 | `motor_control.ino` | `readStringUntil("\r\n")` takes a `char`, not a string — Arduino waited for `'\r'` which Python never sent; fell back to 2 ms timeout causing intermittent partial reads and rejected commands | Replaced by `bcd_automode.ino` using `readStringUntil('\n')` with explicit `\r` strip |
| 2026-07-03 | `motor_control.ino` | `motor_ac()` / `motor_st()` while-loops had no timeout — a loose potentiometer cable or mechanical stop caused the Arduino to hang permanently, ignoring all subsequent serial commands | `bcd_automode.ino` adds a 400 ms watchdog (`MOTOR_TIMEOUT_MS`) that breaks out and sets status `'2'` |
| 2026-07-03 | `bringup_hakuroukun_robot.launch` | `range_min: 1.0` in laser merger discarded all LiDAR readings under 1 m — `obstacle_stop_range = 0.70 m` could never trigger; HOLD and replanner were effectively blind to close obstacles | Fixed to `range_min: 0.3` |
| 2026-07-03 | `communication_bringup.launch` | `arduino_controller_rate` defaulted to 1 Hz while `path_follower` runs at 10 Hz — up to 1 s of stale steering commands per cycle (~30 cm of wrong-direction travel at 0.3 m/s) | Default raised to 10 Hz |
| 2026-07-05 | `hakuroukun_communication_node.py` | Python steer clamp upper bound was 885 but Arduino rejects values above 850 (`PM_ST_N + PM_ST_LIML = 560 + 290`) — silent command rejection on sharp CCW turns | Clamp changed to `max(390, min(850, ...))` |
| 2026-07-05 | `hakuroukun_pose.py` | `log_data/` directory not created automatically — `open()` in `_log_pose` threw `FileNotFoundError` on first call, crashing the log timer silently | Added `os.makedirs(new_folder, exist_ok=True)` in `_register_log_file()` |

---

### Simulation results (thesis runs)

Runs 1 and 2 had three bugs present (see table above) that partially suppressed
detour behavior and caused `closest_i` jumps. Run 3 is the corrected
implementation after all three bugs were fixed.

| Run | Bag | Coverage | Detours fired | Notes |
|-----|-----|----------|---------------|-------|
| Run 1 | `conemap_with_return_20260626_031733.bag` | 65.20% | 1 | Partial — `obs_first` recenter bug + `ROS_IP` bug + `closest_i` jump bug present |
| Run 2 | `conemap_run_20260626_052306.bag` | 63.96% | 2 | Partial — same three bugs present |
| Run 3 | `conemap_run_20260702_*.bag` | **91.21%** | — | **Corrected implementation** — all three bugs fixed |

Baseline comparison: Tai's TASP = 52.80%.

---

### Key references

- Choset & Pignon, 1998 — Boustrophedon Cell Decomposition.
- Galceran & Carreras, 2013 — CPP survey.
- Nguyen Van Tai, 2024 — previous TASP work in this lab (baseline: 52.80% coverage).
- Schmid et al., 2023 — Dynablox (IEEE RA-L, DOI: 10.1109/LRA.2023.3305239) — supports "persistence-gated" obstacle terminology.
- Kondo et al., 2026 — SANDO (arXiv:2604.07599) — supports "persistence-gated" replanning concept.

### Notes on conventions

- Coverage % is computed with `cleaning_width = 1.0 m` to stay comparable to
  Tai's results.
- Unknown cells (`-1`) in the OccupancyGrid are **not** counted as free in
  the coverage denominator.
- `path_follower_config.yaml` ships with `lookahead_distance: 0.8` and
  `max_search_ahead: 25`. Larger values caused oscillation at U-turns.
- `path_follower_config.yaml` has `obstacle_stop_range: 0.70 m` (HOLD trigger)
  and `front_clear_range: 1.0 m` (HOLD→FORWARD clear threshold). The 0.30 m
  hysteresis between these values prevents mode chatter.
- `path_follower_config.yaml` does **not** set `done_dwell_time` — the code
  default of 2.0 s is used. The robot must hold the final coverage pose for
  2.0 s before `/path_follower/done` fires, filtering out end-of-path wiggle.
- `local_replanner_config.yaml` has `obstacle_inflate_m: 0.7` for dynamic
  obstacles. This is intentionally less than `robot_radius: 1.0` — making
  them equal or larger causes "Goal cell occupied" A* errors when computing
  detours near walls.
- `MAX_ACCEL`, `MAX_STEERING`, `MIN_STEERING` in `path_follower.py` are loaded
  from `/hakuroukun_steering_controller/` namespace, which is **not set** by
  any real-robot launch file. Code defaults (2.5, ±0.78 rad) are used silently.
  At ±0.78 rad the steering commands stay within the Arduino's acceptance range.
- `conemap_planning.yaml` uses an **absolute image path**
  (`/root/catkin_ws/src/...`). This only works inside the container.
- All private params on `hakuroukun_pose_node` (`~rotation_angle`,
  `~gps_to_rear_axis`, `~imu_mode`, etc.) are dead — the node never calls
  `rospy.get_param` for any of them. Values in the launch file are silently
  ignored. Do not assume changing them has any effect.
- `tsnd151_imu_node` publishes on `/imu` (confirmed from source). This matches
  `hakuroukun_pose.py`'s subscriber topic — no remap needed.
- `sensor_bringup.launch` has no `/fix` → `/gps/fix` remap.
  `bringup_hakuroukun_robot.launch` starts `hakuroukun_pose_node` as a `<node>`
  directly — the old workaround of running it via `rosrun` is no longer needed.