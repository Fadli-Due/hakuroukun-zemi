# hakuroukun_ws
## Code for Hakuroukun Cleaning Robot

### Setting up the enviroment

-----

* **Enable GUI within Docker containers**

  > **! Caution:** This method exposes PC to external source. Therefore, a more secure alternative way is expected for using GUI within Docker containers. This problem was raised in [Using GUI's with Docker](https://wiki.ros.org/es/docker/Tutorials/GUI#:~:text=%2D%2Dpulse.-,Using%20X%20server,-X%20server%20is)

```bash
#This command is required to run every time the PC is restarted
xhost + 
```
Make a X authentication file with proper permissions for the container to use.

```bash
# If not working, try to run "sudo rm -rf /tmp/.docker.xauth" first
cd ./src/hakuroukun_dockerfiles/
chmod +x ./install/xauth.sh && ./install/xauth.sh
```

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
2. GPS Rotation angle _ calibration :
    - need to calculate and update : ``` <param name="~rotation_angle" value="-90"/> ```
    - how measure : 
        - The robot in default position is heading in y axis
        - Run the robot and get coordinate from GPS by manual mode
        - Calculate the rotation angle by coordinate

3. Arduino firmware :
    - Upload `${hakuroukun_communication}/firmware/motor_control/motor_control.ino` to Arduino with Arduino IDE.

4. Bringup execution :
In the 1st terminal
    ```
    docker exec -it hakuroukun-robot bash
    roslaunch hakuroukun_launch bringup.launch
    ```

4. Controller execution : 
In the 2nd terminal
    ```
    docker exec -it hakuroukun-robot bash
    roslaunch hakuroukun_launch experiment_controller.launch
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
    at ~0.45 m. Stops the robot in milliseconds for sudden intrusions
    (a child running into the path, etc.).
  - **Layer 2 — Local replanner** (`local_replanner.py`): Watches LiDAR
    for obstacles that **persist on the path for ≥ 7 seconds**, then splices
    an A\*-computed detour into the baseline. Yields to transient obstacles
    (pedestrians passing) but routes around static blockers.
- **Restricted zones via cones**: Cones are pre-painted into the planning
  map (Phase A). Real-time ZED2-based detection (Phase B) is deferred.

### Topic flow

```
offline_coverage_planner ──/planned_path──> local_replanner ──/desired_path──> path_follower
                                                  ↑                ↑
                                           /scan_multi (LiDAR)     │
                                                           return_to_start
                                                        (appends /return_path
                                                         on /path_follower/done)
```

`/planned_path` is the latched baseline BCD path. `/desired_path` is what
the follower actually tracks — equal to baseline when no detour is active,
spliced detour when one is. Once the follower publishes `/path_follower/done`,
`return_to_start.py` computes an A\* path back to `baseline[0]` and publishes
`/return_path`; `local_replanner.py` appends it to `current_path` automatically.

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
- `[pf] mode=FORWARD v=0.25 steer=...`

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
  Estimate" tool (same workflow people already use for AMCL initialization).
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

#### 2. Robot setup

**a. Connect the PC to all sensors and control devices** — RTK-GPS,
TSND151 IMU, dual RPLIDARs, Arduino. Confirm each device shows up:
```bash
cat /dev/ttyACM*
```

**b. Measure the GPS rotation angle and update the bringup launch.**
This step must be done at the start of **every experiment day** — the value
depends on how the robot is oriented relative to the GPS axes on that
particular setup.

- Place the robot in its default heading.
- Drive a short straight segment in manual mode and record GPS coordinates.
- Compute the heading angle from the recorded coordinates.
- Update the value in `hakuroukun_boustrophedon_with_cones/launch/bringup_hakuroukun_robot.launch`:
  ```xml
  <param name="~rotation_angle" value="86.5"/>   <!-- replace with today's measurement -->
  ```

If this value is wrong, every pose downstream is wrong, so the offline
path will follow correctly in the map frame but the robot's physical
trajectory will be rotated. Don't skip this.

**c. Upload the Arduino firmware.**
Open the Arduino IDE
(`arduino-ide_2.3.4_Linux_64bit.AppImage` in the downloads folder) and
upload one of:

- `${hakuroukun_communication}/firmware/motor_control/motor_control.ino` —
  for autonomous runs (this is what you want for the coverage experiment).
- `${hakuroukun_communication}/firmware/manualmode/manualmode.ino` —
  for manual control (useful during cone placement and the rotation-angle
  calibration drive).

#### 3. Run experiment (5 terminals)

Each terminal needs its own `docker exec -it hakuroukun-robot bash`.

**T1 — Robot bringup** (sensors, LiDAR stack, `hakuroukun_pose_node`):
```bash
roslaunch hakuroukun_boustrophedon_with_cones bringup_hakuroukun_robot.launch
```

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

Even with `rotation_angle` correctly calibrated, two coordinate frames still
need to be aligned for the robot icon in RViz to land on the correct map pixel:

- **`map` frame** — the painted PGM. (0, 0) is at one corner of
  `conemap_planning.pgm`, fixed when the map was made.
- **`odom` frame** — where `hakuroukun_pose_node` anchored its local frame
  on first power-up. Decided by where the robot happens to be at startup.

These almost never coincide by chance, so on every new experiment session
you'll see the robot icon sitting in the wrong place. The
`map_odom_calibrator` node fixes this without requiring a relaunch:

1. T1 and T4 are running. The robot icon is in the wrong place. Expected.
2. In RViz, click the **"2D Pose Estimate"** tool in the top toolbar.
3. Click on the map where the robot actually is, and drag in the direction
   it's actually facing.
4. The node receives `/initialpose`, computes the offset, and the icon
   snaps to the correct position. Log line in T4:
   `[map_odom_calibrator] map->odom updated: x=… y=… yaw=…`
5. If the icon drifts later, click again. No relaunch needed.

This is the same calibration workflow people use for AMCL on indoor robots —
the only difference is that here it's our updatable `map → odom` that gets
corrected, instead of AMCL's particle filter.

For best results, calibrate while the robot is **stationary** and check
against a second known waypoint before starting a coverage run.

### Simulation results (thesis runs)

Both runs used the conemap world with `obstacle_inflate_m: 0.6` and
`obstacle_stop_range: 0.70` (enforcing the invariant: stop distance > inflation
radius).

| Run | Bag | Coverage | Area | Detours fired |
|-----|-----|----------|------|---------------|
| Run 1 | `conemap_with_return_20260626_031733.bag` | **65.20%** | 488.05 m² | 1 |
| Run 2 | `conemap_run_20260626_052306.bag` | **63.96%** | — | 2 |

Baseline comparison: Tai's TASP = 52.80%.

Run 2 was a parameter-tuned repeat that fixed the A\* start-cell-inside-inflation-bubble
bug introduced when changing `obstacle_inflate_m`. Both runs confirmed a 263-point
return-to-start path executed at the end of coverage.

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