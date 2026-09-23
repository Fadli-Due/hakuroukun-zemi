# hakuroukun_ws
## Code for Hakuroukun Cleaning Robot

This workspace holds all packages for the Hakuroukun outdoor ride-on cleaning
robot (retrofitted IPC Silver 800, ~304 kg, Ackermann steering). The main
thesis package is `hakuroukun_boustrophedon_with_cones` (Due, 2026): an
offline Boustrophedon Cell Decomposition (BCD) coverage planner with a
two-layer online safety architecture. The predecessor package
`hakuroukun_tasp_with_cones` (Nguyen Van Tai, 2024) is kept as a reference
baseline.

### Workspace root

```
src/
├── hakuroukun_boustrophedon_with_cones/  ← thesis package (BCD + online safety)
├── hakuroukun_tasp_with_cones/           ← Tai's TASP baseline (reference)
├── hakuroukun_communication/             ← Arduino firmware + comms node
├── hakuroukun_control/                   ← lower-level control utilities
├── hakuroukun_description/               ← URDF, meshes
├── hakuroukun_dockerfiles/               ← Dockerfile, docker-compose.yml, install/, rules/ (udev)
├── hakuroukun_launch/                    ← shared bringup (sensor, communication, lidar)
├── hakuroukun_localization/              ← hakuroukun_pose (GPS+IMU+wheel fusion), obstacle_detection
├── hakuroukun_navigation/                ← sdv_msgs, trajectory_generation
├── hakuroukun_sensor/                    ← RPLIDAR, TSND151 IMU, u-blox F9P GPS drivers
├── hakuroukun_steering_controller/       ← ros_control plugin
├── DF_Mapping.md                         ← SUPERSEDED, old gmapping attempt; D-F map is now drawn (see D-F field mapping)
├── paint_polygons.py                     ← interactive PGM polygon painter (restricted zones)
├── sync_to_docker.sh                     ← host↔container inotify sync helper
├── frames.pdf                            ← TF tree snapshot
├── rosgraph.png                          ← node/topic graph snapshot
├── gps_calib_log.txt                     ← historical GPS calibration log
└── CMakeLists.txt
```

### Setting up the environment

-----

* Enable GUI within Docker containers

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
> container at `/root/catkin_ws/src`. Edits on the host sync automatically,
> no `docker cp` needed. `catkin_make` is still required after adding new nodes
> or changing `CMakeLists.txt`.
>
> **Docker lifecycle:** Use `docker compose stop` (not `down`) to preserve
> `build/` and `devel/` between sessions. `down` wipes build artifacts and
> forces a full `catkin_make` on next start.
>
> **`ROS_IP=127.0.0.1` in `docker-compose.yml`:** both the `ros-master` and
> `hakuroukun-robot` services set `ROS_IP=127.0.0.1` explicitly. Without
> this, Ubuntu's hostname on the lab laptop resolves to `127.0.1.1`, which
> silently breaks TCP between ROS nodes running in the two containers,
> `/planned_path` publishes but nothing subscribes, no error message. If
> you migrate to a new machine, verify these lines are still present.
>
> **Stale roscore state (2026-07-11):** Running many sim/real launches back
> to back on the same `roscore` session without a clean restart has caused
> repeated silent failures, stale `/use_sim_time`, stale `~skip_init_gate`
> rosparams left over from a previous sim run, and outright node-name
> collisions (`external shutdown request ... Reason: new node registered
> with same name`) that kill unrelated nodes mid-run. If behavior looks
> inexplicable (a node silently dies, a gate bypasses itself, `rosparam get`
> shows a value nobody set today), do a full clean restart before debugging
> further:
> ```bash
> rosnode kill -a
> pkill -9 -f roslaunch
> pkill -9 -f rosmaster
> pkill -9 -f rosout
> ps aux | grep ros   # confirm empty before relaunching
> ```

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
    - As of 2026-07-12, `rotation_angle` is no longer hardcoded.
      `hakuroukun_pose.py` loads it at startup from a calibration file
      (`hakuroukun_pose/config/gps_rotation_calibration.yaml`), written by
      `gps_rotation_calibrator.py`. If the file is missing/unreadable, it
      falls back to the last known-good hardcoded value (172.1°) and logs a
      warning, it never silently uses 0°.
    - **Why this changed:** the GPS unit gets borrowed/remounted by other
      lab members, which silently invalidates any hardcoded rotation angle
      with no error signal, producing "confident but wrong" path tracking
      (the robot follows its own incorrect idea of where it is, cutting
      across coverage rows instead of tracking `/desired_path`). See the
      `gps_rotation_calibrator.py` section below for the recalibration
      workflow. The launch file param `~rotation_angle` remains dead, the
      node never reads it; do not try to set rotation angle via the launch
      file.

3. Arduino firmware:
    - Upload `${hakuroukun_communication}/firmware/bcd_automode/bcd_automode.ino`
      for autonomous BCD runs (see Real Robot section for full firmware workflow).
    - Confirm calibration constants match the Python side before every
      experiment day, see section 2.d below. These drift whenever the
      steering/accel pots are physically touched (remounting, borrowing,
      mechanical adjustment).

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

## `hakuroukun_boustrophedon_with_cones`: Offline BCD coverage with online obstacle handling

This package is my master's thesis work (Due, 2026). It is an offline
Boustrophedon Cell Decomposition (BCD) coverage planner with an online
local replanner that modifies the path when obstacles are detected. It
replaces Tai's online TASP planner from the previous study.

### What it does

- Offline planning: A complete coverage path is computed once from a
  pre-built occupancy map (BCD decomposition + A\* transit segments + pure
  pursuit-following waypoints).
- Online modification (two layers):
  - Layer 1, reflex stop (in `path_follower.py`): LiDAR `FORWARD VETO`
    at 0.90 m. Stops the robot in milliseconds for sudden intrusions
    (a child running into the path, etc.).
  - Layer 2, local replanner (`local_replanner.py`): Watches LiDAR
    for obstacles that persist on the path for ≥ 7 seconds, then splices
    an A\*-computed detour into the baseline. Yields to transient obstacles
    (pedestrians passing) but routes around static blockers. Detours are
    permanent commitments, once an obstacle is classified as static
    (persistence ≥ 7 s), the spliced detour stays in `current_path` for
    the rest of the run. The old "restore baseline when LiDAR loses sight"
    behavior was removed (2026-06-25): losing sight of a static obstacle
    doesn't mean it's gone, and reverting to the pristine baseline would
    contradict the persistence model.
- Return-to-start (`return_to_start.py`): After coverage completes
  (path_follower reports robot has held final pose for ≥ 2 s), an A\* path
  from current pose back to `baseline[0]` is computed and appended to
  `current_path`. The robot drives home automatically. Persistence-gated
  obstacle avoidance keeps running on the return leg, if someone walks
  into the corridor, a detour is spliced onto the return path the same
  way it would be on a coverage lane.
- Restricted zones via cones: Cones are pre-painted into the planning
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
the follower actually tracks, equal to baseline when no detour is active,
spliced detour when one is, with the return-to-home leg appended after
coverage completes. `/path_follower/done` is a latched Bool fired once
when the robot has held the final coverage pose for `done_dwell_time`
seconds; this triggers `return_to_start` to compute the A\* return path.

`offline_coverage_planner.py` also publishes a latched `/bcd_valid_area_m2`
(std_msgs/Float32) with the EDT-inflated free-space area, this is the
denominator both `cleaning_simulator.py` (live coverage %) and
`evaluate_real_run.py` (post-hoc coverage %) use, so the two match.

**Init handshake (real robot only):** `map_odom_calibrator` also publishes
a latched `/map_odom_calibrator/initialized` Bool that starts `False` and
flips `True` on the first 2D Pose Estimate click in RViz. `path_follower`
gates its main loop on this flag, it sends no motion commands until
the click has landed. This prevents the robot from acting on an
uninitialized map-frame yaw (root cause of the 2026-07-05 real-robot
failure to move). In simulation the handshake is bypassed via
`~skip_init_gate:=true` on the path_follower node, because AMCL owns the
map frame directly and there is no calibrator to click.

> **Watch for stale `~skip_init_gate` on real-robot launches (2026-07-11):**
> this param has shown up set `True` on a real-robot launch with no
> corresponding `<param>` line in the launch file itself, a leftover
> rosparam from an earlier sim session on the same long-lived `roscore`.
> If `path_follower`'s startup log shows `~skip_init_gate=True. Gate 0
> bypassed` on a real run, do not proceed: `rosparam delete
> /path_follower/skip_init_gate` (and check `rosparam list | grep
> skip_init_gate` for other nodes too, `offline_coverage_planner` has its
> own separate instance of this same gate and param name) and restart
> clean. See the "Stale roscore state" note near the top of this README.

### Package layout

```
hakuroukun_boustrophedon_with_cones/
├── config/
│   ├── amcl_params.yaml                 ← AMCL tuning (simulation only)
│   ├── boustrophedon_config.yaml        ← BCD parameters (robot_radius=1.0, lane_spacing=1.3, turn_margin=0.80)
│   ├── path_follower_config.yaml        ← pure pursuit (lookahead=0.8, search_ahead=25, obstacle_stop_range=0.90)
│   ├── local_replanner_config.yaml      ← persistence=7s, window=15m, lookahead=7m, obstacle_inflate=1.2m
│   ├── costmap_params.yaml
│   ├── gmapping_params.yaml             ← sim mapping (mapping_warehouse.launch)
│   ├── gmapping_params_real.yaml        ← SUPERSEDED: real D-F gmapping (map is now drawn, not SLAM)
│   └── gps_rotation_calibration_sim.yaml ← sim override (both values 0.0; real value lives in hakuroukun_pose/)
├── launch/
│   ├── bringup_hakuroukun_conemap_sim.launch     ← Gazebo + robot (conemap world)
│   ├── bringup_hakuroukun_warehouse_sim.launch   ← Gazebo + robot (warehouse world)
│   ├── bringup_hakuroukun_df.launch              ← Gazebo + robot (D-F world, sim mirror of outdoor site)
│   ├── bringup_hakuroukun_df_flat.launch         ← Gazebo D-F on flat ground (no terrain mesh)
│   ├── bringup_hakuroukun_robot.launch           ← Real robot bringup
│   ├── offline_path_planning_conemap.launch      ← Planner + replanner + follower (conemap sim, AMCL)
│   ├── offline_path_planning.launch              ← Planner + replanner + follower (warehouse sim, AMCL)
│   ├── offline_path_planning_df.launch           ← Planner + replanner + follower (D-F sim)
│   ├── offline_path_planning_df_outline.launch   ← D-F outline-map variant
│   ├── offline_path_planning_real.launch         ← Planner + replanner + follower + return-to-start (real robot, no AMCL)
│   ├── df_mapping_real.launch                    ← SUPERSEDED: gmapping the D-F field (abandoned; map is now drawn)
│   ├── df_mapping.launch / df_mapping_minimal.launch
│   ├── mapping_warehouse.launch                  ← gmapping (sim, only needed to rebuild warehouse map)
│   ├── slam_gmapping.launch
│   ├── gazebo_cones.launch / gazebo_cones_warehouse.launch
│   ├── gazebo_df.launch / gazebo_df_flat.launch
│   ├── gps_test.launch
│   └── straight_line_test.launch                 ← GPS rotation angle validation
├── maps/
│   ├── conemap_planning.{pgm,yaml}      ← main map used for simulation thesis experiments
│   ├── conemap_walls_cones.{pgm,yaml}
│   ├── df_area_outline_rotated.{pgm,yaml} ← real-world thesis map (drawn from GPS corners, rotated 8.3°; 308.75 m² valid)
│   ├── df_area_outline_painted.{pgm,yaml} ← outline + painted restricted zones (input to rotate_map.py)
│   ├── df_area_outline.{pgm,yaml}       ← bare drawn outline (before painting/rotation)
│   ├── df_area_planning.{pgm,yaml}      ← predecessor real map (pre-rotation naming)
│   ├── rotate_map.py                    ← rotates the painted map 8.3° into the field-aligned frame
│   ├── warehouse_map.{pgm,yaml}
│   └── saved_map.{pgm,yaml}
├── scripts/
│   ├── planning/
│   │   ├── offline_coverage_planner.py   ← BCD; publishes /planned_path, /bcd_valid_area_m2
│   │   ├── return_to_start.py            ← A* return-to-home leg after /path_follower/done
│   │   └── simple_astar.py
│   ├── control/
│   │   ├── path_follower.py              ← pure pursuit + reflex stop + init-handshake gate
│   │   ├── local_replanner.py            ← online persistence-gated detour layer
│   │   ├── map_odom_calibrator.py        ← dynamic map→odom TF + init-handshake publisher
│   │   ├── odom_tf_broadcaster.py        ← real-robot odom→base_link TF
│   │   ├── map_manager.py                ← shared map-loading helpers
│   │   ├── keyboard_teleop.py            ← manual driving w/ keyboardmode.ino (needs python3)
│   │   ├── sim_teleop_key.py             ← sim-only Twist teleop
│   │   └── publish_initial_pose.py       ← programmatic AMCL initialpose helper (sim)
│   ├── evaluation/
│   │   ├── cleaning_simulator.py         ← live coverage % (publishes /cleaned_map)
│   │   ├── calculate_coverage_from_image.py
│   │   ├── caculate_average_error.py     ← Savg / Smax / RMS trajectory error (live)
│   │   ├── caculate_coverage.py
│   │   └── evaluate_real_run.py          ← post-hoc rosbag → all thesis metrics (see below)
│   ├── obstacles/
│   │   ├── cone_zone_creator.py
│   │   └── cones_detector_simulation.py
│   └── tools/
│       ├── test_obstacle_spawner.py      ← spawn static box / pedestrian in Gazebo
│       ├── verify_map.py
│       ├── visualizer.py
│       └── health_check.py
├── worlds/
│   ├── 30x30area.world                   ← conemap world (main sim thesis map)
│   ├── 10x10area.world
│   ├── df_area.world                     ← D-F outdoor site mirror
│   ├── flat_ground.world
│   └── hakuroukun_warehouse_v1.world
├── rviz/
│   ├── boustrophedon_with_cones.rviz
│   ├── df_mapping.rviz
│   ├── mapping.rviz
│   ├── straight_line_test.rviz
│   └── view_robot.rviz
├── acrhive/                              ← archived old launch/config/results (sic: folder name)
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

### Simulation: conemap (main thesis scenario)

The conemap uses pre-painted cone polygons as restricted zones. It is the
main environment for thesis simulation quantitative results.

Use 5 terminals in this order. Each terminal needs its own
`docker exec -it hakuroukun-robot bash`.

**T1. Gazebo + bringup:**
```bash
roslaunch hakuroukun_boustrophedon_with_cones bringup_hakuroukun_conemap_sim.launch
```

**T2. Rosbag (start BEFORE the robot begins moving):**
```bash
mkdir -p /root/catkin_ws/bags && cd /root/catkin_ws/bags
rosbag record -a -O conemap_run_$(date +%Y%m%d_%H%M%S).bag \
    __name:=conemap_recorder
```
`-a` records all topics. The `__name` lets you stop the bag cleanly with
`rosnode kill /conemap_recorder` once the run finishes, this flushes
the bag properly so it doesn't get left in `.active` state.

T3. Cleaning simulator (publishes `/cleaned_map` from the robot's pose):
```bash
rosrun hakuroukun_boustrophedon_with_cones cleaning_simulator.py
```

**T4. Planner + follower + AMCL + RViz:**
```bash
roslaunch hakuroukun_boustrophedon_with_cones offline_path_planning_conemap.launch
```
Wait for the green coverage path to appear in RViz and for AMCL's particle
cloud to converge tightly around the robot (a few seconds). Don't skip
this check, if the cloud stays diffuse or is offset, abort and reset
before continuing.

> **Sim-only param:** the sim launch sets `~skip_init_gate:=true` on the
> `path_follower` node to bypass the real-robot init handshake. AMCL owns
> the map frame in sim, so there is no `map_odom_calibrator` to click.
> path_follower's log will print a `~skip_init_gate=True — Gate 0 bypassed`
> warning at startup, which is expected in sim.

Useful log lines to confirm a healthy start:
- `[BCD] N coverage cells`
- `[BCD] published /planned_path with N points`
- `[local_replanner] baseline path received: N points`
- `[return_to_start] ready. robot_radius=1.00 densify=0.20`
- `[return_to_start] baseline received: home = (X, Y)`
- `[return_to_start] static map ready (WxH, res=...m)`
- `[path_follower] ~skip_init_gate=True — Gate 0 bypassed` (sim only)
- `[pf] mode=FORWARD v=0.25 steer=...`

At the end of coverage, the return leg sequence should look like this:
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

**T5. Live coverage readout:**
```bash
rosrun hakuroukun_boustrophedon_with_cones calculate_coverage_from_image.py
```

**Stopping the run:** kill the rosbag first so it flushes properly, then
shut down the launches:
```bash
rosnode kill /conemap_recorder
```

#### RViz displays to add

- `/cleaned_map` (Map, alpha ~0.5), coverage trail.
- `/planned_path` (Path), blue baseline BCD path.
- `/desired_path` (Path), what the follower is tracking *now*.
- `/return_path` (Path). A\* return-to-home leg (appears at end of coverage).
- `/local_replanner/obstacle_grid` (Map), persistent obstacle cells.
- `/local_replanner/markers` (MarkerArray), blue baseline + orange active detour.
- `/particlecloud` (PoseArray). AMCL convergence check.

### Simulation: warehouse

Same 5-terminal pattern, swap the bringup and planning launch files:
```bash
# T1
roslaunch hakuroukun_boustrophedon_with_cones bringup_hakuroukun_warehouse_sim.launch
# T4
roslaunch hakuroukun_boustrophedon_with_cones offline_path_planning.launch
```
T2 (rosbag), T3 (`cleaning_simulator.py`), and T5 (`calculate_coverage_from_image.py`)
are identical to the conemap flow.

> **`~skip_init_gate` in warehouse launch:** if you haven't already, add
> `<param name="skip_init_gate" value="true"/>` to the `path_follower`
> node block in `offline_path_planning.launch` too. Without it,
> path_follower will hang at Gate 0 forever waiting for a calibrator
> that AMCL-based sim doesn't run.

### Testing the online replanner

Three obstacle scenarios verify the two-layer architecture:

- Scenario A, static box on the path → reflex stop, then detour.
- Scenario B, pedestrian passes briefly (3 s) → reflex stop, no detour.
- Scenario C, pedestrian stays > 7 s → reflex stop, then detour.

Detailed steps, expected logs, and pass criteria are in
[`SIM_TEST_PROCEDURE.md`](./hakuroukun_boustrophedon_with_cones/SIM_TEST_PROCEDURE.md).

Quick example (Scenario A):
```bash
# T3 — spawn a static box ~10 m ahead of the robot
docker exec -it hakuroukun-robot bash
rosrun hakuroukun_boustrophedon_with_cones test_obstacle_spawner.py \
    static --x 5.0 --y 2.0
```

### D-F outdoor field mapping

The real-world thesis map is drawn offline from surveyed GPS corners. It is not
built with SLAM. We tried gmapping and gave up on it: the D-F field is open and
flat, so with no walls or fences for the scan to lock onto, the scan matching
diverges (you get `Likelihood ≈ -166` in the log). The old gmapping procedure in
[`DF_Mapping.md`](./DF_Mapping.md) is kept for reference only. Do not follow it.

You only need to rebuild the map if the field layout changes, or if the GPS
calibration drifts far enough that the map no longer lines up. The map in use is
`maps/df_area_outline_rotated.{pgm,yaml}`, which is what Run 1 and Run 2 used.

There are three offline steps, no SLAM:

1. Survey the corners. Drive the robot to each boundary corner, stop, and read
   its map-frame `(x, y)`. The pose node turns the GPS `/fix` into local XY and
   publishes it here:
   ```bash
   rostopic echo -n1 /hakuroukun_pose/rear_wheel_odometry/pose/pose/position
   ```
   Check that `/fix status` is 2 (RTK fixed) at every corner. Since this XY comes
   from the GPS conversion, run the GPS rotation calibration first (see the
   `gps_rotation_calibrator.py` section). Go around the perimeter in order.

2. Draw the outline with `map_draw.py` (in the package root). Put the surveyed
   corners in its `corners` list. It fills the inside as free space, draws the
   boundary as a wall, leaves the outside unknown, and writes the `.pgm` and
   `.yaml` (0.05 m/px, 2 m margin, origin computed from the corners). Change the
   hard-coded output paths to your own machine before running it.

3. Paint the restricted zones, then rotate. Paint the cone and keep-out polygons
   onto the outline with `paint_polygons.py` (a small matplotlib painter that
   writes a P5 PGM using only the `{0, 205, 254}` pixel values), and save it as
   `df_area_outline_painted.*`. Then run `maps/rotate_map.py`. It rotates the
   painted map 8.3° counter-clockwise around its centre so the field's X and Y
   axes come out straight, and recomputes the YAML origin and yaw so the world
   coordinates still match. The result is `df_area_outline_rotated.{pgm,yaml}`.

The 8.3° is a value we tuned by hand to match the current GPS calibration. If you
re-survey the field or the GPS rotation changes, open the map in RViz and adjust
`alpha_deg` in `rotate_map.py` until the outline sits on top of the driven GPS
track.

`offline_path_planning_real.launch` already loads `df_area_outline_rotated.yaml`
by default, so once it is in `maps/` the real run uses it with no extra flags. If
you only need to change the restricted zones on a map you already have, re-paint
with `paint_polygons.py` and run `rotate_map.py` again.

### Real robot

This section follows the same structure as Tai's `hakuroukun_tasp_with_cones`
real-robot procedure (Setup → Robot setup → Run experiment), so it should
read side-by-side with the previous lab documentation.

Key differences from simulation:
- No AMCL. Localization comes from RTK-GPS + IMU via `hakuroukun_pose.py`
  (the live node, see the "hakuroukun_pose.py entry point" note below;
  there is a dead duplicate `hakuroukun_pose_node.py` in a sibling folder
  that must not be edited or launched).
- `map → odom` is published by `map_odom_calibrator.py`. It starts identity
  and is updated at runtime when you click in RViz with the "2D Pose
  Estimate" tool. The calibrator also owns the init handshake (see below):
  it publishes a latched `/map_odom_calibrator/initialized` Bool that
  `path_follower` gates on. Until you click, path_follower will not send
  motion commands.
- `odom → base_link` is published by `odom_tf_broadcaster.py` from
  `/hakuroukun_pose/rear_wheel_odometry`.
- The planning map is `df_area_outline_rotated.yaml` (the default in
  `offline_path_planning_real.launch`), not `conemap_planning.yaml`. It is the
  drawn outline rotated 8.3° to line up with the field (see the D-F outdoor field
  mapping section). Run 1 (58.57% coverage) and Run 2 (55.94%) both used it.
- The cones are painted into the planning map, and the physical cones are placed
  on the ground to match before the run. The planner treats them as restricted
  zones and plans around them. It would be good to also detect the cones live
  with the LiDAR, but the planner does not depend on that, and in the field runs
  the reflex-HOLD did not detect the low cones reliably (see Real-world results).

#### 1. Set up experiment environment

Place the physical cones on the ground at positions matching the polygons
painted into `df_area_outline_rotated.pgm`. The cones define restricted zones
the planner must not cross.

- 10 cones total.
- Cone coordinates can be re-extracted from the planning map with
  `scripts/tools/` if needed.

> **obstacle_stop_range note:** `path_follower_config.yaml` sets
> `obstacle_stop_range: 0.90 m`. The robot will enter HOLD if the LiDAR
> sees anything within 0.90 m, including physical cones. If the cone
> positions on the ground don't perfectly match the painted polygons in the
> map, the robot may trigger unexpected HOLDs near cone boundaries. Watch
> for this on the first run.
>
> **`front_stop_range` note (2026-07-11):** there is a related but
> separate legacy REVERSE-recovery threshold, `front_stop_range` in
> `path_follower_config.yaml` (currently 1.50 m). If this sits between
> `obstacle_stop_range` (0.90 m) and the intended HOLD trigger, obstacles
> in that band can fall through to legacy REVERSE recovery instead of
> the persistence-gated A* replanner, bypassing the thesis's core
> replanning contribution for that obstacle. Confirm `front_stop_range`
> matches your intended test (drop to ~0.90 for pure replanner testing;
> the current 1.50 is acceptable only when you deliberately are NOT
> testing the replanner that day).

#### 2. Robot setup

**a. Recreate device symlinks (every session, do not persist across reboots):**
```bash
sudo ln -sf /dev/ttyACM0 /dev/imu
```
Confirm all three symlinks exist before launching:
```bash
ls /dev/arduino /dev/imu /dev/gps
```

> Port mapping is NOT fixed, verify every session. Port mapping has
> been observed to change between sessions (not just across reboots).
> Always identify fresh with `cat /dev/ttyACM0`, `cat /dev/ttyACM1`,
> `cat /dev/ttyACM2` before creating symlinks:
> - Silent output (nothing prints) → IMU (TSND151 doesn't stream
>   unprompted; silence is the positive signal, not a dead port).
> - Readable NMEA text (`$GPGGA`, `$GPRMC`, ...) → GPS.
> - Garbled/binary junk → Arduino.
>
> Do not assume yesterday's (or even this morning's) mapping still holds.
>
> **IMU permissions:** the IMU serial device sometimes comes up without
> world-write permission and the pose node fails to open it. Fix, every
> session:
> ```bash
> sudo chmod a+rw /dev/imu
> ```
> This does not persist across reboots.

b. Connect the PC to all sensors and control devices. RTK-GPS,
TSND151 IMU, dual RPLIDARs, Arduino. Confirm each device shows up:
```bash
cat /dev/ttyACM*
```

c. GPS rotation angle calibration, now file-based, not hardcoded.

> **As of 2026-07-12:** `hakuroukun_pose.py` loads `rotation_angle` from
> `hakuroukun_pose/config/gps_rotation_calibration.yaml` at startup
> (`_load_rotation_angle()`), falling back to the historical hardcoded
> 172.1° only if the file is missing or unreadable. The launch file param
> `~rotation_angle` remains dead and is silently ignored, do not try to
> set it there.
>
> **Recalibration workflow:** run `gps_rotation_calibrator.py` (see its
> own section below) during the manual drive from E building to D-F. It
> opportunistically harvests straight, sustained-speed segments from that
> drive, cross-checks them against each other, and writes a fresh
> calibration file on shutdown (Ctrl+C), only if the segments agree
> closely enough to trust. If it can't produce a confident result, it
> refuses to write and the previous calibration stays in effect (a stale
> calibration is safer than a silently wrong new one).
>
> Recalibrate whenever the GPS unit has been borrowed, unmounted, or
> physically touched by anyone, this is the most common way this value
> goes silently stale, since a remounted antenna changes the true rotation
> offset with no error signal anywhere else in the stack.

d. Steering and accel calibration (do this before every experiment day,
or whenever the pots/pedal mechanism have been touched).

The pot readings for neutral, and the full mechanical range, must be
consistent across the Arduino firmware and the Python communication node.
Measure fresh with `keyboardmode.ino` rather than trusting old numbers if
there's any chance the hardware moved:

1. Upload `keyboardmode.ino` via Arduino IDE.
2. Run keyboard teleop (use `python3` explicitly, the script uses
   f-strings, Python 2 will fail with a SyntaxError):
   ```bash
   python3 /root/catkin_ws/src/hakuroukun_boustrophedon_with_cones/scripts/control/keyboard_teleop.py
   ```
3. **Steering:** on a flat surface, find wheels-straight and read `pm_st`.
   Then drive to each mechanical stop (left, right) and read `pm_st` at
   each. **Current calibrated values (2026-07-11):**
   - Straight / neutral: `pm_st = 496`
   - Right stop: `pm_st = 279`
   - Left stop: `pm_st = 705`
4. **Accel:** read `pm_ac` with pedal fully released, and fully pressed.
   **Current calibrated values (2026-07-11):**
   - Released / neutral: `pm_ac = 163`
   - Full press: `pm_ac = 796`
   - Breakaway threshold (measured, not a mechanical stop): `pm_ac ≈ 500`.
     Below this the accel pot target changes but the physical
     pedal/drivetrain does not actually move the vehicle, confirmed via
     watchdog trips showing `pm_ac` pinned at neutral despite the Arduino
     trying for the full watchdog window. Any accel mapping used by
     `path_follower`/`hakuroukun_communication_node.py` must clear
     this threshold with margin across its whole operating range,
     including at high steering angles (see note below).
5. Update all of the following with the measured values:
   ```cpp
   // bcd_automode.ino
   #define PM_ST_N     496   // steering neutral
   #define PM_ST_LIMR  217   // PM_ST_N - PM_ST_LIMR = 279 (right stop)
   #define PM_ST_LIML  209   // PM_ST_N + PM_ST_LIML = 705 (left stop)
   #define PM_AC_N     163   // accel neutral (pedal released)
   #define PM_AC_LIMU  633   // PM_AC_N + PM_AC_LIMU = 796 (full press)
   #define PM_AC_LIMD   20   // software noise buffer, not a measured stop
   ```
   ```python
   # hakuroukun_communication_node.py — steering clamp in _apply_indentification()
   steering_command = max(279, min(705, steering_command))

   # _corrected_steering_command() — asymmetric quadratic intercepts
   c_cw  = 496   # straddles PM_ST_N
   c_ccw = 581   # PM_ST_N + 85 (mechanical backlash offset, unchanged in form)

   # accel mapping in _apply_indentification() — rescaled 2026-07-11.
   # The old slope (163 + v*375) only reached pm_ac=257-313 across
   # path_follower's full v=0.25..0.4 range — entirely below the 500
   # breakaway threshold, so the robot was structurally incapable of
   # moving regardless of correct steering/gating. New slope + floor:
   raw_accel = 163 + linear_velocity * 1428
   acceleration_command = max(raw_accel, 560)   # floor above breakaway,
                                                 # applied whenever ANY
                                                 # forward motion is
                                                 # commanded — see note below
   acceleration_command = max(163, min(796, acceleration_command))
   ```
6. Upload `bcd_automode.ino`. The robot is now ready for autonomous runs.

> **Why the accel floor, specifically (2026-07-11 root cause):**
> `path_follower.py`'s `compute_pp()` reduces commanded speed via
> `curvature_penalty` at high steering angles, exactly when the vehicle
> needs the *most* torque (a sharp turn / large heading correction). Under
> the old accel slope, this pushed `v` low enough that the resulting accel
> command dropped back below the 500 breakaway threshold right when it
> mattered most. Observed symptom: the vehicle would rotate in place
> (`yaw` in `hakuroukun_pose.py`'s log changing) while `x, y` stayed
> essentially frozen (`net_disp ≈ 0` in `path_follower`'s log), steering
> pot was moving, but the vehicle never gained enough drive force to
> actually translate. The `max(raw_accel, 560)` floor guarantees enough
> torque whenever *any* forward motion is requested, independent of how
> low `v` gets from the curvature penalty.

> Why both files (steering)? `PM_ST_N`/`PM_ST_LIMR`/`PM_ST_LIML` in the
> Arduino define the center and edges of its range-check window,
> commands outside this window are silently rejected. `c_cw`/`c_ccw` in
> Python are what get sent as steering pot targets. If firmware and
> Python disagree, the robot drives with a constant offset and/or some
> commands are rejected at startup.

> **Steering curve-switching oscillation (fixed 2026-07-11):**
> `_corrected_steering_command()` picks between two asymmetric quadratic
> curves (`c_cw` / `c_ccw`, modeling mechanical backlash) based on whether
> the goal angle increased or decreased versus the previous tick, with
> no deadband. When the true steering intent is nearly flat (small,
> steady heading correction), tick-to-tick floating-point noise in
> `compute_pp()`'s output was enough to flip which curve got used every
> single 100 ms cycle. Since the two curves' intercepts differ by 85
> counts, this produced a real ~70+ count pot target oscillation even
> when the robot's actual desired heading was steady, tripping the motor
> watchdog (target never converges) and generating malformed-command
> rejections. **Fix:** direction is now only re-evaluated when the goal
> angle moves more than a small hysteresis threshold
> (`STEERING_HYSTERESIS_RAD = 0.03`, tune if oscillation reappears) from
> the last *committed* goal angle, not the raw previous tick.

e. Upload the Arduino firmware.
Open the Arduino IDE
(`arduino-ide_2.3.4_Linux_64bit.AppImage` in the downloads folder) and
upload one of:

- `${hakuroukun_communication}/firmware/bcd_automode/bcd_automode.ino`,
  for autonomous BCD coverage runs. Purpose-built for this launch chain:
  proper `\n` terminator, motor watchdog (400 ms), 10 Hz serial rate,
  diagnostic pot-reading feedback.
- `manualmode.ino`, for manual button control (useful during cone placement).
- `keyboardmode.ino`, for keyboard teleop via `keyboard_teleop.py` (driving
  to experiment area, steering/accel calibration, and running
  `gps_rotation_calibrator.py` during the drive, see below).

> After uploading `bcd_automode.ino`, verify the startup diagnostic
> shows the CURRENT calibrated `PM_ST_N` (see section 2.d above), not a
> stale value from a previous calibration. Firmware/Python mismatch on
> this constant has been the single most common real-robot failure mode
> this project has hit.

> **Serial command format:** `bcd_automode.ino` reads with `readStringUntil('\n')`.
> `hakuroukun_communication_node.py` sends `"0{dir}{steer:03d}{accel:03d}00"` +
> `"\r\n"` (10 data chars + `\r\n`). The Arduino strips any trailing `\r` before
> the length check, so the protocol is robust to both `\n` and `\r\n` line
> endings. Do not revert to `motor_control.ino` for autonomous runs, it
> relies on a fragile 2 ms serial timeout instead of a proper terminator,
> and its motor while-loops can hang indefinitely if the potentiometer
> cable is loose.
>
> **Malformed-command rejections (`status=1`, unresolved):** intermittent
> `Arduino rejected command: 10000000,...` warnings, decoded as message
> length != 10, i.e. a malformed/truncated read, have been observed
> periodically during real-robot runs, even after the steering hysteresis
> fix above. Root cause not yet found. Does not appear to block motion
> (isolated drops), but worth investigating if tracking issues persist
> after ruling out calibration/localization causes.
>
> Reverse-gear command rejection is the current dominant hardware
> failure mode. Thesis Run 2 diagnostic: 366 of 408 commanded reverse
> events (89.7%) were not physically executed, the gear actuator
> remained in forward despite the command. This is the primary driver
> of the ~15-19% run-time spent stalled in both field runs, and directly
> caps achievable coverage. Both runs finished the majority of the
> planned path anyway (58.57% / 55.94%), but any recovery maneuver that
> requires reverse is at high risk of stalling. Not yet resolved at the
> firmware/actuator level.

#### 3. Run experiment (5 terminals)

Each terminal needs its own `docker exec -it hakuroukun-robot bash`.

T1. Robot bringup (sensors, LiDAR stack, `hakuroukun_pose.py`):
```bash
roslaunch hakuroukun_boustrophedon_with_cones bringup_hakuroukun_robot.launch
```
> **GPS timeout:** `hakuroukun_pose.py` calls `wait_for_message('/fix', timeout=10s)`
> at startup. If the GPS does not have a fix within 10 seconds, the node raises
> an exception and bringup fails. Always confirm the GPS LED is solid before
> launching T1.
>
> **Expected log after ~5s:** `Gyro Z bias calibrated: <X> deg/s`. Then
> odometry starts publishing on `/hakuroukun_pose/rear_wheel_odometry`.
> If you also see `[hakuroukun_pose] Loaded GPS rotation angle ... deg`,
> the calibration file loaded successfully; if you see `Could not load GPS
> calibration file ... Falling back to hardcoded 172.1 deg`, the file is
> missing or unreadable, the node still starts, but check whether that
> fallback is trustworthy before relying on precise tracking.

**T2. Rosbag (start BEFORE the robot begins moving):**
```bash
mkdir -p /root/catkin_ws/bags && cd /root/catkin_ws/bags
rosbag record -a -O real_run_$(date +%Y%m%d_%H%M%S).bag \
    __name:=real_recorder
```
> Always use `-a` for real runs. `evaluate_real_run.py` (see below) needs
> `/fix`, `/hakuroukun_pose/rear_wheel_odometry`, `/cmd_controller`,
> `/hakuroukun/gear_state`, `/tf`, `/planned_path` (or `/desired_path`),
> and `/map` all together to reproduce the thesis numbers. Selectively
> recorded bags will silently produce partial metrics, the thesis Run 1
> lost gear-rejection and GPS-covariance metrics for exactly this reason.

T3. Cleaning simulator (marks cells under the robot as cleaned, publishes `/cleaned_map`):
```bash
rosrun hakuroukun_boustrophedon_with_cones cleaning_simulator.py
```

**T4. Planner + replanner + follower + return-to-start + RViz:**
```bash
roslaunch hakuroukun_boustrophedon_with_cones offline_path_planning_real.launch
```
This defaults to `map_file:=df_area_outline_rotated.yaml`. Pass
`map_file:=<other>.yaml` on the command line only if you deliberately
want a different map, the thesis results are all on this default.

> **Expected log immediately after T4 comes up:**
> ```
> [map_odom_calibrator] /map_odom_calibrator/initialized = False.
>   path_follower.py is gated until the first click.
> [path_follower] waiting for 2D Pose Estimate click
>   (/map_odom_calibrator/initialized)...
> ```
> The robot will send no motion commands until you click 2D Pose Estimate.
> This is by design (see calibration section below). If instead you see
> `~skip_init_gate=True — Gate 0 bypassed` on a real-robot launch, see the
> "stale `~skip_init_gate`" warning earlier in this README before
> proceeding, do not click through it.

**T5. Live coverage readout:**
```bash
rosrun hakuroukun_boustrophedon_with_cones calculate_coverage_from_image.py
```

**Stopping:** `rosnode kill /real_recorder` first to flush the bag, then shut
down the launches. Then process the bag through `evaluate_real_run.py` for
the reportable metrics (see "Post-run evaluation" below).

#### Calibrating `map → odom` with RViz (required every experiment day)

The 2D Pose Estimate click in RViz serves two purposes on the real robot:

1. Anchors the map frame to the odom frame (same job as AMCL in sim).
   Without this, the robot icon appears at the wrong place because the
   `map` frame (defined by the painted PGM) and the `odom` frame (defined
   by wherever `hakuroukun_pose.py` initialized) don't coincide by chance.
2. Releases the path_follower init handshake. path_follower gates
   its main loop on `/map_odom_calibrator/initialized`. Until the first
   click lands, the robot cannot move, even if all sensors are healthy
   and the coverage path is ready.

This is intentional: on 2026-07-05 the robot commanded saturated steering
on the first tick because the map-frame yaw was uninitialized (IMU
integrated from 0, robot was actually facing West). Enforcing the click
before motion eliminates this failure mode entirely.

**Procedure:**

1. T1 and T4 are running. The robot icon is in the wrong place. Expected.
   path_follower is looping on Gate 0, logging "waiting for 2D Pose
   Estimate click" every 2 seconds. Also expected.
2. In RViz, confirm Global Options → Fixed Frame = `map`.

   > **Critical:** setting the pose estimate while Fixed Frame is `odom`
   > applies the correction in the wrong coordinate frame, producing a
   > large persistent heading error (alpha ~0.72–1.57 rad) that prevents
   > path following even when GPS and IMU are working correctly. This was
   > the root cause of the failed outdoor run on 2026-06-30.
   >
   > `map_odom_calibrator.py` will warn in the log if `/initialpose` arrives
   > with `frame_id != map`, but it will still apply the transform. The
   > warning alone will not save you, check Fixed Frame yourself first.

3. Click the "2D Pose Estimate" tool in the top toolbar.
4. Click on the map where the robot actually is, and drag in the direction
   it's actually facing.
5. Expected log sequence in T4:
   ```
   [map_odom_calibrator] map->odom updated: x=... y=... yaw=... rad
   [map_odom_calibrator] initialization complete — consumers gated on
     /map_odom_calibrator/initialized are released.
   [path_follower] pose initialized via 2D Pose Estimate — main loop released.
   [pf] mode=FORWARD v=... steer=... alpha=<small> closest_i=0/...
   ```
   The robot icon snaps to the correct position and path_follower starts
   sending motion commands.
6. If the icon drifts later, click again. No relaunch needed. The
   handshake flag is one-shot (True on first click, stays True), subsequent
   clicks only correct the TF, they don't reset any state.

> **Guard:** `map_odom_calibrator` will not apply `/initialpose` until it has
> received at least one message on `/hakuroukun_pose/rear_wheel_odometry`. If
> you click 2D Pose Estimate too early (before T1 is fully up), the click is
> silently discarded and path_follower stays gated. Wait for the pose node's
> "Gyro Z bias calibrated" log line before calibrating.

**Sanity check before letting the robot move:** after your click, watch
the `[pf] mode=FORWARD` heartbeat. The `alpha` field is the heading error
to the lookahead point, it should be small (< 0.3 rad). If alpha is
large (e.g. > 1 rad), your click had the wrong heading, click again with
better drag direction before allowing motion.

**Optional pre-run sanity check:** spin the robot in place manually and
watch `/hakuroukun_pose/rear_wheel_odometry`, the XY position should stay
roughly fixed while yaw changes. If the position walks in a circle, the
GPS-to-rear-axle heading correction in `hakuroukun_pose.py` is not working
correctly.

> **LiDAR range note:** The laser merger has `range_min: 0.3 m`. Obstacles
> closer than 0.3 m are invisible to `/scan_multi`. The path follower's
> `obstacle_stop_range` is 0.90 m, well within this window.

---

### `gps_rotation_calibrator.py`: self-calibrating GPS rotation angle

**Location:** `hakuroukun_localization/hakuroukun_pose/src/hakuroukun_pose/gps_rotation_calibrator.py`

**What it does:** subscribes to `/fix` and `/imu` directly (no dependency
on `hakuroukun_pose.py` being up, only the raw sensor drivers from
`sensor_bringup.launch` need to be running). While you drive normally
(`keyboardmode.ino` + `keyboard_teleop.py`) from E building to D-F, it
opportunistically buffers GPS+IMU samples and identifies straight,
sustained-speed segments of the drive. For each qualifying segment it
computes a candidate rotation angle (comparing the raw GPS bearing implied
by that segment's displacement against the IMU's independently-integrated
heading over the same window). On shutdown (Ctrl+C), if enough segments
agree closely enough, it writes the averaged result to
`gps_rotation_calibration.yaml`; if not, it refuses to write and leaves
the previous calibration in place, printing a warning instead of a
silently untrustworthy new value.

**Usage, run this every time you suspect the GPS may have moved (or
routinely, on any commute):**
```bash
# in the terminal you'll use for the drive
source /root/catkin_ws/devel/setup.bash
python3 /root/catkin_ws/src/hakuroukun_localization/hakuroukun_pose/src/hakuroukun_pose/gps_rotation_calibrator.py
```
Start it at the same time as `keyboard_teleop.py`. Keep the robot
stationary for the first few seconds, like `hakuroukun_pose.py`, it
runs its own independent gyro bias calibration (50 stationary samples)
before trusting any yaw data; watch for its own
`Gyro Z bias calibrated: ... deg/s` log line. Then drive normally the rest
of the way, no deliberate "calibration maneuver" needed, it harvests
straight stretches from ordinary driving. Ctrl+C when you arrive at D-F,
before swapping to `bcd_automode.ino`.

Config knobs (top of the script): `MIN_SEGMENT_DIST_M` (default 8 m),
`MAX_YAW_SPREAD_DEG` (default 6°, how straight a segment must be),
`MAX_SEGMENT_AGREEMENT_DEG` (default 3°, max allowed spread between
candidates before the result is refused), `MIN_CANDIDATES_REQUIRED`
(default 2).

**Output file:** `hakuroukun_pose/config/gps_rotation_calibration.yaml`:
```yaml
rotation_angle_deg: 172.4
calibrated_at: "2026-07-12T07:15:00"
segments_used: 3
segment_agreement_deg: 0.6
candidate_angles_deg: [172.1, 172.6, 172.5]
```
`segment_agreement_deg` is the trust indicator, small means the day's
candidates agreed closely; if it's large or the file wasn't updated at
all, treat the current rotation angle with suspicion and re-run on the
next drive. `hakuroukun_pose.py` reads this file automatically at startup
(see section 2.c above); no manual step needed to apply a fresh result
beyond having run the calibrator once.

**Important implementation note:** the calibrator's IMU integration
deliberately matches `hakuroukun_pose.py`'s exact convention,
`angular_velocity.z` is in deg/s (not rad/s), converted with
`math.radians()` before integrating, and the TSND151 driver does not
populate `msg.orientation` (neither node reads it). If this driver's
behavior ever changes, both this script and `hakuroukun_pose.py`'s
`_integrate_yaw()` need to be updated together.

---

### Post-run evaluation: `evaluate_real_run.py`

**Location:** `scripts/evaluation/evaluate_real_run.py`

The canonical post-hoc evaluation tool. Reads a rosbag and produces
every metric reported in the thesis (Chapter 4 sim tables and Chapter 5
real-world tables), so a fresh run can be quoted against the same
methodology with no manual bookkeeping.

**Metrics produced per bag:**
- Coverage % (occupancy-grid raster of trajectory ⊗ 1.0 m cleaning footprint
  against BCD-inflated free cells from `/bcd_valid_area_m2`, falling back
  to raw free cells if the latched Float32 wasn't recorded).
- Trajectory error `Savg` / `Smax` / `RMS` (nearest-neighbor, planned →
  actual), the same nearest-neighbor method Tai (2024) used, so results
  are directly comparable across baselines.
- Duration, path length, mean speed in motion, % of run in motion.
- Stalls (`< 0.02 m/s for ≥ 3.0 s`): count + total time + % of run.
- Reverse gear command rejection: commanded vs. actual from
  `/hakuroukun/gear_state` (thesis Run 2: 89.7%).
- GPS quality: position covariance min / median / max, fix count, fix-status
  histogram, max gap between fixes.
- Restricted-cell hits: trajectory samples falling in occupied map cells
  (thesis Run 1: 5.02%, Run 2: 5.96%, remember these represent map/reality
  misalignment on the GPS-derived map, not physical collisions).
- 6-panel diagnostic plot (`<stem>_plot.png`).

**Usage:**
```bash
# single bag
python3 scripts/evaluation/evaluate_real_run.py /path/to/run.bag \
    --output-dir results/ --cleaning-width 1.0

# batch (glob or multiple explicit paths)
python3 scripts/evaluation/evaluate_real_run.py \
    --batch bags/real_run_*.bag --output-dir results/
# → per-bag summary.txt + metrics.json + plot.png, plus a
#   comparison_YYYYMMDD_HHMMSS.csv sorted by coverage%
```

Required packages (in the container): `numpy pandas matplotlib rosbags`.
Install with `pip install --user rosbags pandas matplotlib numpy` if
missing.

Topics the bag must contain (all standard Hakuroukun stack):
`/fix`, `/hakuroukun_pose/rear_wheel_odometry`, `/cmd_controller`,
`/hakuroukun/gear_state`, `/tf`, `/planned_path` (or `/desired_path`), `/map`.
See the T2 rosbag note above, use `-a` on real runs to guarantee all
of these get recorded.

---

### Simulation results (thesis)

Two simulation runs are reported in the thesis, both on the D-F conemap
environment (`conemap_planning.yaml`, 444.29 m² raw free area, 334.44 m²
BCD-inflated valid area). Run 1 is the obstacle-free baseline; Run 2
injects a cylindrical obstacle to exercise the two-layer safety architecture.

| Metric | Run 1 (no obstacle) | Run 2 (with obstacle) |
|---|---|---|
| Coverage (raw denom) | 60.00% | 54.41% |
| Cleaned area | 266.56 m² | 241.73 m² |
| Duration | 22.89 min | 23.88 min |
| Path length | 330.27 m | 307.84 m |
| Savg | 0.25 m | 0.40 m |
| Smax | 0.92 m | 3.57 m |
| RMS | 0.32 m | 0.75 m |
| Planned pts > 1 m off | 0 | 167 (8.3%) |
| Stall count / total | 11 / 130.5 s (9.5%) | 13 / 182.3 s (12.7%) |
| Restricted-cell hits | 0 | 0 |

**Baseline (Tai TASP, 2024):** 52.80% sim coverage on a different map
configuration (718.18 m² valid area). Because the two evaluations use
different map extents and cone layouts, a direct coverage % comparison
is not controlled. What can be compared like-for-like is trajectory
tracking under equivalent methodology: Run 1's Savg 0.25 m and Smax 0.92 m
are tighter than Tai's Savg 0.31 m / Smax 1.19 m, indicating cleaner lane
following.

**Notes on Run 2:**
- Zero restricted-cell hits confirms the pre-encoded cone polygons are
  respected throughout.
- The replanner fired once successfully (rerouted around the first
  cylindrical obstacle, rejoined the coverage lanes) and failed to
  trigger on a second obstacle encounter later in the run. The 167
  planned points > 1 m off (and the Smax = 3.57 m spike) are concentrated
  at the location of that second unresolved encounter, this is a
  software-level replanner issue under investigation, not a design flaw.
- The 4.5-point coverage drop between Run 1 and Run 2 (60.00 → 54.41%)
  is the cost of incomplete obstacle recovery, not the cost of running
  the two-layer safety architecture itself.

Development history (for context, not for reporting): several
earlier sim runs during development yielded 65% / 64% / 91% coverage
figures under a range of measurement conventions and mid-development
bug states. Three bugs were fixed on the path to the numbers above:
`obs_first`/`obs_grid` recenter lockstep, `ROS_IP=127.0.0.1` in
docker-compose, and `closest_i` seeding on new path arrival during HOLD.
The numbers reported in this section and the thesis are from the
corrected implementation with all three fixes applied, evaluated by
`evaluate_real_run.py` against the same denominator basis as the
real-world runs.

---

### Real-world results (thesis)

Two field runs on 2026-07-20 at the D-F outdoor field area (TUT campus), both
on the corner-derived `df_area_outline_rotated.yaml` map (the drawn outline
rotated 8.3° to axis-align with the field; 308.75 m² valid area).
Both runs were fully evaluated post-hoc via `evaluate_real_run.py` from the
same `bcd_automode.ino` autonomous run, no manual seeding beyond the
2D Pose Estimate click.

| Metric | Run 1 (run3.bag) | Run 2 (run4.bag) |
|---|---|---|
| Coverage | 58.57% | 55.94% |
| Cleaned area | 180.84 m² | 172.72 m² |
| Duration | 31.73 min | 30.54 min |
| Path length | 294.16 m | 281.10 m |
| Savg | 0.38 m | 0.38 m |
| Smax | 1.35 m | 1.30 m |
| RMS | 0.47 m | 0.46 m |
| Planned pts > 1 m off | 43 (3.9%) | 28 (2.6%) |
| Stall count / total | 23 / 355.1 s (18.7%) | 23 / 276.6 s (15.1%) |
| Reverse cmds rejected | not recorded | 366 / 408 (89.7%) |
| GPS cov median / max | not recorded | 1.69e-4 / 4.41e-4 m² |
| Restricted-cell hits | 950 (5.02%) | 1059 (5.96%) |

Run 1 was recorded without gear-state and GPS topics in the bag, so
reverse-rejection rate and GPS quality metrics are unavailable for that
run. Both runs used the same autonomous stack; the similar stall counts
(23 each) suggest the reverse-rejection fault was active in Run 1 as well.

**Baseline (Tai TASP, 2024, same physical field):** 38.07% coverage on
411.46 m² valid area, duration 11 min 46 s. Different map (larger valid
area, different cone layout), so this is a context comparison, not a
controlled one.

Key qualitative findings (from thesis §5.4):

- No physical collisions in either run. The robot did not strike any
  wall, cone, or pole. Both runs terminated by stopping in a safe area.
- Restricted-cell hits reflect map inaccuracy, not real violations.
  The GPS-corner-derived map approximates the field but does not match
  the actual geometry with full accuracy, the robot occasionally
  traversed open ground that the map labels as restricted. Physical
  restricted zones were respected.
- Layer 2 (Persistence-Gated A\* replanner) did not trigger in either
  field run. No sustained-obstacle scenarios arose that met the 7 s
  persistence threshold during the runs.
- Layer 1 (Reflex HOLD) fired reliably against walls (D-building
  staircase and adjacent garden patch in Run 2, visible as a looping
  recovery trajectory in the RViz sequence), but showed inconsistent
  detection against lower-profile objects like cones and persons in
  close proximity. Root cause not conclusively identified in the field;
  candidates include sensor detection-range thresholds not being met
  for low-profile obstacles.
- Reverse-gear command rejection is the dominant hardware limitation.
  89.7% of commanded reverse events in Run 2 were not physically executed.
  This directly caps how many recovery maneuvers can complete, and is
  the single biggest lever for improving real-world coverage.
- Heading drift over long runs. The TSND151 magnetometer is disabled
  (chassis magnetic interference), so map-frame yaw is derived entirely
  from gyro integration seeded at startup, no absolute heading correction
  during the run. Over ~30 min with frequent recovery-maneuver direction
  changes, this compounds into visible lateral offset from planned sweep
  lanes in the later phase of Run 2.
- Both field runs exceeded the TASP real-world baseline (38.07%)
  within the same physical environment despite the reverse-rejection fault.
  Different maps and cone layouts prevent this from being a controlled
  comparison, but the direction is consistent: offline BCD produces more
  complete traversal than the reactive TASP approach even under hardware
  constraints.

---

### Key references

- Choset & Pignon, 1998. Boustrophedon Cell Decomposition.
- Galceran & Carreras, 2013. CPP survey.
- Nguyen Van Tai, 2024, previous TASP work in this lab (baselines:
  52.80% sim, 38.07% real; Savg 0.31 m, Smax 1.19 m sim).
- Schmid et al., 2023. Dynablox (IEEE RA-L, DOI: 10.1109/LRA.2023.3305239), supports "persistence-gated" obstacle terminology.
- Kondo et al., 2026. SANDO (arXiv:2604.07599), supports "persistence-gated" replanning concept.

### Notes on conventions

- Coverage % is computed with `cleaning_width = 1.0 m` to stay comparable to
  Tai's results.
- Unknown cells (`-1`) in the OccupancyGrid are not counted as free in
  the coverage denominator. The reported coverage denominator is the
  EDT-inflated free space where BCD generates lanes, published by
  `offline_coverage_planner.py` on the latched `/bcd_valid_area_m2`
  (`std_msgs/Float32`) topic. `cleaning_simulator.py` (live) and
  `evaluate_real_run.py` (post-hoc) both consume this topic, so live
  and reported numbers use the same basis.
- `path_follower_config.yaml` ships with `lookahead_distance: 0.8` and
  `max_search_ahead: 25`. Larger values caused oscillation at U-turns.
- `path_follower_config.yaml` has `obstacle_stop_range: 0.90 m` (HOLD trigger)
  and `front_clear_range: 1.0 m` (HOLD→FORWARD clear threshold). The 0.10 m
  hysteresis between these values prevents mode chatter. See also the
  `front_stop_range` (1.50 m) note under "Real robot → 1. Set up
  experiment environment", a separate, related reverse-recovery
  threshold that must be set deliberately depending on whether you're
  testing obstacle avoidance that day.
- `path_follower_config.yaml` does not set `done_dwell_time`, the code
  default of 2.0 s is used. The robot must hold the final coverage pose for
  2.0 s before `/path_follower/done` fires, filtering out end-of-path wiggle.
- `local_replanner_config.yaml` has `obstacle_inflate_m: 1.2` for dynamic
  obstacles. This must stay larger than the robot's physical half-width
  (~0.81 m); a smaller value (has been observed drifting to 0.7 m during
  tuning) lets A\* plan detour arcs that physically clip obstacles.
  There is an outdated comment inside the YAML claiming the value was
  lowered to 0.9 m, the comment is stale; the actual live value is 1.2 m.
  If you re-tune, keep both the value and the comment consistent.
- `MAX_ACCEL`, `MAX_STEERING`, `MIN_STEERING` in `path_follower.py` are loaded
  from `/hakuroukun_steering_controller/` namespace, which is not set by
  any real-robot launch file. Code defaults (2.5, ±0.78 rad) are used silently
, then a defensive clamp in `__init__` further tightens `MAX_STEERING`
  to ±0.75 rad to stay clear of the Arduino's rejection boundary (see
  the steering calibration section above).
- `~skip_init_gate` (path_follower parameter, default `False`): when `True`,
  bypasses the init handshake. Set `True` in sim launch files (AMCL owns
  the map frame); leave unset / `False` for real-robot launches so the
  2D Pose Estimate click is required before motion. **Note:**
  `offline_coverage_planner.py` has its own separate instance of this same
  gate/param name, check both if you see unexpected gate-bypass behavior.
  Both are vulnerable to stale-rosparam carryover on a long-lived roscore
  session (see "Stale roscore state" note near the top of this README).
- `hakuroukun_pose.py` initializes `self.yaw = 0.0` in odom frame. The
  actual map-frame heading is set by the 2D Pose Estimate click via
  `map_odom_calibrator.py`. Do not re-introduce the 2026-07-05 workaround
  of hardcoding `self.yaw = math.pi`, the calibrator now owns initial
  heading via the map→odom TF.
- `conemap_planning.yaml` uses an absolute image path
  (`/root/catkin_ws/src/...`). This only works inside the container.
- `~gps_to_rear_axis`, `~imu_offset`, etc. as launch-file params on the
  pose node are dead for most fields, the node hardcodes
  `gps_to_rear_axis = 0.6` inline in `_gps_callback` rather than reading
  the param. `~rotation_angle` is likewise dead (see the GPS rotation
  angle section above, it's now sourced from a calibration file instead,
  not a param either). Do not assume changing launch-file params for this
  node has any effect without checking the source first.
- `tsnd151_imu_node` publishes on `/imu` (confirmed from source). This matches
  `hakuroukun_pose.py`'s subscriber topic, no remap needed. It publishes
  `angular_velocity.z` in deg/s, not rad/s, and does not populate
  `orientation`, both `hakuroukun_pose.py` and `gps_rotation_calibrator.py`
  account for this explicitly; keep them in sync if the driver ever changes.
- `sensor_bringup.launch` has no `/fix` → `/gps/fix` remap.
  `bringup_hakuroukun_robot.launch` starts `hakuroukun_pose.py` as a `<node>`
  directly, the old workaround of running it via `rosrun` is no longer needed.
- `hakuroukun_localization/hakuroukun_pose/src/hakuroukun_pose_node.py` is a
  dead duplicate, never imported, never launched by any current launch
  file. Do not edit it expecting changes to take effect; edit
  `hakuroukun_pose/hakuroukun_pose/hakuroukun_pose.py` instead.
- `gps_rotation_calibration_sim.yaml` (in this package's `config/`) sets
  both values to 0.0 for simulation, sim uses `/ground_truth/odometry`
  and does not need a GPS rotation correction. The real-world calibration
  file lives separately at `hakuroukun_pose/config/gps_rotation_calibration.yaml`
  and is what `hakuroukun_pose.py` loads at startup.