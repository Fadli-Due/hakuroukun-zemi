# D-F Real Area Mapping — Session Procedure

## What you're doing
Building a fresh occupancy grid of the D-F outdoor field with the real robot,
using the same GPS+IMU+wheel-fused pose that navigation uses. This replaces
reusing `conemap_planning.yaml` — the new map will reflect actual terrain and
obstacle geometry, not painted-in polygons.

## Prerequisites
- Arduino has **`keyboardmode.ino`** flashed (NOT `bcd_automode.ino`).
- Files installed inside the container:
  - `hakuroukun_boustrophedon_with_cones/launch/df_mapping_real.launch`
  - `hakuroukun_boustrophedon_with_cones/config/gmapping_params_real.yaml`
- `catkin_make` after copying in the launch file.
- Robot at D-F, sky visible to GPS antenna, laptop battery OK for ~30 min drive.

## Deploy the two files
Assuming you're editing on the host and syncing to the container the usual way:

```bash
# from host
docker cp df_mapping_real.launch \
    hakuroukun-robot:/root/catkin_ws/src/hakuroukun_boustrophedon_with_cones/launch/
docker cp gmapping_params_real.yaml \
    hakuroukun-robot:/root/catkin_ws/src/hakuroukun_boustrophedon_with_cones/config/

# inside container
docker exec -it hakuroukun-robot bash
cd /root/catkin_ws && catkin_make && source devel/setup.bash
```

(Reminder from your notes: `docker cp` resets exec permissions — the launch and
YAML don't need +x, but any Python nodes you copy do.)

## Terminal layout (5 terminals, each in the container)

### T1 — Robot bringup
```bash
roslaunch hakuroukun_boustrophedon_with_cones bringup_hakuroukun_robot.launch
```
Wait for both:
- `[hakuroukun_pose] Loaded GPS rotation angle -176.20 deg from ...`
- `Gyro Z bias calibrated: X.XXXX deg/s`

Keep robot **completely still** for ~10 s before doing anything else.
Then confirm `/fix status: 2` (RTK) — check with `rostopic echo -n1 /fix`.

### T2 — Rosbag recording (start BEFORE any motion)
```bash
mkdir -p /root/catkin_ws/bags && cd /root/catkin_ws/bags
rosbag record -a -O df_map_$(date +%Y%m%d_%H%M%S).bag __name:=map_recorder
```
Recording the whole session lets you re-map offline from this bag if the live
map turns out badly — no need for a second field trip.

### T3 — Keyboard teleop
(Whatever teleop node you use with `keyboardmode.ino`. If you drive by publishing
`/cmd_controller` directly, that also works. Just don't move yet.)

### T4 — Mapping
```bash
roslaunch hakuroukun_boustrophedon_with_cones df_mapping_real.launch
```
RViz should open with Fixed Frame = `map`, showing the robot at origin, laser
scan points around it, and an empty (unknown/gray) map. Within a few seconds
you should see occupied (black) and free (white) cells appear where the scan
covers.

**Sanity check before driving:** the map → odom → base_link TF chain should be
intact. `rosrun rqt_tf_tree rqt_tf_tree` in a spare terminal to confirm.

### T5 — (optional) Live monitor
```bash
rostopic hz /map
rostopic hz /scan_multi
```
Both should tick steadily. `/map` at ~0.5 Hz (2 s update interval), `/scan_multi`
at whatever your merger runs (~12 Hz based on today's bags).

## Driving pattern
- Start pointed along a natural axis of the area.
- Speed: **0.2–0.3 m/s** — slower is better for a clean map. Avoid the 0.4+ m/s
  you'd use for a coverage run.
- Overlap sweeps by ~50%. Loop closures help even with RTK-anchored odom.
- Cover the full extent of the intended coverage area **plus a 1–2 m margin** on
  all sides so the planner's inflated costmap has real (not unknown) data at
  the edges of the workable region.
- Drive slowly past each cone and stationary obstacle from at least two angles
  so the LiDAR sees them cleanly.
- If RViz shows the map going crazy (walls moving, sudden rotations), stop and
  check `/fix cov_xx` — a GPS RTK dropout is corrupting the odom.

## Save the map
When you're satisfied with the RViz view:
```bash
cd /root/catkin_ws/bags   # or wherever you want to save
rosrun map_server map_saver -f df_real_$(date +%Y%m%d_%H%M%S) map:=/map
```
This produces `df_real_<timestamp>.pgm` and `df_real_<timestamp>.yaml`.

## Shutdown order
```bash
rosnode kill /map_recorder    # T2 rosbag first, so the bag flushes cleanly
# then Ctrl+C in reverse: T5 -> T4 -> T3 -> T2 -> T1
```

## Verify the map
```bash
# check bag integrity
rosbag info df_map_YYYYMMDD_HHMMSS.bag

# view the saved map
eog df_real_YYYYMMDD_HHMMSS.pgm    # or any image viewer
cat df_real_YYYYMMDD_HHMMSS.yaml   # sanity check the origin
```

Look for:
- Sharp, straight obstacle edges (walls, buildings) — blurry/duplicated means
  odom drift during the run
- No large unknown patches inside your intended coverage area
- Origin coords in the .yaml that match a reasonable spot near the robot start
  position

## Next session (using this map for navigation)
1. Copy `df_real_<timestamp>.pgm` and `.yaml` into
   `hakuroukun_boustrophedon_with_cones/maps/` (or wherever your existing
   conemap lives).
2. Point `offline_path_planning_real.launch` at the new map YAML.
3. On startup, click **2D Pose Estimate** in RViz to seed the robot's location
   in the saved map (same `map_odom_calibrator.py` workflow you already use).
4. Run the BCD planner as usual.

## If gmapping struggles
Symptoms and quick fixes:

| Symptom | Try |
|---|---|
| Map walls doubling / ghosting | Slow down; loop-close by revisiting start; check GPS cov during that segment |
| Sudden map rotation | GPS RTK dropout — check `/fix` cov_xx spike, wait for solid green LED before continuing |
| Sparse obstacle edges | Raise `particles` from 30 → 60 in `gmapping_params_real.yaml` and relaunch |
| Cells around robot never turn "free" | LiDAR frame wrong. Verify `base_link -> laser_link` static TF matches the physical LiDAR mount, and that `/scan_multi` frame_id is `laser_link` (or update the launch to match) |
| Whole map keeps shifting each map update | Motion-model noise too low. Raise `srr/srt/str/stt` from 0.05 → 0.10 |

## Fallback: post-process from the rosbag
If the live map is bad, you can re-run gmapping offline from the T2 rosbag:
```bash
rosparam set /use_sim_time true
roscore &
rosbag play --clock df_map_<timestamp>.bag &
roslaunch hakuroukun_boustrophedon_with_cones df_mapping_real.launch open_rviz:=true
# once done: map_saver
```
This lets you tune gmapping params without a second field session.