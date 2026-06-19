# Local Replanner — Simulation Test Procedure

Three scenarios to verify the online obstacle modification layer works as
Sensei requested. Run all three in the conemap sim before the real robot.

## Setup (once)

In the container, after `catkin_make`:

```bash
chmod +x ~/catkin_ws/src/hakuroukun_boustrophedon_with_cones/scripts/control/local_replanner.py
chmod +x ~/catkin_ws/src/hakuroukun_boustrophedon_with_cones/scripts/tools/test_obstacle_spawner.py
```

In your RViz config, **add two displays** so you can see what the replanner sees:

- **Map** display, topic `/local_replanner/obstacle_grid`, color scheme "map".
  Persistent obstacles will show up as black cells over the static map.
- **MarkerArray** display, topic `/local_replanner/markers`.
  Blue line = baseline BCD path; orange line = active detour.

The original `/desired_path` (Path display) will be the live path, which equals
baseline when no detour is active and equals the spliced path when one is.

---

## Scenario A — Static obstacle (the main case)

**What we're testing:** the robot is following the BCD path, encounters an
unexpected static obstacle, and the replanner carves a detour around it.

### Steps

**T1** — launch the sim:
```bash
docker exec -it hakuroukun-robot bash
roslaunch hakuroukun_boustrophedon_with_cones bringup_hakuroukun_conemap_sim.launch
```

**T2** — launch the planning stack:
```bash
docker exec -it hakuroukun-robot bash
roslaunch hakuroukun_boustrophedon_with_cones offline_path_planning_conemap.launch
```

Wait until you see in T2's log:
- `[BCD] published /planned_path with N points`
- `[local_replanner] baseline path received: N points`

The robot should start moving along the blue baseline.

**T3** — pick a point ~10–15m ahead of the robot's current position along the
baseline (use RViz; click on the blue line, read the coordinates). Drop a box:
```bash
docker exec -it hakuroukun-robot bash
rosrun hakuroukun_boustrophedon_with_cones test_obstacle_spawner.py \
    static --x 5.0 --y 2.0
```

Replace `5.0`, `2.0` with the coordinates you picked.

**T4** — rosbag for evidence:
```bash
docker exec -it hakuroukun-robot bash
rosbag record -a -O scenario_A_static.bag \
    /desired_path /planned_path /local_replanner/obstacle_grid \
    /local_replanner/markers /scan_multi \
    /hakuroukun_pose/rear_wheel_odometry __name:=scenario_A_recorder
```

### Expected behavior

1. **t = 0**: robot follows blue baseline normally.
2. **Robot approaches box**: as soon as `min_front < 0.45m`, the **reflex stop**
   in `path_follower.py` fires. Robot decelerates. Log: `FORWARD VETO`.
3. **t ≈ +7s after first sighting**: the cells under the box exceed the
   persistence threshold. In the obstacle_grid display, **black cells appear
   on the box**.
4. **Next eval cycle (within 0.5s)**: log shows
   `[local_replanner] blockage on path [...]. Computing detour.` then
   `[local_replanner] DETOUR applied: ...`.
5. **Orange detour line appears in RViz**, going around the box.
6. **Robot resumes**, follows the orange detour, rejoins the blue baseline
   past the obstacle.

### Pass criteria

- The reflex stop fires before contact (no Gazebo collision warnings).
- A detour is computed and applied within ~8s of the obstacle becoming visible.
- The robot reaches the rejoin point without further intervention.
- Coverage of the rest of the map proceeds normally afterwards.

### Cleanup

```bash
rosnode kill /scenario_A_recorder
rosrun hakuroukun_boustrophedon_with_cones test_obstacle_spawner.py \
    delete --name test_box_0
```

---

## Scenario B — Pedestrian who passes through (the kid case)

**What we're testing:** a person (or animal, child) briefly crosses the path
but moves on. The robot must STOP (reflex), wait, and resume the original path
**without** carving a detour around empty space.

### Steps

Same T1, T2 as above. In T3, spawn a pedestrian that dwells for **3 seconds**
(less than the 7s persistence threshold) and then walks off:

```bash
rosrun hakuroukun_boustrophedon_with_cones test_obstacle_spawner.py \
    pedestrian --x 5.0 --y 2.0 --dwell 3.0 --exit-x 20.0 --exit-y 20.0
```

### Expected behavior

1. Robot approaches pedestrian.
2. **Reflex stop** fires (`FORWARD VETO`) — robot stops well before contact.
3. Pedestrian stands for 3s. Obstacle_grid will show **decay-time stamps** but
   NOT persistent cells (because 3 < 7).
4. Pedestrian walks away. After `clear_time` (1s) of no further sightings,
   the stamps fade.
5. `min_front` recovers, the reflex stop releases.
6. **No detour log line appears.** The live path stays equal to baseline.
7. Robot resumes along the original blue baseline.

### Pass criteria

- Robot stops before contact.
- No `[local_replanner] DETOUR applied` log line during the encounter.
- No orange detour marker appears.
- After pedestrian leaves, the robot resumes the **original** baseline path.

---

## Scenario C — Pedestrian who stays too long

**What we're testing:** the persistence logic eventually triggers a detour
even on a "soft" obstacle, if it really doesn't move. This confirms the
mechanism is not just relying on object class but on persistence.

### Steps

Same as B, but with dwell = 15s:

```bash
rosrun hakuroukun_boustrophedon_with_cones test_obstacle_spawner.py \
    pedestrian --x 5.0 --y 2.0 --dwell 15.0 --exit-x 20.0 --exit-y 20.0
```

### Expected behavior

1. Robot stops on reflex (same as B).
2. After ~7s of dwelling, persistent cells appear in obstacle_grid.
3. Detour is computed and the robot starts driving **around** the pedestrian.
4. Pedestrian (still in dwell phase or now moving away) — replanner has
   committed to the detour; even when the pedestrian leaves, the live path
   stays on the detour until the rejoin point. After rejoining, baseline
   resumes.

### Pass criteria

- Detour does eventually fire on a persistent "soft" obstacle.
- Robot reaches rejoin point without re-stopping spuriously.

### Note

For the real demo and thesis writeup, scenario B is the safety story
(reflex stop protects humans, no inappropriate detour). Scenario A is the
primary contribution (online modification of an offline path). Scenario C
demonstrates the persistence mechanism is principled, not hardcoded by
object type.

---

## Quick reference — key logs to grep

- `FORWARD VETO` — Layer 1 (reflex) firing. Good.
- `MODE → REVERSE` — Layer 1 backing up. Indicates robot got too close.
- `DETOUR applied` — Layer 2 carved a detour. Good (scenarios A, C).
- `obstacle cleared — baseline restored` — detour retired, back on baseline.
- `A* detour failed` — replanner couldn't find a way around. Investigate.

## Troubleshooting

**No baseline path appears.** `offline_coverage_planner` didn't publish. Check
T2 for `[BCD] published /planned_path`. If missing, check the map loaded
(see `/map` in RViz).

**Persistent cells never appear (Scenario A).** The LiDAR isn't seeing the box.
Check that the box is in the LiDAR's height plane and within `scan_max_range`.
Verify `/scan_multi` shows hits on the box in RViz.

**Detour appears but robot ignores it.** `path_follower` may still be tracking
the old path. Confirm the live `/desired_path` matches the orange line.

**Detour appears but is poor (e.g., cuts close to walls).** The obstacle
inflation is `robot_radius` from `boustrophedon_config.yaml`. Increase it
(current default 1.0m) for more clearance.
