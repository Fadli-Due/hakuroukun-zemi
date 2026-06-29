#!/usr/bin/env python3
# =============================================================================
#  evaluate_real_run.py
#
#  Post-hoc evaluation script for a real-robot experiment rosbag.
#
#  Usage:
#      python3 evaluate_real_run.py <path_to_bag> [options]
#
#  Options:
#      --map       Path to conemap_planning.pgm  (default: auto-find via ROS)
#      --map-yaml  Path to conemap_planning.yaml (default: alongside --map)
#      --cone-radius  Radius around cone zone centre treated as restricted (m)
#                     default: 1.5 (matches robot_radius + cone buffer)
#      --out-dir   Where to save result images  (default: same dir as bag)
#      --plot      Show matplotlib plots interactively (default: save only)
#
#  Outputs:
#      <bag_stem>_coverage.png       — cleaned area overlaid on map
#      <bag_stem>_trajectory.png     — actual vs planned path
#      <bag_stem>_results.txt        — all numeric metrics (copy into thesis)
#
#  Metrics produced:
#      Coverage %         — cleaned cells / total valid cells × 100
#      Savg               — mean cross-track error (nearest-neighbour, Tai method)
#      Smax               — maximum cross-track error
#      RMSE               — RMS cross-track error
#      Zone violations    — number of robot poses inside any restricted zone
#      Detour count       — number of A* detours fired (from /local_replanner log)
#
#  Tai's nearest-neighbour method (from thesis §3.4):
#      For each odometry pose p_i, find the nearest point q_j on the planned
#      path. The cross-track error e_i = ||p_i - q_j||. This matches Tai's
#      implementation so results are directly comparable.
# =============================================================================
import argparse
import os
import sys
import math
import numpy as np

try:
    import rosbag
except ImportError:
    sys.exit("ERROR: rosbag not importable. Run this script inside the Docker "
             "container:\n  docker exec -it hakuroukun-robot bash\n"
             "  python3 /root/catkin_ws/src/hakuroukun_boustrophedon_with_cones"
             "/scripts/evaluation/evaluate_real_run.py <bag>")

from PIL import Image

# ---- ROS message imports (only needed for type hints / field access) --------
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped


# =============================================================================
#  Helpers
# =============================================================================

def load_map(pgm_path, yaml_path):
    """Load a ROS map_server map. Returns (image_array, origin_x, origin_y, resolution)."""
    import yaml
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    res = meta["resolution"]
    origin = meta["origin"]          # [x, y, theta]
    ox, oy = origin[0], origin[1]
    negate = meta.get("negate", 0)
    occ_thresh = meta.get("occupied_thresh", 0.65)
    free_thresh = meta.get("free_thresh", 0.196)

    img = np.array(Image.open(pgm_path))
    # ROS convention: pixel value 0 = black = occupied, 254 = white = free
    if negate:
        occ_prob = img / 255.0
    else:
        occ_prob = 1.0 - img / 255.0

    free  = occ_prob <= free_thresh
    occ   = occ_prob >= occ_thresh
    # unknown = ~free & ~occ  (grey pixels)
    return free, occ, ox, oy, res


def world_to_pixel(x, y, ox, oy, res, img_height):
    """Convert map-frame (x,y) to pixel (col, row). Row 0 is bottom in ROS maps."""
    col = int((x - ox) / res)
    row = img_height - 1 - int((y - oy) / res)
    return col, row


def pixel_to_world(col, row, ox, oy, res, img_height):
    x = col * res + ox
    y = (img_height - 1 - row) * res + oy
    return x, y


def extract_odom(bag, topic="/hakuroukun_pose/rear_wheel_odometry"):
    poses = []
    for _, msg, _ in bag.read_messages(topics=[topic]):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        poses.append((x, y))
    return poses


def extract_planned_path(bag, topic="/desired_path"):
    """Take the first /desired_path message as the planned path (latched)."""
    for _, msg, _ in bag.read_messages(topics=[topic]):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if pts:
            return pts
    return []


def extract_detour_count(bag):
    """Count DETOUR applied log messages from /rosout."""
    count = 0
    for _, msg, _ in bag.read_messages(topics=["/rosout", "/rosout_agg"]):
        if hasattr(msg, "msg") and "DETOUR applied" in msg.msg:
            count += 1
    return count


def compute_coverage(poses, free_mask, ox, oy, res, cleaning_radius_m=0.5):
    """
    Paint the robot's footprint (circle of cleaning_radius_m) onto the free mask
    and count covered pixels.

    Returns (coverage_pct, cleaned_mask).
    """
    h, w = free_mask.shape
    cleaned = np.zeros((h, w), dtype=bool)
    r_px = int(math.ceil(cleaning_radius_m / res))

    for (x, y) in poses:
        col, row = world_to_pixel(x, y, ox, oy, res, h)
        for dr in range(-r_px, r_px + 1):
            for dc in range(-r_px, r_px + 1):
                if dr*dr + dc*dc <= r_px*r_px:
                    rr, cc = row + dr, col + dc
                    if 0 <= rr < h and 0 <= cc < w and free_mask[rr, cc]:
                        cleaned[rr, cc] = True

    total_valid = int(np.sum(free_mask))
    total_cleaned = int(np.sum(cleaned))
    pct = 100.0 * total_cleaned / total_valid if total_valid > 0 else 0.0
    return pct, cleaned


def compute_trajectory_error(poses, planned_path):
    """
    Tai's nearest-neighbour cross-track error method.

    For each robot pose, find the nearest point on the planned path.
    Returns (Savg, Smax, RMSE) in metres.
    """
    if not poses or not planned_path:
        return float("nan"), float("nan"), float("nan")

    path_arr = np.array(planned_path)      # (N, 2)
    pose_arr = np.array(poses)             # (M, 2)

    errors = []
    for p in pose_arr:
        dists = np.linalg.norm(path_arr - p, axis=1)
        errors.append(float(np.min(dists)))

    errors = np.array(errors)
    return float(np.mean(errors)), float(np.max(errors)), float(np.sqrt(np.mean(errors**2)))


def check_zone_violations(poses, cone_centres, cone_radius_m):
    """
    Count how many robot poses fall inside any restricted cone zone.

    cone_centres: list of (x, y) in map frame
    Returns violation_count (int).
    """
    if not cone_centres:
        return 0
    violations = 0
    for (px, py) in poses:
        for (cx, cy) in cone_centres:
            if math.hypot(px - cx, py - cy) < cone_radius_m:
                violations += 1
                break   # count once per pose even if inside multiple zones
    return violations


# =============================================================================
#  Cone zone centres for the D-F area (from conemap_planning static map)
#  These match the painted restricted zones in conemap_planning.pgm.
#  Update if cones are moved.
# =============================================================================
DF_CONE_CENTRES = [
    # (map-frame x, map-frame y)   — read from conemap_planning.yaml origin
    # and measured cone pixel positions.  Placeholder — fill in from RViz
    # by clicking on each cone centre and reading the coordinates.
    # Example: (-12.0, -2.5), (-10.5, -4.0)
    # Leave empty to skip zone-violation check.
]


# =============================================================================
#  Plotting
# =============================================================================

def save_coverage_plot(free_mask, occ_mask, cleaned_mask, poses, out_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    h, w = free_mask.shape
    rgb = np.ones((h, w, 3), dtype=np.uint8) * 200   # grey = unknown
    rgb[free_mask]  = [255, 255, 255]                  # white = free
    rgb[occ_mask]   = [0,   0,   0  ]                  # black = wall
    rgb[cleaned_mask] = [144, 238, 144]                 # light green = cleaned

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(rgb, origin="upper")
    ax.set_title("Cleaned area (green) over map")
    ax.axis("off")
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close()
    print(f"  Saved: {out_path}")


def save_trajectory_plot(free_mask, occ_mask, poses, planned_path,
                         ox, oy, res, out_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    h, w = free_mask.shape
    rgb = np.ones((h, w, 3), dtype=np.uint8) * 200
    rgb[free_mask] = [255, 255, 255]
    rgb[occ_mask]  = [0,   0,   0  ]

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(rgb, origin="upper")

    def to_px_list(pts):
        cols = [(world_to_pixel(x, y, ox, oy, res, h)[0]) for x, y in pts]
        rows = [(world_to_pixel(x, y, ox, oy, res, h)[1]) for x, y in pts]
        return cols, rows

    if planned_path:
        pc, pr = to_px_list(planned_path)
        ax.plot(pc, pr, "b-", linewidth=0.8, alpha=0.6, label="Planned path")

    if poses:
        ac, ar = to_px_list(poses)
        ax.plot(ac, ar, "r-", linewidth=1.0, alpha=0.8, label="Actual trajectory")
        ax.plot(ac[0],  ar[0],  "go", markersize=8, label="Start")
        ax.plot(ac[-1], ar[-1], "rs", markersize=8, label="End")

    ax.legend(loc="upper right", fontsize=8)
    ax.set_title("Planned (blue) vs Actual (red) trajectory")
    ax.axis("off")
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close()
    print(f"  Saved: {out_path}")


# =============================================================================
#  Main
# =============================================================================

def main():
    ap = argparse.ArgumentParser(description="Evaluate a Hakuroukun real-robot rosbag.")
    ap.add_argument("bag", help="Path to .bag file")
    ap.add_argument("--map", default=None,
                    help="Path to conemap_planning.pgm")
    ap.add_argument("--map-yaml", default=None,
                    help="Path to conemap_planning.yaml (default: <map>.yaml)")
    ap.add_argument("--cleaning-width", type=float, default=1.0,
                    help="Robot cleaning width in metres (default: 1.0)")
    ap.add_argument("--cone-radius", type=float, default=1.5,
                    help="Restricted zone radius around each cone centre (m)")
    ap.add_argument("--out-dir", default=None,
                    help="Output directory for plots/results (default: bag directory)")
    ap.add_argument("--plot", action="store_true",
                    help="Show plots interactively (requires display)")
    args = ap.parse_args()

    bag_path = os.path.abspath(args.bag)
    if not os.path.exists(bag_path):
        sys.exit(f"ERROR: bag not found: {bag_path}")

    stem = os.path.splitext(os.path.basename(bag_path))[0]
    out_dir = args.out_dir or os.path.dirname(bag_path)
    os.makedirs(out_dir, exist_ok=True)

    # ---- Locate map files --------------------------------------------------
    if args.map is None:
        # Try to find relative to this script's package
        script_dir = os.path.dirname(os.path.abspath(__file__))
        pkg_root = os.path.join(script_dir, "..", "..")
        default_pgm = os.path.join(pkg_root, "maps", "conemap_planning.pgm")
        args.map = os.path.normpath(default_pgm)

    if args.map_yaml is None:
        args.map_yaml = os.path.splitext(args.map)[0] + ".yaml"

    if not os.path.exists(args.map):
        sys.exit(f"ERROR: map PGM not found: {args.map}\n"
                 "Pass --map /path/to/conemap_planning.pgm")
    if not os.path.exists(args.map_yaml):
        sys.exit(f"ERROR: map YAML not found: {args.map_yaml}")

    print(f"\n{'='*60}")
    print(f"  Bag:      {bag_path}")
    print(f"  Map:      {args.map}")
    print(f"{'='*60}\n")

    # ---- Load map ----------------------------------------------------------
    print("Loading map...")
    free_mask, occ_mask, map_ox, map_oy, map_res = load_map(args.map, args.map_yaml)
    print(f"  Map size: {free_mask.shape[1]}w x {free_mask.shape[0]}h px, "
          f"res={map_res}m, origin=({map_ox:.2f},{map_oy:.2f})")
    print(f"  Valid (free) cells: {int(np.sum(free_mask))}")

    # ---- Open bag ----------------------------------------------------------
    print("\nReading bag...")
    bag = rosbag.Bag(bag_path, "r")

    poses         = extract_odom(bag)
    planned_path  = extract_planned_path(bag)
    detour_count  = extract_detour_count(bag)

    print(f"  Odometry poses:       {len(poses)}")
    print(f"  Planned path points:  {len(planned_path)}")
    print(f"  Detours fired:        {detour_count}")
    bag.close()

    if not poses:
        sys.exit("ERROR: No odometry data found in bag. "
                 "Check topic /hakuroukun_pose/rear_wheel_odometry")

    # ---- Metrics -----------------------------------------------------------
    print("\nComputing coverage...")
    cleaning_radius = args.cleaning_width / 2.0
    cov_pct, cleaned_mask = compute_coverage(
        poses, free_mask, map_ox, map_oy, map_res, cleaning_radius)

    print("Computing trajectory error...")
    savg, smax, rmse = compute_trajectory_error(poses, planned_path)

    print("Checking zone violations...")
    violations = check_zone_violations(poses, DF_CONE_CENTRES, args.cone_radius)

    # ---- Results -----------------------------------------------------------
    total_valid_m2 = int(np.sum(free_mask)) * map_res * map_res
    total_cleaned_m2 = int(np.sum(cleaned_mask)) * map_res * map_res

    results = f"""
{'='*60}
  EXPERIMENT EVALUATION RESULTS
  Bag: {os.path.basename(bag_path)}
{'='*60}

  COVERAGE
    Total valid area:      {total_valid_m2:.2f} m²
    Cleaned area:          {total_cleaned_m2:.2f} m²
    Coverage percentage:   {cov_pct:.2f} %
    (Tai baseline:         38.07 %)

  TRAJECTORY ERROR  (Tai nearest-neighbour method)
    Mean error  Savg:      {savg:.4f} m
    Max error   Smax:      {smax:.4f} m
    RMS error   RMSE:      {rmse:.4f} m

  SAFETY / COMPLIANCE
    Restricted zone violations: {violations}
    A* detours fired:           {detour_count}

  MISC
    Total poses recorded:  {len(poses)}
    Planned path points:   {len(planned_path)}
{'='*60}
"""
    print(results)

    txt_path = os.path.join(out_dir, f"{stem}_results.txt")
    with open(txt_path, "w") as f:
        f.write(results)
    print(f"  Saved: {txt_path}")

    # ---- Plots -------------------------------------------------------------
    print("\nGenerating plots...")
    cov_plot  = os.path.join(out_dir, f"{stem}_coverage.png")
    traj_plot = os.path.join(out_dir, f"{stem}_trajectory.png")

    save_coverage_plot(free_mask, occ_mask, cleaned_mask, poses, cov_plot)
    save_trajectory_plot(free_mask, occ_mask, poses, planned_path,
                         map_ox, map_oy, map_res, traj_plot)

    if args.plot:
        import matplotlib.pyplot as plt
        plt.show()

    print("\nDone.")


if __name__ == "__main__":
    main()