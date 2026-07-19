#!/usr/bin/env python3
"""evaluate_real_run.py

Post-hoc evaluation of Hakuroukun BCD coverage runs from rosbag(s).

Metrics computed (matching Tai's TASP methodology + extra diagnostics):
  - Coverage % (occupancy-grid raster of trajectory ⊗ cleaning_width footprint,
                against free cells in /map, excluding occupied/restricted)
  - Trajectory error Savg / Smax / RMS (nearest-neighbor from planned to actual)
  - Duration, path length, mean speed when moving
  - Stall count + total stall time (speed < 0.02 m/s for >= 3 s)
  - Reverse gear command rejection rate  (path_follower requested rev vs Arduino actual)
  - Overall gear commanded/actual mismatch %
  - GPS quality (min/median/max position covariance, dropout count)
  - Restricted-zone / obstacle-cell violations (trajectory samples in occupied cells)

Usage:
  # single bag
  python3 evaluate_real_run.py /path/to/run.bag [--output-dir results/] \
      [--cleaning-width 1.0] [--stall-threshold 0.02] [--stall-min-duration 3.0] \
      [--no-plots]

  # batch (glob or multiple explicit paths)
  python3 evaluate_real_run.py --batch bags/real_run_*.bag [--output-dir results/]
  # -> writes per-bag files plus a combined comparison CSV

Outputs per bag:
  <stem>_metrics.json    machine-readable metrics
  <stem>_summary.txt     human-readable summary
  <stem>_plot.png        6-panel diagnostic figure (unless --no-plots)

Batch mode also writes:
  comparison_YYYYMMDD_HHMMSS.csv    one row per bag, sorted by coverage%

Requires: python3, numpy, pandas, matplotlib, rosbags
  pip install --user rosbags pandas matplotlib numpy

Topics expected (all from the standard Hakuroukun stack):
  /fix                                       sensor_msgs/NavSatFix
  /hakuroukun_pose/rear_wheel_odometry       nav_msgs/Odometry
  /cmd_controller                            std_msgs/Float64MultiArray
  /hakuroukun/gear_state                     std_msgs/Int8MultiArray  (data=[commanded, actual])
  /tf                                        tf2_msgs/TFMessage       (map->odom + odom->base_link)
  /planned_path or /desired_path             nav_msgs/Path            (frame=map)
  /map                                       nav_msgs/OccupancyGrid   (frame=map)
"""

from __future__ import annotations
import argparse
import glob
import json
import sys
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

try:
    from rosbags.highlevel import AnyReader
except ImportError:
    sys.stderr.write("ERROR: rosbags not installed. Run: pip install --user rosbags\n")
    sys.exit(1)


# ============================================================================
# Extraction
# ============================================================================

NEEDED_TOPICS = {
    '/fix', '/cmd_controller', '/hakuroukun/gear_state',
    '/tf', '/tf_static',
    '/planned_path', '/desired_path', '/map',
}


def extract_bag(bag_path: Path, odom_topic: str = '/hakuroukun_pose/rear_wheel_odometry') -> Dict[str, Any]:
    """Read a bag, return dict of DataFrames + path/map metadata.

    odom_topic: which nav_msgs/Odometry topic to treat as the robot's pose feed.
      Real-robot bags: '/hakuroukun_pose/rear_wheel_odometry' (default).
      Sim bags using ground-truth odometry: '/ground_truth/odometry'.
    Bags without /fix or /hakuroukun/gear_state (e.g. sim runs) are handled
    gracefully — those sections of the report are simply omitted (n_samples: 0).

    Only connections in NEEDED_TOPICS (+ odom_topic) are iterated. This matters
    a lot for bags recorded with `rosbag record -a` in Gazebo, which also
    capture huge high-rate topics (gazebo_msgs/LinkStates, ModelStates, camera
    images, diagnostics) that bloat the bag to 10-20x the size actually needed
    and are never used here — deserializing those is what makes an unfiltered
    read of a 17 GB / 4.6M-message bag take an extremely long time.
    """
    data = {
        'fix': [], 'odom': [], 'cmd': [], 'gear': [],
        'tf_map_odom': [], 'tf_odom_base': [],
        'planned_path': None, 'desired_path': None, 'map': None,
        't_start': None, 't_end': None,
    }
    wanted = NEEDED_TOPICS | {odom_topic}
    with AnyReader([bag_path]) as reader:
        data['t_start'] = reader.start_time / 1e9
        data['t_end'] = reader.end_time / 1e9

        conns = [c for c in reader.connections if c.topic in wanted]
        found_topics = {c.topic for c in conns}
        missing = wanted - found_topics
        if missing:
            sys.stderr.write(f"  note: topics not present in bag (OK if expected): {sorted(missing)}\n")
        print(f"  filtering to {len(conns)} connections on {sorted(found_topics)} "
              f"(skipping everything else in the bag)", flush=True)

        n_msg = 0
        for conn, ts, raw in reader.messages(connections=conns):
            n_msg += 1
            if n_msg % 50000 == 0:
                print(f"  ... {n_msg} messages processed", flush=True)
            t = ts / 1e9
            topic = conn.topic
            msg = reader.deserialize(raw, conn.msgtype)
            if topic == '/fix':
                data['fix'].append({
                    't': t, 'lat': msg.latitude, 'lon': msg.longitude, 'alt': msg.altitude,
                    'status': msg.status.status,
                    'cov_xx': msg.position_covariance[0],
                    'cov_yy': msg.position_covariance[4],
                    'cov_zz': msg.position_covariance[8],
                })
            elif topic == odom_topic:
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                data['odom'].append({
                    't': t, 'x': p.x, 'y': p.y,
                    'qz': q.z, 'qw': q.w,
                    'vx': msg.twist.twist.linear.x,
                    'wz': msg.twist.twist.angular.z,
                })
            elif topic == '/cmd_controller':
                arr = list(msg.data)
                row = {'t': t}
                for i, v in enumerate(arr):
                    row[f'c{i}'] = float(v)
                data['cmd'].append(row)
            elif topic == '/hakuroukun/gear_state':
                arr = list(msg.data)
                row = {'t': t}
                for i, v in enumerate(arr):
                    row[f'g{i}'] = int(v)
                data['gear'].append(row)
            elif topic == '/tf':
                for tr in msg.transforms:
                    parent = tr.header.frame_id
                    child = tr.child_frame_id
                    if parent == 'map' and child == 'odom':
                        data['tf_map_odom'].append({
                            't': t,
                            'x': tr.transform.translation.x,
                            'y': tr.transform.translation.y,
                            'qz': tr.transform.rotation.z,
                            'qw': tr.transform.rotation.w,
                        })
                    elif parent == 'odom' and child == 'base_link':
                        data['tf_odom_base'].append({
                            't': t,
                            'x': tr.transform.translation.x,
                            'y': tr.transform.translation.y,
                            'qz': tr.transform.rotation.z,
                            'qw': tr.transform.rotation.w,
                        })
            elif topic == '/planned_path' and data['planned_path'] is None:
                data['planned_path'] = {
                    'frame_id': msg.header.frame_id,
                    'points': [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses],
                }
            elif topic == '/desired_path' and data['desired_path'] is None:
                data['desired_path'] = {
                    'frame_id': msg.header.frame_id,
                    'points': [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses],
                }
            elif topic == '/map' and data['map'] is None:
                data['map'] = {
                    'frame_id': msg.header.frame_id,
                    'resolution': msg.info.resolution,
                    'width': msg.info.width,
                    'height': msg.info.height,
                    'origin_x': msg.info.origin.position.x,
                    'origin_y': msg.info.origin.position.y,
                    'data': np.array(msg.data, dtype=np.int8),
                }

    for k in ['fix', 'odom', 'cmd', 'gear', 'tf_map_odom', 'tf_odom_base']:
        data[k] = pd.DataFrame(data[k])
    return data


# ============================================================================
# Derived quantities
# ============================================================================

def yaw_from_quat(qz, qw):
    return 2.0 * np.arctan2(qz, qw)


def build_map_trajectory(bag: Dict[str, Any]) -> pd.DataFrame:
    """Chain map->odom and odom->base_link to get robot pose in map frame."""
    tf_mo = bag['tf_map_odom'].sort_values('t').reset_index(drop=True)
    tf_ob = bag['tf_odom_base'].sort_values('t').reset_index(drop=True)
    if len(tf_mo) == 0 or len(tf_ob) == 0:
        return pd.DataFrame(columns=['t', 'x', 'y', 'yaw'])

    t_ob = tf_ob['t'].values
    t_mo = tf_mo['t'].values
    # for each odom->base, take latest map->odom at or before its timestamp
    idx = np.searchsorted(t_mo, t_ob, side='right') - 1
    idx = np.clip(idx, 0, len(t_mo) - 1)

    mo_yaw = yaw_from_quat(tf_mo['qz'].values[idx], tf_mo['qw'].values[idx])
    mo_x = tf_mo['x'].values[idx]
    mo_y = tf_mo['y'].values[idx]
    ob_x = tf_ob['x'].values
    ob_y = tf_ob['y'].values
    ob_yaw = yaw_from_quat(tf_ob['qz'].values, tf_ob['qw'].values)

    c = np.cos(mo_yaw); s = np.sin(mo_yaw)
    return pd.DataFrame({
        't': t_ob,
        'x': mo_x + c * ob_x - s * ob_y,
        'y': mo_y + s * ob_x + c * ob_y,
        'yaw': mo_yaw + ob_yaw,
    })


def compute_speed(traj: pd.DataFrame, smooth_window: int = 5) -> np.ndarray:
    """Speed (m/s) from position derivatives, moving-average smoothed."""
    t = traj['t'].values
    dt = np.diff(t)
    dx = np.diff(traj['x'].values)
    dy = np.diff(traj['y'].values)
    v = np.sqrt(dx**2 + dy**2) / np.where(dt > 0, dt, np.nan)
    return pd.Series(v).rolling(smooth_window, center=True, min_periods=1).mean().values


def detect_stalls(traj: pd.DataFrame, speed: np.ndarray,
                  threshold: float, min_duration: float) -> List[Dict[str, float]]:
    """Return list of stall events {t_start_rel, t_end_rel, dur, x, y}."""
    t = traj['t'].values[1:]
    x = traj['x'].values[1:]
    y = traj['y'].values[1:]
    t0 = traj['t'].values[0]
    is_slow = speed < threshold
    stalls: List[Dict[str, float]] = []
    i = 0
    while i < len(is_slow):
        if is_slow[i]:
            j = i
            while j < len(is_slow) and is_slow[j]:
                j += 1
            dur = t[j - 1] - t[i]
            if dur >= min_duration:
                stalls.append({'t_start': t[i] - t0, 't_end': t[j - 1] - t0,
                               'dur': float(dur), 'x': float(x[i]), 'y': float(y[i])})
            i = j
        else:
            i += 1
    return stalls


def trajectory_error(planned_pts, actual_xy):
    """Nearest-neighbor error from each planned point to the actual trajectory.
    Returns Savg, Smax, RMS, per-point distances."""
    if len(actual_xy) == 0 or len(planned_pts) == 0:
        return float('nan'), float('nan'), float('nan'), np.array([])
    pl = np.asarray(planned_pts)
    ac = np.asarray(actual_xy)
    d = np.empty(len(pl))
    # chunked to avoid a full |pl|x|ac| matrix if either is huge
    chunk = 500
    for i in range(0, len(pl), chunk):
        seg = pl[i:i+chunk][:, None, :] - ac[None, :, :]
        d[i:i+chunk] = np.sqrt((seg**2).sum(-1)).min(axis=1)
    return float(d.mean()), float(d.max()), float(np.sqrt((d**2).mean())), d


def compute_coverage(traj_xy: np.ndarray, map_info: Dict[str, Any],
                     cleaning_width: float = 1.0) -> Dict[str, Any]:
    """Rasterize path with cleaning_width footprint against map's free cells."""
    res = map_info['resolution']
    W, H = map_info['width'], map_info['height']
    ox, oy = map_info['origin_x'], map_info['origin_y']
    grid = map_info['data'].reshape(H, W)

    valid_mask = (grid == 0)
    total_valid = int(valid_mask.sum())

    cleaned = np.zeros_like(grid, dtype=bool)
    r_cells = int(np.ceil((cleaning_width / 2.0) / res))
    dx = np.arange(-r_cells, r_cells + 1)
    DX, DY = np.meshgrid(dx, dx)
    circ = (DX * res)**2 + (DY * res)**2 <= (cleaning_width / 2.0)**2
    offx = DX[circ]; offy = DY[circ]

    for x, y in traj_xy:
        cx = int((x - ox) / res)
        cy = int((y - oy) / res)
        xs = cx + offx; ys = cy + offy
        m = (xs >= 0) & (xs < W) & (ys >= 0) & (ys < H)
        cleaned[ys[m], xs[m]] = True

    cleaned_valid = int((cleaned & valid_mask).sum())
    # obstacle-cell violations (trajectory samples that fell inside occupied cells)
    violations = 0
    for x, y in traj_xy:
        cx = int((x - ox) / res); cy = int((y - oy) / res)
        if 0 <= cx < W and 0 <= cy < H and grid[cy, cx] == 100:
            violations += 1
    return {
        'resolution': float(res),
        'total_valid_cells': total_valid,
        'total_valid_area_m2': float(total_valid * res * res),
        'cleaned_cells': cleaned_valid,
        'cleaned_area_m2': float(cleaned_valid * res * res),
        'coverage_pct': float(100.0 * cleaned_valid / total_valid) if total_valid else 0.0,
        'restricted_violations': violations,
        'restricted_violations_pct': float(100.0 * violations / len(traj_xy)) if len(traj_xy) else 0.0,
        'cleaned_mask': cleaned,
        'valid_mask': valid_mask,
        'grid': grid,
    }


def gear_stats(gear_df: pd.DataFrame) -> Dict[str, Any]:
    """Commanded vs actual mismatch, split by direction (rev cmds specifically)."""
    if len(gear_df) == 0 or 'g0' not in gear_df.columns or 'g1' not in gear_df.columns:
        return {'n_samples': 0}
    cmd = gear_df['g0'].values
    act = gear_df['g1'].values
    n_total = len(cmd)
    n_mis = int((cmd != act).sum())
    n_cmd_rev = int((cmd == 1).sum())
    n_act_rev = int((act == 1).sum())
    # rev rejection: commanded rev but actual still forward
    n_rev_rejected = int(((cmd == 1) & (act == 0)).sum())
    return {
        'n_samples': n_total,
        'mismatch_count': n_mis,
        'mismatch_pct': float(100.0 * n_mis / n_total),
        'n_cmd_rev': n_cmd_rev,
        'n_act_rev': n_act_rev,
        'n_rev_rejected': n_rev_rejected,
        'rev_rejection_pct': float(100.0 * n_rev_rejected / n_cmd_rev) if n_cmd_rev else 0.0,
    }


def gps_stats(fix_df: pd.DataFrame) -> Dict[str, Any]:
    if len(fix_df) == 0:
        return {'n_samples': 0}
    cov = fix_df['cov_xx'].values
    dt = np.diff(fix_df['t'].values) if len(fix_df) > 1 else np.array([0.0])
    return {
        'n_samples': int(len(fix_df)),
        'cov_xx_min': float(cov.min()),
        'cov_xx_median': float(np.median(cov)),
        'cov_xx_max': float(cov.max()),
        'cov_xx_mean': float(cov.mean()),
        'max_dt_s': float(dt.max()) if len(dt) else 0.0,
        'n_gaps_over_2s': int((dt > 2.0).sum()),
        'n_cov_over_1e-2': int((cov > 1e-2).sum()),   # RTK float or worse
        'n_cov_over_1': int((cov > 1.0).sum()),       # major dropout
        'status_hist': {int(k): int(v) for k, v in fix_df['status'].value_counts().to_dict().items()},
    }


# ============================================================================
# Full evaluation of one bag
# ============================================================================

def evaluate(bag_path: Path,
             cleaning_width: float = 1.0,
             stall_threshold: float = 0.02,
             stall_min_duration: float = 3.0,
             odom_topic: str = '/hakuroukun_pose/rear_wheel_odometry') -> Dict[str, Any]:
    """Return the full metrics dict for one bag (no plotting, no IO to disk)."""
    print(f"  reading {bag_path.name} ...", flush=True)
    bag = extract_bag(bag_path, odom_topic=odom_topic)

    traj = build_map_trajectory(bag)
    speed = compute_speed(traj) if len(traj) > 1 else np.array([])
    stalls = detect_stalls(traj, speed, stall_threshold, stall_min_duration) if len(speed) else []
    total_stall = sum(s['dur'] for s in stalls)

    duration = float(bag['t_end'] - bag['t_start'])
    if len(traj) > 1:
        pth = np.sqrt(np.diff(traj['x'].values)**2 + np.diff(traj['y'].values)**2).sum()
    else:
        pth = 0.0

    # Prefer planned_path; fall back to desired_path
    plan_source = None
    plan_pts = None
    if bag['planned_path'] and len(bag['planned_path']['points']) > 0:
        plan_pts = bag['planned_path']['points']
        plan_source = '/planned_path'
    elif bag['desired_path'] and len(bag['desired_path']['points']) > 0:
        plan_pts = bag['desired_path']['points']
        plan_source = '/desired_path'

    if plan_pts is not None and len(traj) > 0:
        savg, smax, srms, d_per_pt = trajectory_error(plan_pts, traj[['x', 'y']].values)
        n_gt_1m = int((d_per_pt > 1.0).sum())
        n_planned = len(plan_pts)
    else:
        savg = smax = srms = float('nan')
        d_per_pt = np.array([])
        n_gt_1m = 0
        n_planned = 0

    if bag['map'] is not None and len(traj) > 0:
        cov = compute_coverage(traj[['x', 'y']].values, bag['map'], cleaning_width)
    else:
        cov = None

    is_moving = speed > stall_threshold if len(speed) else np.array([], dtype=bool)
    mean_speed_moving = float(speed[is_moving].mean()) if is_moving.any() else 0.0
    frac_moving = float(100.0 * is_moving.mean()) if len(is_moving) else 0.0

    metrics = {
        'bag_file': str(bag_path.name),
        'bag_path': str(bag_path.absolute()),
        'evaluated_at': datetime.now().isoformat(timespec='seconds'),
        'params': {
            'cleaning_width_m': cleaning_width,
            'stall_threshold_mps': stall_threshold,
            'stall_min_duration_s': stall_min_duration,
        },
        'run': {
            't_start_epoch': bag['t_start'],
            't_end_epoch': bag['t_end'],
            'duration_s': duration,
            'duration_min': duration / 60.0,
            'path_length_m': float(pth),
            'mean_speed_when_moving_mps': mean_speed_moving,
            'fraction_time_moving_pct': frac_moving,
            'n_odom_samples': int(len(traj)),
            'planned_source': plan_source,
            'n_planned_points': n_planned,
        },
        'coverage': {
            'coverage_pct': cov['coverage_pct'] if cov else float('nan'),
            'cleaned_area_m2': cov['cleaned_area_m2'] if cov else float('nan'),
            'total_valid_area_m2': cov['total_valid_area_m2'] if cov else float('nan'),
            'restricted_violations': cov['restricted_violations'] if cov else 0,
            'restricted_violations_pct': cov['restricted_violations_pct'] if cov else 0.0,
        },
        'trajectory_error': {
            'savg_m': savg,
            'smax_m': smax,
            'rms_m': srms,
            'n_planned_over_1m_off': n_gt_1m,
            'pct_planned_over_1m_off': float(100.0 * n_gt_1m / n_planned) if n_planned else 0.0,
        },
        'stalls': {
            'count': len(stalls),
            'total_stall_time_s': float(total_stall),
            'pct_of_run_stalled': float(100.0 * total_stall / duration) if duration else 0.0,
        },
        'gear': gear_stats(bag['gear']),
        'gps': gps_stats(bag['fix']),
    }

    # Attach data blobs for plotting (not written to JSON)
    metrics['_traj'] = traj
    metrics['_speed'] = speed
    metrics['_stalls'] = stalls
    metrics['_planned_pts'] = plan_pts
    metrics['_d_per_pt'] = d_per_pt
    metrics['_coverage_arrays'] = cov
    metrics['_bag'] = bag
    return metrics


# ============================================================================
# Formatting & plotting
# ============================================================================

def format_summary(m: Dict[str, Any]) -> str:
    r = m['run']; c = m['coverage']; te = m['trajectory_error']
    st = m['stalls']; g = m.get('gear', {}); gp = m.get('gps', {})
    lines = [
        f"Bag:               {m['bag_file']}",
        f"Duration:          {r['duration_s']:.1f} s   ({r['duration_min']:.2f} min)",
        f"Path length:       {r['path_length_m']:.2f} m",
        f"Mean speed moving: {r['mean_speed_when_moving_mps']:.3f} m/s   "
        f"({r['fraction_time_moving_pct']:.1f}% of run in motion)",
        f"Planned source:    {r['planned_source']}  ({r['n_planned_points']} pts)",
        "",
        f"Coverage:          {c['coverage_pct']:.2f} %   "
        f"(cleaned {c['cleaned_area_m2']:.2f} / valid {c['total_valid_area_m2']:.2f} m²)",
        f"Restricted-cell hits: {c['restricted_violations']} samples "
        f"({c['restricted_violations_pct']:.2f}% of trajectory)",
        "",
        f"Trajectory error (nearest-neighbor planned -> actual):",
        f"  Savg = {te['savg_m']:.4f} m   Smax = {te['smax_m']:.4f} m   RMS = {te['rms_m']:.4f} m",
        f"  Planned pts > 1 m off: {te['n_planned_over_1m_off']} "
        f"({te['pct_planned_over_1m_off']:.1f}%)",
        "",
        f"Stalls (< {m['params']['stall_threshold_mps']} m/s for "
        f">= {m['params']['stall_min_duration_s']:.1f} s):",
        f"  count = {st['count']}  total = {st['total_stall_time_s']:.1f} s "
        f"({st['pct_of_run_stalled']:.1f}% of run)",
    ]
    if g.get('n_samples', 0):
        lines += [
            "",
            f"Gear (commanded vs actual):",
            f"  Overall mismatch:  {g['mismatch_count']}/{g['n_samples']} "
            f"({g['mismatch_pct']:.2f}%)",
            f"  Reverse commanded: {g['n_cmd_rev']}   actual reverse engaged: {g['n_act_rev']}",
            f"  Reverse rejected:  {g['n_rev_rejected']} "
            f"({g['rev_rejection_pct']:.1f}% of reverse commands)",
        ]
    if gp.get('n_samples', 0):
        lines += [
            "",
            f"GPS quality ({gp['n_samples']} fixes):",
            f"  cov_xx min={gp['cov_xx_min']:.2e}  median={gp['cov_xx_median']:.2e}  "
            f"max={gp['cov_xx_max']:.2e}",
            f"  fixes with cov > 0.01 m²: {gp['n_cov_over_1e-2']}   "
            f"cov > 1 m²: {gp['n_cov_over_1']}",
            f"  max gap between fixes: {gp['max_dt_s']:.2f} s   "
            f"gaps > 2 s: {gp['n_gaps_over_2s']}",
            f"  status hist: {gp['status_hist']}",
        ]
    return "\n".join(lines)


def make_plot(m: Dict[str, Any], out_path: Path) -> None:
    traj = m['_traj']; speed = m['_speed']
    stalls = m['_stalls']; plan_pts = m['_planned_pts']
    d = m['_d_per_pt']; cov = m['_coverage_arrays']; bag = m['_bag']

    if len(traj) == 0 or cov is None or plan_pts is None:
        print(f"  (skipping plot for {m['bag_file']} — insufficient data)")
        return

    px, py = zip(*plan_pts)
    px = np.array(px); py = np.array(py)
    # zoom to combined extent + small margin
    xs_all = np.concatenate([px, traj['x'].values])
    ys_all = np.concatenate([py, traj['y'].values])
    xmin, xmax = xs_all.min() - 1.5, xs_all.max() + 1.5
    ymin, ymax = ys_all.min() - 1.5, ys_all.max() + 1.5

    grid = cov['grid']
    res = cov['resolution']
    map_info = bag['map']
    extent = [map_info['origin_x'],
              map_info['origin_x'] + grid.shape[1] * res,
              map_info['origin_y'],
              map_info['origin_y'] + grid.shape[0] * res]

    fig = plt.figure(figsize=(15, 20))
    gs = fig.add_gridspec(5, 2, hspace=0.35, wspace=0.25)

    # --- planned vs actual ---
    ax = fig.add_subplot(gs[0, 0])
    disp = np.ones_like(grid, dtype=float)
    disp[grid == 100] = 0.0
    disp[grid == -1] = 0.85
    ax.imshow(disp, origin='lower', extent=extent, cmap='gray', vmin=0, vmax=1, alpha=0.7)
    ax.plot(px, py, 'b-', lw=1.0, alpha=0.6, label=f'planned ({len(plan_pts)} pts)')
    tt = (traj['t'].values - traj['t'].iloc[0]) / max(traj['t'].iloc[-1] - traj['t'].iloc[0], 1e-9)
    ax.scatter(traj['x'].values, traj['y'].values, c=tt, cmap='plasma', s=1.2, alpha=0.7)
    ax.plot(traj['x'].iloc[0], traj['y'].iloc[0], 'go', ms=10, mec='k', label='start')
    ax.plot(traj['x'].iloc[-1], traj['y'].iloc[-1], 'ks', ms=10, label='end')
    for st in stalls:
        ax.plot(st['x'], st['y'], 'y*', ms=12, mec='k', mew=0.7)
    ax.set_xlim(xmin, xmax); ax.set_ylim(ymin, ymax); ax.set_aspect('equal')
    ax.set_title('Planned (blue) vs actual (time colored)   ★=stalls')
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    ax.legend(loc='lower left', fontsize=8); ax.grid(alpha=0.3)

    # --- coverage ---
    ax = fig.add_subplot(gs[0, 1])
    d3 = np.ones((*grid.shape, 3))
    d3[grid == 100] = [0.1, 0.1, 0.1]
    d3[grid == -1] = [0.85, 0.85, 0.85]
    cleaned_mask = cov['cleaned_mask'] & cov['valid_mask']
    d3[cleaned_mask] = [0.2, 0.75, 0.35]
    ax.imshow(d3, origin='lower', extent=extent)
    ax.plot(traj['x'].values, traj['y'].values, 'r-', lw=0.5, alpha=0.5)
    ax.set_xlim(xmin, xmax); ax.set_ylim(ymin, ymax); ax.set_aspect('equal')
    ax.set_title(f"Coverage = {cov['coverage_pct']:.2f}%   "
                 f"({cov['cleaned_area_m2']:.1f}/{cov['total_valid_area_m2']:.1f} m²)")
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.grid(alpha=0.3)

    # --- speed + reverse-command overlay ---
    ax = fig.add_subplot(gs[1, :])
    t0 = traj['t'].iloc[0]
    t_rel = traj['t'].values[1:] - t0
    ax.plot(t_rel, speed, 'b-', lw=0.5, alpha=0.7, label='|speed|')
    if len(bag['gear']) and 'g0' in bag['gear'].columns:
        gt = bag['gear']['t'].values - t0
        cmd_g = bag['gear']['g0'].values
        act_g = bag['gear']['g1'].values
        rev_mask = (cmd_g == 1)
        if rev_mask.any():
            diff = np.diff(rev_mask.astype(int))
            starts = np.where(diff == 1)[0] + 1
            ends = np.where(diff == -1)[0] + 1
            if rev_mask[0]: starts = np.r_[0, starts]
            if rev_mask[-1]: ends = np.r_[ends, len(rev_mask)]
            for a, b in zip(starts, ends):
                ax.axvspan(gt[a], gt[b-1], alpha=0.18, color='red')
        mm = (cmd_g == 1) & (act_g == 0)
        if mm.any():
            ax.plot(gt[mm], np.full(mm.sum(), -0.03), 'rx', ms=4, alpha=0.5,
                    label='rev cmd rejected')
    for st in stalls:
        ax.axvspan(st['t_start'], st['t_end'], alpha=0.13, color='yellow')
    ax.axhline(m['params']['stall_threshold_mps'], color='orange', lw=0.5, ls='--', alpha=0.5)
    ax.set_ylim(-0.08, max(0.7, float(np.nanpercentile(speed, 99.5)) * 1.15) if len(speed) else 0.7)
    ax.set_xlabel('time [s]'); ax.set_ylabel('speed [m/s]')
    ax.set_title(f"Speed   yellow=stall bands   red-shaded=commanded reverse   ✗=reverse rejected")
    ax.legend(fontsize=8, loc='upper right'); ax.grid(alpha=0.3)

    # --- planned-point deviation heatmap ---
    ax = fig.add_subplot(gs[2, 0])
    sc = ax.scatter(px, py, c=d, cmap='hot_r', s=4, vmin=0, vmax=3)
    plt.colorbar(sc, ax=ax, label='dist to nearest actual [m]')
    ax.set_xlim(xmin, xmax); ax.set_ylim(ymin, ymax); ax.set_aspect('equal')
    te = m['trajectory_error']
    ax.set_title(f"Planned pt deviation | Savg={te['savg_m']:.2f}  "
                 f"Smax={te['smax_m']:.2f}  RMS={te['rms_m']:.2f} m")
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.grid(alpha=0.3)

    # --- deviation histogram ---
    ax = fig.add_subplot(gs[2, 1])
    if len(d):
        ax.hist(d, bins=40, color='steelblue', edgecolor='k', alpha=0.8)
        ax.axvline(1.0, color='red', ls='--', lw=1, label='1 m threshold')
        ax.set_xlabel('nearest-neighbor deviation [m]')
        ax.set_ylabel('# planned points')
        ax.set_title(f"Deviation histogram | {te['n_planned_over_1m_off']}/"
                     f"{m['run']['n_planned_points']} pts > 1 m "
                     f"({te['pct_planned_over_1m_off']:.0f}%)")
        ax.legend(fontsize=8); ax.grid(alpha=0.3)

    # --- GPS covariance ---
    ax = fig.add_subplot(gs[3, :])
    if len(bag['fix']):
        ax.semilogy(bag['fix']['t'].values - t0, bag['fix']['cov_xx'].values, 'g-', lw=0.7, label='cov_xx')
        ax.semilogy(bag['fix']['t'].values - t0, bag['fix']['cov_yy'].values, 'b-', lw=0.7, alpha=0.6, label='cov_yy')
        ax.axhline(1e-3, color='orange', lw=0.5, ls='--', alpha=0.5, label='1e-3 (great RTK)')
        ax.axhline(1e-2, color='red', lw=0.5, ls='--', alpha=0.5, label='1e-2 (RTK float)')
        gp = m['gps']
        ax.set_title(f"GPS covariance | median={gp['cov_xx_median']:.2e}  "
                     f"max={gp['cov_xx_max']:.2e}  fixes>1e-2: {gp['n_cov_over_1e-2']}")
        ax.set_xlabel('time [s]'); ax.set_ylabel('cov [m²]')
        ax.legend(fontsize=8, loc='upper right'); ax.grid(alpha=0.3, which='both')

    # --- gear cmd vs actual ---
    ax = fig.add_subplot(gs[4, :])
    if len(bag['gear']) and 'g0' in bag['gear'].columns:
        gt = bag['gear']['t'].values - t0
        cmd_g = bag['gear']['g0'].values
        act_g = bag['gear']['g1'].values
        ax.plot(gt, cmd_g + 0.05, 'r.', ms=1.5, alpha=0.5, label='commanded (+0.05 offset)')
        ax.plot(gt, act_g, 'b.', ms=1.5, alpha=0.5, label='actual')
        g = m['gear']
        ax.set_title(f"Gear cmd vs actual | mismatch {g['mismatch_count']}/{g['n_samples']} "
                     f"({g['mismatch_pct']:.1f}%) | rev rejected "
                     f"{g['n_rev_rejected']}/{g['n_cmd_rev']} ({g['rev_rejection_pct']:.0f}%)")
        ax.set_xlabel('time [s]'); ax.set_ylabel('gear (0=fwd, 1=rev)')
        ax.set_ylim(-0.25, 1.35)
        ax.legend(fontsize=8, loc='upper right'); ax.grid(alpha=0.3)

    fig.suptitle(f"{m['bag_file']}", fontsize=14, y=0.995)
    plt.savefig(out_path, dpi=110, bbox_inches='tight')
    plt.close(fig)


def write_outputs(m: Dict[str, Any], out_dir: Path, make_plots: bool = True) -> None:
    """Write JSON, TXT summary, and (optionally) PNG plot for one evaluation."""
    stem = Path(m['bag_file']).stem
    out_dir.mkdir(parents=True, exist_ok=True)
    # strip non-serializable blobs for JSON
    clean = {k: v for k, v in m.items() if not k.startswith('_')}
    (out_dir / f"{stem}_metrics.json").write_text(json.dumps(clean, indent=2))
    (out_dir / f"{stem}_summary.txt").write_text(format_summary(m) + "\n")
    if make_plots:
        make_plot(m, out_dir / f"{stem}_plot.png")


# ============================================================================
# CLI
# ============================================================================

BATCH_COLUMNS = [
    'bag_file', 'duration_min', 'path_length_m',
    'coverage_pct', 'cleaned_area_m2', 'total_valid_area_m2',
    'savg_m', 'smax_m', 'rms_m', 'pct_planned_over_1m_off',
    'stall_count', 'stall_total_s', 'stall_pct_of_run',
    'gear_mismatch_pct', 'n_cmd_rev', 'rev_rejection_pct',
    'gps_cov_median', 'gps_cov_max', 'n_gps_cov_over_1e-2',
    'restricted_violations',
]


def to_row(m: Dict[str, Any]) -> Dict[str, Any]:
    r = m['run']; c = m['coverage']; te = m['trajectory_error']
    st = m['stalls']; g = m.get('gear', {}); gp = m.get('gps', {})
    return {
        'bag_file': m['bag_file'],
        'duration_min': r['duration_min'],
        'path_length_m': r['path_length_m'],
        'coverage_pct': c['coverage_pct'],
        'cleaned_area_m2': c['cleaned_area_m2'],
        'total_valid_area_m2': c['total_valid_area_m2'],
        'savg_m': te['savg_m'],
        'smax_m': te['smax_m'],
        'rms_m': te['rms_m'],
        'pct_planned_over_1m_off': te['pct_planned_over_1m_off'],
        'stall_count': st['count'],
        'stall_total_s': st['total_stall_time_s'],
        'stall_pct_of_run': st['pct_of_run_stalled'],
        'gear_mismatch_pct': g.get('mismatch_pct', float('nan')),
        'n_cmd_rev': g.get('n_cmd_rev', 0),
        'rev_rejection_pct': g.get('rev_rejection_pct', float('nan')),
        'gps_cov_median': gp.get('cov_xx_median', float('nan')),
        'gps_cov_max': gp.get('cov_xx_max', float('nan')),
        'n_gps_cov_over_1e-2': gp.get('n_cov_over_1e-2', 0),
        'restricted_violations': c['restricted_violations'],
    }


def resolve_paths(patterns: List[str]) -> List[Path]:
    out: List[Path] = []
    for pat in patterns:
        matches = sorted(glob.glob(pat))
        if matches:
            out.extend(Path(p) for p in matches)
        else:
            p = Path(pat)
            if p.exists():
                out.append(p)
            else:
                sys.stderr.write(f"WARNING: no bag matched '{pat}'\n")
    return out


def main() -> int:
    ap = argparse.ArgumentParser(
        description='Evaluate a Hakuroukun coverage rosbag.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split('Usage:')[1].split('Requires:')[0] if '__doc__' else '')
    ap.add_argument('bags', nargs='*', help='bag file path(s), or use --batch')
    ap.add_argument('--batch', nargs='+', metavar='PATTERN',
                    help='batch mode: glob pattern(s) or list of bag paths')
    ap.add_argument('--output-dir', default='./eval_results', type=Path,
                    help='directory to write outputs (default: ./eval_results)')
    ap.add_argument('--cleaning-width', type=float, default=1.0,
                    help='robot cleaning footprint width in m (default 1.0, matches Tai)')
    ap.add_argument('--stall-threshold', type=float, default=0.02,
                    help='speed threshold (m/s) below which robot is "stalled" (default 0.02)')
    ap.add_argument('--stall-min-duration', type=float, default=3.0,
                    help='minimum stall duration in seconds to count (default 3.0)')
    ap.add_argument('--no-plots', action='store_true',
                    help='skip PNG plot generation (faster for large batches)')
    ap.add_argument('--odom-topic', default='/hakuroukun_pose/rear_wheel_odometry',
                    help='nav_msgs/Odometry topic to use as the robot pose feed. '
                         'Default is the real-robot fused pose. For sim bags using '
                         'ground-truth odometry (e.g. offline_path_planning_conemap_df.launch '
                         'with odom_tf_broadcaster reading /ground_truth/odometry), pass '
                         '--odom-topic /ground_truth/odometry')

    args = ap.parse_args()

    if args.batch:
        bag_paths = resolve_paths(args.batch)
    elif args.bags:
        bag_paths = resolve_paths(args.bags)
    else:
        ap.print_help()
        return 1

    if not bag_paths:
        sys.stderr.write("No bags to evaluate.\n")
        return 1

    args.output_dir.mkdir(parents=True, exist_ok=True)
    rows: List[Dict[str, Any]] = []

    for i, bp in enumerate(bag_paths, 1):
        print(f"[{i}/{len(bag_paths)}] {bp.name}", flush=True)
        try:
            m = evaluate(bp,
                         cleaning_width=args.cleaning_width,
                         stall_threshold=args.stall_threshold,
                         stall_min_duration=args.stall_min_duration,
                         odom_topic=args.odom_topic)
            write_outputs(m, args.output_dir, make_plots=not args.no_plots)
            rows.append(to_row(m))
            print(format_summary(m))
            print()
        except Exception as e:
            sys.stderr.write(f"  FAILED on {bp.name}: {type(e).__name__}: {e}\n")
            import traceback; traceback.print_exc(file=sys.stderr)

    if len(rows) > 1 or args.batch:
        df = pd.DataFrame(rows, columns=BATCH_COLUMNS).sort_values(
            'coverage_pct', ascending=False, na_position='last')
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_path = args.output_dir / f'comparison_{stamp}.csv'
        df.to_csv(csv_path, index=False, float_format='%.4f')
        print(f"\nComparison CSV: {csv_path}")
        print(df.to_string(index=False, float_format=lambda x: f'{x:.3f}'))

    return 0


if __name__ == '__main__':
    sys.exit(main())