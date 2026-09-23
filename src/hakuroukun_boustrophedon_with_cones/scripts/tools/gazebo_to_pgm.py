#!/usr/bin/env python3
"""gazebo_world_to_pgm.py

Rasterize a Gazebo .world file into a ROS occupancy-grid PGM/YAML pair.

Motivation
----------
The warehouse static map used for BCD planning and coverage evaluation was
originally produced by a gmapping pass, which introduces noise, wall
fringes, and small registration offsets vs. the actual Gazebo geometry.
This means "coverage %" against that map does not exactly correspond to
"coverage %" against the world the robot is actually driving in.

This script bypasses SLAM entirely: it reads the .world SDF directly and
draws every wall box + construction-cone footprint into a PGM at pixel
alignment. The resulting map is a perfect, noise-free, drop-in
replacement for the gmapping-derived one, so BCD plans against the same
geometry that Gazebo simulates, and evaluate_real_run.py grades against
the same geometry too.

What it handles
---------------
- <model> elements with a top-level <pose>
- <link> nested inside each model with optional <pose>
- <collision> inside each link with optional <pose>
- Geometry types:
    * <box><size>W H Z</size></box>   -> rotated rectangle (uses composed yaw)
    * <mesh><uri>...construction_cone...</uri></mesh>
                                       -> filled circle (radius from CLI)
- Pose composition: world_pose = model.pose o link.pose o collision.pose
  (yaw-only; roll/pitch ignored — all warehouse walls are Z-axis rotations)
- Height filter: only geometry whose vertical extent overlaps
  [z_filter_min, z_filter_max] is drawn (default 0.05 .. 2.0 m — matches
  a typical LiDAR scan plane and excludes the ground plane).
- Skips models by name: 'ground_plane' (and anything the user passes via
  --skip-models).

What it does NOT handle
-----------------------
- <include><uri>model://...</uri></include> at world scope. If the
  warehouse world grows to pull in external models with wall geometry,
  those would need to be resolved by parsing the included .sdf too. The
  current warehouse world defines all wall boxes inline, so this is not
  a problem today.
- Cylinders, spheres, arbitrary meshes other than construction_cone.
  Warns and skips.
- Arbitrary roll/pitch. Warns if any model has non-negligible roll/pitch.
"""

import argparse
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw


# -----------------------------------------------------------------------------
# Pose helpers
# -----------------------------------------------------------------------------

def parse_pose(text):
    """Parse a Gazebo <pose>x y z roll pitch yaw</pose> into a 6-tuple.

    Missing values default to 0. If the element is missing entirely,
    callers should pass None and this function isn't invoked.
    """
    if text is None:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    parts = text.strip().split()
    vals = [float(p) for p in parts]
    while len(vals) < 6:
        vals.append(0.0)
    return tuple(vals[:6])


def compose_pose(parent, child):
    """Compose two SDF poses assuming Z-axis-only rotation.

    world_pos = parent_pos + Rz(parent_yaw) * child_pos
    world_yaw = parent_yaw + child_yaw
    """
    px, py, pz, proll, ppitch, pyaw = parent
    cx, cy, cz, croll, cpitch, cyaw = child
    c = np.cos(pyaw)
    s = np.sin(pyaw)
    wx = px + c * cx - s * cy
    wy = py + s * cx + c * cy
    wz = pz + cz
    return (wx, wy, wz, proll + croll, ppitch + cpitch, pyaw + cyaw)


def get_child_pose(element):
    """Return (x,y,z,roll,pitch,yaw) from the direct <pose> child of an SDF
    element, or the identity pose if absent."""
    p = element.find('pose')
    if p is None:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    return parse_pose(p.text)


# -----------------------------------------------------------------------------
# World -> collision list
# -----------------------------------------------------------------------------

def extract_collisions(world_path, skip_models, warn):
    """Walk the .world SDF and yield dicts describing each collision to draw.

    Returned dicts have:
        kind: 'box' or 'cone'
        world_x, world_y: center in world frame (m)
        world_yaw: yaw in world frame (rad)  [box only]
        size_x, size_y: box footprint (m)    [box only]
        radius: circle radius (m)            [cone only]
        z_min, z_max: vertical extent for height filtering (m)
        model_name: for diagnostics
    """
    tree = ET.parse(world_path)
    root = tree.getroot()

    # Gazebo files sometimes wrap in <sdf><world>, sometimes just <world>.
    world = root.find('world') if root.tag != 'world' else root
    if world is None:
        raise SystemExit(f"No <world> element found in {world_path}")

    n_seen = 0
    n_drawn = 0
    n_skipped_geom = 0
    for model in world.findall('model'):
        name = model.get('name', '')
        if name in skip_models:
            continue

        model_pose = get_child_pose(model)
        # Sanity: warn about non-trivial roll/pitch (we ignore them).
        _, _, _, mroll, mpitch, _ = model_pose
        if abs(mroll) > 0.01 or abs(mpitch) > 0.01:
            warn(f"model '{name}' has non-zero roll/pitch — ignoring "
                 f"(roll={mroll:.3f}, pitch={mpitch:.3f})")

        for link in model.findall('link'):
            link_pose = get_child_pose(link)
            mp_lp = compose_pose(model_pose, link_pose)

            for coll in link.findall('collision'):
                n_seen += 1
                coll_pose = get_child_pose(coll)
                world_pose = compose_pose(mp_lp, coll_pose)
                wx, wy, wz, _, _, wyaw = world_pose

                geom = coll.find('geometry')
                if geom is None:
                    n_skipped_geom += 1
                    continue

                box = geom.find('box')
                if box is not None:
                    size_text = box.findtext('size', default='').strip()
                    parts = size_text.split()
                    if len(parts) != 3:
                        warn(f"model '{name}': box size malformed "
                             f"('{size_text}'), skipping")
                        n_skipped_geom += 1
                        continue
                    sx, sy, sz = (float(p) for p in parts)
                    yield dict(
                        kind='box',
                        model_name=name,
                        world_x=wx, world_y=wy,
                        world_yaw=wyaw,
                        size_x=sx, size_y=sy,
                        z_min=wz - sz / 2.0,
                        z_max=wz + sz / 2.0,
                    )
                    n_drawn += 1
                    continue

                mesh = geom.find('mesh')
                if mesh is not None:
                    uri = mesh.findtext('uri', default='')
                    if 'construction_cone' in uri:
                        scale_text = mesh.findtext('scale', default='1 1 1')
                        sc = [float(p) for p in scale_text.split()]
                        while len(sc) < 3:
                            sc.append(1.0)
                        # Standard Gazebo construction_cone.dae is ~0.35m tall
                        # with ~0.17m base radius at scale 1.0. We only care
                        # about the vertical extent for filtering; radius is
                        # supplied by the caller (--cone-radius).
                        z_top = wz + 0.35 * sc[2]
                        yield dict(
                            kind='cone',
                            model_name=name,
                            world_x=wx, world_y=wy,
                            # radius filled in later using --cone-radius
                            z_min=wz,
                            z_max=z_top,
                        )
                        n_drawn += 1
                        continue
                    else:
                        warn(f"model '{name}': non-cone mesh uri "
                             f"'{uri}', skipping")
                        n_skipped_geom += 1
                        continue

                # Any other geometry (cylinder, sphere, ...)
                geom_kinds = [child.tag for child in list(geom)]
                warn(f"model '{name}': unsupported geometry "
                     f"{geom_kinds}, skipping")
                n_skipped_geom += 1

    print(f"[gazebo->pgm] collisions seen: {n_seen}, drawable: {n_drawn}, "
          f"skipped: {n_skipped_geom}")


# -----------------------------------------------------------------------------
# Rasterization
# -----------------------------------------------------------------------------

def world_to_px(wx, wy, origin_x, origin_y, res, height_px):
    """Convert world (m) to pixel (col, row) with Y flipped for PGM."""
    col = (wx - origin_x) / res
    row_from_bottom = (wy - origin_y) / res
    row = height_px - row_from_bottom  # PGM has Y-down
    return col, row


def rasterize(collisions, width_px, height_px, origin_x, origin_y, res,
              cone_radius, z_min_keep, z_max_keep):
    """Draw all collisions onto a PGM image.

    Pixel values:
        255 = free (default)
        0   = occupied (walls, cones)
    We do NOT emit an unknown/grey band here — the entire canvas is
    "known free" by construction, except where geometry marks it occupied.
    If the launch YAML expects a grey border, run `map_server` normally;
    the grey ring at map edges is created by rviz/map_server based on
    negate/thresholds only when the source PGM has unknown pixels. Since
    we have no unknown region, everything outside the drawn walls is
    treated as free — which is correct: it IS free in Gazebo (no walls
    there, robot could physically drive there).
    """
    img = Image.new('L', (width_px, height_px), color=255)
    draw = ImageDraw.Draw(img)

    n_drawn = 0
    n_height_filtered = 0
    for c in collisions:
        # Height filter: only draw geometry whose z-extent overlaps the
        # keep band (typically the LiDAR scan plane).
        if c['z_max'] < z_min_keep or c['z_min'] > z_max_keep:
            n_height_filtered += 1
            continue

        if c['kind'] == 'box':
            # Compute the 4 corners of the rotated rectangle in world coords.
            hx = c['size_x'] / 2.0
            hy = c['size_y'] / 2.0
            corners_local = np.array([
                [-hx, -hy],
                [ hx, -hy],
                [ hx,  hy],
                [-hx,  hy],
            ])
            cos_y = np.cos(c['world_yaw'])
            sin_y = np.sin(c['world_yaw'])
            R = np.array([[cos_y, -sin_y], [sin_y, cos_y]])
            corners_world = (R @ corners_local.T).T + np.array(
                [c['world_x'], c['world_y']])

            # Convert to pixel space and draw filled polygon.
            px_pts = [world_to_px(x, y, origin_x, origin_y, res, height_px)
                      for x, y in corners_world]
            draw.polygon(px_pts, fill=0)
            n_drawn += 1

        elif c['kind'] == 'cone':
            cx_px, cy_px = world_to_px(c['world_x'], c['world_y'],
                                       origin_x, origin_y, res, height_px)
            r_px = cone_radius / res
            draw.ellipse(
                [cx_px - r_px, cy_px - r_px, cx_px + r_px, cy_px + r_px],
                fill=0)
            n_drawn += 1

    print(f"[gazebo->pgm] drew {n_drawn} shapes, "
          f"filtered {n_height_filtered} by height")
    return img


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--world', required=True,
                    help='Path to the Gazebo .world SDF file')
    ap.add_argument('--out-pgm', required=True,
                    help='Output PGM path (e.g. warehouse_map_gt.pgm)')
    ap.add_argument('--out-yaml', required=True,
                    help='Output YAML path (e.g. warehouse_map_gt.yaml)')

    # Match the existing warehouse_map.yaml defaults.
    ap.add_argument('--resolution', type=float, default=0.05,
                    help='Map resolution in m/px (default 0.05)')
    ap.add_argument('--origin-x', type=float, default=-25.0,
                    help='Map origin X in world frame (m, default -25)')
    ap.add_argument('--origin-y', type=float, default=-25.0,
                    help='Map origin Y in world frame (m, default -25)')
    ap.add_argument('--width-px', type=int, default=4000,
                    help='Map width in pixels (default 4000 = 200 m at 0.05)')
    ap.add_argument('--height-px', type=int, default=4000,
                    help='Map height in pixels (default 4000)')

    ap.add_argument('--cone-radius', type=float, default=1.0,
                    help='Cone footprint radius in meters (default 1.0 m, '
                         'matches robot_radius so cones are marked as '
                         'forbidden regions the planner will keep clear).')

    # Height band for what counts as an obstacle. LiDAR at ~0.35m; anything
    # taller than the ground plane and shorter than the robot is a wall.
    ap.add_argument('--z-min-keep', type=float, default=0.05,
                    help='Minimum z of obstacle vertical extent to draw (m)')
    ap.add_argument('--z-max-keep', type=float, default=2.0,
                    help='Maximum z of obstacle vertical extent to draw (m)')

    ap.add_argument('--skip-models', nargs='*',
                    default=['ground_plane'],
                    help="Model names to skip (default: ['ground_plane'])")

    ap.add_argument('--quiet', action='store_true',
                    help='Suppress per-model warnings')

    args = ap.parse_args()

    if not os.path.isfile(args.world):
        raise SystemExit(f"World file not found: {args.world}")

    warnings = []
    def warn(msg):
        warnings.append(msg)
        if not args.quiet:
            print(f"[gazebo->pgm] WARN: {msg}", file=sys.stderr)

    collisions = list(extract_collisions(
        args.world, skip_models=set(args.skip_models), warn=warn))

    # Backfill cone radius (kept out of extract_collisions so we can size
    # from --cone-radius without needing to re-parse).
    for c in collisions:
        if c['kind'] == 'cone':
            c['radius'] = args.cone_radius

    img = rasterize(collisions,
                    width_px=args.width_px, height_px=args.height_px,
                    origin_x=args.origin_x, origin_y=args.origin_y,
                    res=args.resolution,
                    cone_radius=args.cone_radius,
                    z_min_keep=args.z_min_keep, z_max_keep=args.z_max_keep)

    # Save PGM (P5, grayscale).
    img.save(args.out_pgm)

    # Write YAML matching ROS map_server format. The image path is stored
    # as an absolute path if that's what was passed, otherwise as basename
    # (map_server resolves relative paths from the YAML's directory).
    pgm_ref = args.out_pgm
    if not os.path.isabs(pgm_ref):
        pgm_ref = os.path.basename(pgm_ref)
    yaml_body = (
        f"image: {pgm_ref}\n"
        f"resolution: {args.resolution:.6f}\n"
        f"origin: [{args.origin_x:.6f}, {args.origin_y:.6f}, 0.000000]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.196\n"
    )
    with open(args.out_yaml, 'w') as f:
        f.write(yaml_body)

    print(f"[gazebo->pgm] wrote {args.out_pgm} "
          f"({args.width_px}x{args.height_px} px)")
    print(f"[gazebo->pgm] wrote {args.out_yaml}")
    if warnings:
        print(f"[gazebo->pgm] {len(warnings)} warning(s) — see stderr")


if __name__ == '__main__':
    main()