#!/usr/bin/env python3
"""
pgm_to_gazebo_world.py
Convert a ROS occupancy-grid .pgm/.yaml pair into a Gazebo .world file whose
walls are extruded from the occupied pixels of the map.

Assumptions
-----------
- Only near-black pixels (< occ_threshold) are treated as walls. Gray "unknown"
  (typically 205) and white "free" (typically 254) are ignored. If the input
  map ever bakes cone-restricted zones in as black, they will become physical
  walls too — repaint them gray (205) first if that is not desired.
- Walls are extruded to a single fixed height (default 1.5 m). This is tall
  enough to be seen by any reasonably-mounted LiDAR and short enough not to
  break shadow / lighting expectations.

Method
------
For each row of the image, we run-length-encode contiguous wall pixels into
one axis-aligned box. This keeps the SDF box count in the low hundreds for
typical outline maps (vs. one box per pixel = tens of thousands).
Overlapping / touching boxes are physically fine in ODE.

Coordinate convention
---------------------
ROS map_server: `origin` is the world-frame pose of the map's lower-left
pixel. Image row 0 is at the TOP of the image, so:
    world_x_center = origin_x + (col + 0.5) * resolution
    world_y_center = origin_y + (H - row - 0.5) * resolution
where H is image height in pixels.

Usage
-----
    python3 pgm_to_gazebo_world.py \
        --yaml  df_area_outline_painted.yaml \
        --out   df_walls.world \
        --template flat_ground.world     # optional; inherits sun/ground/physics
"""
import argparse
import os
import re
import sys
import numpy as np
from PIL import Image


def load_map(yaml_path):
    """Minimal YAML parser — we only need image, resolution, origin."""
    with open(yaml_path) as f:
        text = f.read()

    def find(key):
        m = re.search(rf"^\s*{key}\s*:\s*(.+?)\s*$", text, re.MULTILINE)
        if not m:
            raise ValueError(f"'{key}' not found in {yaml_path}")
        return m.group(1)

    image_name = find("image").strip()
    resolution = float(find("resolution"))
    origin_raw = find("origin")
    nums = re.findall(r"-?\d+\.?\d*", origin_raw)
    if len(nums) < 2:
        raise ValueError(f"could not parse origin from '{origin_raw}'")
    origin_x = float(nums[0])
    origin_y = float(nums[1])

    # Support relative image paths (same directory as yaml)
    if not os.path.isabs(image_name):
        image_name = os.path.join(os.path.dirname(yaml_path), image_name)

    img = np.array(Image.open(image_name))
    return img, resolution, origin_x, origin_y


def row_runs(occupied_row):
    """Yield (col_start, col_end_inclusive) for each contiguous True run."""
    in_run = False
    start = 0
    for c, v in enumerate(occupied_row):
        if v and not in_run:
            start = c
            in_run = True
        elif not v and in_run:
            yield start, c - 1
            in_run = False
    if in_run:
        yield start, len(occupied_row) - 1


def build_boxes(img, resolution, origin_x, origin_y, occ_threshold, height):
    """Return list of dicts describing each wall box in world coordinates."""
    H, W = img.shape
    occupied = img < occ_threshold
    boxes = []
    for r in range(H):
        for c0, c1 in row_runs(occupied[r]):
            length_cells = c1 - c0 + 1
            size_x = length_cells * resolution
            size_y = resolution
            size_z = height
            cx = origin_x + (c0 + c1 + 1) * 0.5 * resolution
            cy = origin_y + (H - r - 0.5) * resolution
            cz = height * 0.5
            boxes.append(dict(cx=cx, cy=cy, cz=cz,
                              sx=size_x, sy=size_y, sz=size_z))
    return boxes


BOX_TEMPLATE = """\
      <link name="wall_{i:05d}">
        <pose>{cx:.4f} {cy:.4f} {cz:.4f} 0 0 0</pose>
        <collision name="col">
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
        </collision>
        <visual name="vis">
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>"""


WORLD_TEMPLATE = """\
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="df_walled">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>

    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>false</shadows>
    </scene>

    <gravity>0 0 -9.8</gravity>

    <!-- All wall boxes are grouped into one static model so Gazebo treats
         them as immovable terrain. -->
    <model name="df_walls">
      <static>true</static>
{links}
    </model>
  </world>
</sdf>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--yaml", required=True, help="ROS map yaml")
    ap.add_argument("--out", required=True, help="Output .world path")
    ap.add_argument("--occ-threshold", type=int, default=128,
                    help="Pixel value below which a cell is treated as wall "
                         "(default 128 — captures 0=black walls only)")
    ap.add_argument("--height", type=float, default=1.5,
                    help="Wall extrusion height in metres (default 1.5)")
    args = ap.parse_args()

    img, resolution, origin_x, origin_y = load_map(args.yaml)
    H, W = img.shape
    print(f"[info] map: {W}x{H} @ {resolution} m/px  origin=({origin_x}, {origin_y})")
    print(f"[info] world extent: "
          f"x=[{origin_x:.2f}, {origin_x + W*resolution:.2f}]  "
          f"y=[{origin_y:.2f}, {origin_y + H*resolution:.2f}]")

    boxes = build_boxes(img, resolution, origin_x, origin_y,
                        args.occ_threshold, args.height)
    print(f"[info] generated {len(boxes)} wall box(es)")

    links = "\n".join(
        BOX_TEMPLATE.format(i=i, **b) for i, b in enumerate(boxes)
    )
    world_xml = WORLD_TEMPLATE.format(links=links)

    with open(args.out, "w") as f:
        f.write(world_xml)
    print(f"[info] wrote {args.out}")


if __name__ == "__main__":
    main()