#!/usr/bin/env python3
"""Rotate a ROS map PGM around its image center and update YAML origin
so that world (GPS) coordinates are preserved."""

import cv2
import numpy as np
import math
import yaml

# --- inputs ---
in_pgm    = 'df_area_outline_painted.pgm'
in_yaml   = 'df_area_outline_painted.yaml'
out_pgm   = 'df_area_outline_rotated.pgm'
out_yaml  = 'df_area_outline_rotated.yaml'
alpha_deg = 8.3    # CCW rotation to apply. Positive = counterclockwise.

# --- load YAML ---
with open(in_yaml) as f:
    meta = yaml.safe_load(f)
res = float(meta['resolution'])
ox_old, oy_old, yaw_old = meta['origin']
assert abs(yaw_old) < 1e-9, "old yaw must be 0; adapt formula otherwise"

# --- load and rotate PGM ---
img = cv2.imread(in_pgm, cv2.IMREAD_GRAYSCALE)
if img is None:
    raise SystemExit(f"could not read {in_pgm}")
H, W = img.shape

# 205 = unknown-gray, matches the fill outside your painted polygon
M = cv2.getRotationMatrix2D((W/2.0, H/2.0), alpha_deg, 1.0)
rotated = cv2.warpAffine(img, M, (W, H),
                         flags=cv2.INTER_NEAREST,
                         borderMode=cv2.BORDER_CONSTANT,
                         borderValue=205)
cv2.imwrite(out_pgm, rotated)

# --- corrected origin (see derivation in earlier message) ---
a = math.radians(alpha_deg)
c, s = math.cos(-a), math.sin(-a)
xi_c  = W * res / 2.0
eta_c = H * res / 2.0
dx = (c - 1) * xi_c + (-s) * eta_c
dy = ( s   ) * xi_c + (c - 1) * eta_c
ox_new  = ox_old + dx
oy_new  = oy_old + dy
yaw_new = -a

# --- write new YAML ---
with open(out_yaml, 'w') as f:
    f.write(f"image: {out_pgm}\n")
    f.write(f"resolution: {res}\n")
    f.write(f"origin: [{ox_new:.4f}, {oy_new:.4f}, {yaw_new:.6f}]\n")
    f.write(f"negate: {meta.get('negate', 0)}\n")
    f.write(f"occupied_thresh: {meta.get('occupied_thresh', 0.65)}\n")
    f.write(f"free_thresh: {meta.get('free_thresh', 0.196)}\n")

print(f"wrote {out_pgm} and {out_yaml}")
print(f"new origin: [{ox_new:.4f}, {oy_new:.4f}, {yaw_new:.6f}]")