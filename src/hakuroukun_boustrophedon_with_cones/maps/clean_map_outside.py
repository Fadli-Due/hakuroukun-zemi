#!/usr/bin/env python3
"""
clean_map_outside.py
Flood-fill unknown (gray ~205) pixels reachable from image borders → black (occupied).
This fixes the inflated coverage denominator without touching anything inside the warehouse.

ROS OccupancyGrid PGM convention:
  254 (white)  = free
  0   (black)  = occupied
  205 (gray)   = unknown

Usage:
  python3 clean_map_outside.py <input.pgm> [output.pgm]
  If output is omitted, writes to <input_stem>_cleaned.pgm
"""

import sys
import struct
from collections import deque


def read_pgm(path):
    """Read a P5 (binary) PGM file. Returns (width, height, maxval, pixels as bytearray)."""
    with open(path, "rb") as f:
        # Read magic
        magic = f.readline().strip()
        if magic != b"P5":
            raise ValueError(f"Expected P5 PGM, got {magic}")

        # Skip comments
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()

        # Width and height
        w, h = map(int, line.split())

        # Max value
        maxval = int(f.readline().strip())

        # Pixel data
        if maxval < 256:
            pixels = bytearray(f.read(w * h))
        else:
            # 16-bit PGM — unlikely for ROS maps but handle it
            raw = f.read(w * h * 2)
            pixels = bytearray(struct.unpack(f">{w*h}H", raw))

    return w, h, maxval, pixels


def write_pgm(path, w, h, maxval, pixels):
    """Write a P5 (binary) PGM file."""
    with open(path, "wb") as f:
        f.write(f"P5\n{w} {h}\n{maxval}\n".encode())
        f.write(bytes(pixels))


def flood_fill_outside(w, h, pixels, unknown_lo=195, unknown_hi=215):
    """
    BFS from every border pixel. Walk through any pixel in the unknown range
    [unknown_lo, unknown_hi] and mark it as occupied (0). This eliminates all
    'outside-the-warehouse' unknown space while preserving interior unknowns.
    """
    visited = bytearray(w * h)  # 0 = unvisited
    queue = deque()

    # Seed: every border pixel that is in the unknown range
    for x in range(w):
        for y_row in (0, h - 1):
            idx = y_row * w + x
            if unknown_lo <= pixels[idx] <= unknown_hi and not visited[idx]:
                visited[idx] = 1
                queue.append(idx)

    for y in range(h):
        for x_col in (0, w - 1):
            idx = y * w + x_col
            if unknown_lo <= pixels[idx] <= unknown_hi and not visited[idx]:
                visited[idx] = 1
                queue.append(idx)

    # BFS
    filled = 0
    while queue:
        idx = queue.popleft()
        pixels[idx] = 0  # Set to occupied
        filled += 1

        y, x = divmod(idx, w)
        for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h:
                nidx = ny * w + nx
                if not visited[nidx] and unknown_lo <= pixels[nidx] <= unknown_hi:
                    visited[nidx] = 1
                    queue.append(nidx)

    return filled


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 clean_map_outside.py <input.pgm> [output.pgm]")
        sys.exit(1)

    in_path = sys.argv[1]
    if len(sys.argv) >= 3:
        out_path = sys.argv[2]
    else:
        stem = in_path.rsplit(".", 1)[0]
        out_path = f"{stem}_cleaned.pgm"

    print(f"Reading: {in_path}")
    w, h, maxval, pixels = read_pgm(in_path)
    print(f"  Dimensions: {w} x {h},  maxval: {maxval}")

    # Stats before
    n_free = sum(1 for p in pixels if p > 220)
    n_occupied = sum(1 for p in pixels if p < 30)
    n_unknown = sum(1 for p in pixels if 195 <= p <= 215)
    print(f"  Before — free: {n_free}, occupied: {n_occupied}, unknown: {n_unknown}")

    filled = flood_fill_outside(w, h, pixels)
    print(f"  Flood-filled {filled} outside-unknown pixels → occupied")

    # Stats after
    n_free2 = sum(1 for p in pixels if p > 220)
    n_occupied2 = sum(1 for p in pixels if p < 30)
    n_unknown2 = sum(1 for p in pixels if 195 <= p <= 215)
    print(f"  After  — free: {n_free2}, occupied: {n_occupied2}, unknown: {n_unknown2}")

    write_pgm(out_path, w, h, maxval, pixels)
    print(f"Saved:   {out_path}")

    # Quick sanity: estimate total valid area
    resolution = 0.05  # default ROS map resolution (m/pixel)
    valid_cells = n_free2 + n_unknown2  # remaining free + any interior unknowns
    area_m2 = valid_cells * (resolution ** 2)
    print(f"\n  Estimated valid area at {resolution} m/px resolution: {area_m2:.1f} m²")
    print(f"  (Tai's thesis total valid area was 718.18 m²)")


if __name__ == "__main__":
    main()