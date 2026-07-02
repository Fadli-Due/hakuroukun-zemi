#!/usr/bin/env python3
"""
paint_polygons.py
-----------------
Interactive polygon painter for ROS occupancy-grid PGM maps.
 
Use this to convert a Gmapping output (walls + cone-dots) into a planning
map where restricted areas formed by cones are painted as solid occupied
polygons -- matching the semantic intent of Nguyen Cong Tai's TASP
restricted-area approach.
 
Usage:
    python3 paint_polygons.py <input_pgm> <output_pgm>
 
Controls (in the matplotlib window):
    Left-click     : add a vertex to the current polygon
    Enter          : close the current polygon, start a new one
    u              : undo last vertex (or remove last completed polygon)
    q              : save and quit
    [matplotlib's built-in pan / zoom in the toolbar work as usual]
 
Output:
    - <output_pgm>          painted PGM (P5 binary, no anti-aliasing)
    - Pixel-value histogram printed to confirm only {0, 205, 254} present
    - A companion .yaml is NOT written; copy your existing yaml and just
      change the `image:` field to point at the new pgm.
"""
 
import argparse
import sys
 
import numpy as np
from PIL import Image, ImageDraw
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon
 
 
UNKNOWN = 205   # ROS map_server "unknown" gray
OCCUPIED = 0    # what we paint with
FREE = 254
 
 
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input_pgm",  help="Path to the source Gmapping PGM")
    ap.add_argument("output_pgm", help="Path to save the painted PGM")
    args = ap.parse_args()
 
    img = Image.open(args.input_pgm)
    arr = np.array(img)
    print(f"Loaded {args.input_pgm}")
    print(f"  shape   : {arr.shape}")
    print(f"  dtype   : {arr.dtype}")
    print(f"  range   : {arr.min()} .. {arr.max()}")
 
    # Auto-zoom matplotlib view to the content (anything not UNKNOWN gray)
    known = arr != UNKNOWN
    if not known.any():
        print("ERROR: input PGM is entirely unknown-gray; nothing to paint.")
        sys.exit(1)
    rows = np.where(known.any(axis=1))[0]
    cols = np.where(known.any(axis=0))[0]
    pad = 30
    r0, r1 = max(0, rows[0] - pad), min(arr.shape[0] - 1, rows[-1] + pad)
    c0, c1 = max(0, cols[0] - pad), min(arr.shape[1] - 1, cols[-1] + pad)
 
    fig, ax = plt.subplots(figsize=(11, 11))
    ax.imshow(arr, cmap="gray", vmin=0, vmax=255, interpolation="nearest")
    ax.set_xlim(c0, c1)
    ax.set_ylim(r1, r0)   # reversed: image y grows downward
    ax.set_title("Click vertices around the cones. Enter=close polygon. u=undo. q=save & quit.")
 
    polygons = []   # list of list of (x, y) pixel coords
    current = []
    artists = []
 
    def redraw():
        for a in artists:
            a.remove()
        artists.clear()
        for poly in polygons:
            if len(poly) >= 3:
                p = MplPolygon(poly, closed=True,
                               facecolor="red", alpha=0.35, edgecolor="red", lw=1.5)
                ax.add_patch(p)
                artists.append(p)
        if current:
            xs, ys = zip(*current)
            line, = ax.plot(xs, ys, "y-o", markersize=5, lw=1.2)
            artists.append(line)
        fig.canvas.draw_idle()
 
    def on_click(event):
        if event.inaxes != ax:
            return
        if event.button == 1:
            current.append((event.xdata, event.ydata))
            print(f"  + vertex ({event.xdata:7.1f}, {event.ydata:7.1f})  "
                  f"[current polygon has {len(current)} verts]")
            redraw()
 
    def on_key(event):
        if event.key == "enter":
            if len(current) >= 3:
                polygons.append(list(current))
                print(f"  >>> Closed polygon #{len(polygons)} "
                      f"with {len(current)} vertices")
                current.clear()
                redraw()
            else:
                print("  Need at least 3 vertices to close a polygon.")
        elif event.key == "u":
            if current:
                v = current.pop()
                print(f"  - removed vertex ({v[0]:.1f}, {v[1]:.1f})")
            elif polygons:
                p = polygons.pop()
                print(f"  - removed last polygon ({len(p)} verts)")
            redraw()
        elif event.key == "q":
            plt.close(fig)
 
    fig.canvas.mpl_connect("button_press_event", on_click)
    fig.canvas.mpl_connect("key_press_event", on_key)
 
    print()
    print("Controls:")
    print("  Left-click : add vertex")
    print("  Enter      : close current polygon, start new one")
    print("  u          : undo last vertex (or pop last polygon)")
    print("  q          : save & quit")
    print()
    print("Tip: use the matplotlib toolbar magnifier to zoom in on the cones.")
    print()
    plt.show()
 
    if not polygons:
        print("No polygons drawn -- nothing to save.")
        sys.exit(0)
 
    # Paint with PIL ImageDraw -- crisp pixels, no anti-aliasing
    out = Image.fromarray(arr.copy())
    draw = ImageDraw.Draw(out)
    for poly in polygons:
        verts = [(int(round(x)), int(round(y))) for x, y in poly]
        draw.polygon(verts, fill=OCCUPIED, outline=OCCUPIED)
 
    out.save(args.output_pgm)
    out_arr = np.array(out)
    print()
    print(f"Saved {args.output_pgm}")
    print(f"  polygons painted : {len(polygons)}")
    print(f"  shape            : {out_arr.shape}")
 
    vals, counts = np.unique(out_arr, return_counts=True)
    print()
    print("Output pixel-value histogram:")
    for v, c in zip(vals, counts):
        pct = 100 * c / out_arr.size
        label = ""
        if v == OCCUPIED:
            label = "(occupied / black)"
        elif v == UNKNOWN:
            label = "(unknown / gray)"
        elif v in (254, 255):
            label = "(free / white)"
        else:
            label = "(intermediate -- map_server will treat as unknown)"
        print(f"  {v:3d}: {c:>10d}  ({pct:5.2f}%) {label}")
 
    intermediates = [v for v in vals if v not in (OCCUPIED, UNKNOWN, 254, 255)]
    if intermediates:
        print()
        print("WARNING: intermediate pixel values present:", intermediates)
        print("These were already in the input PGM (Gmapping quantization).")
        print("They are typically a thin border around cones and walls and")
        print("are not painted-polygon artifacts. Safe to ignore.")
 
 
if __name__ == "__main__":
    main()
 