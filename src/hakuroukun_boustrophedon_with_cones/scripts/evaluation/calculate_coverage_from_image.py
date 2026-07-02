#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32

# Latched valid area published by offline_coverage_planner.py after BCD
# inflation + CC-filter. None until the planner has run.
_bcd_valid_area_m2 = None

def valid_area_cb(msg: Float32):
    global _bcd_valid_area_m2
    _bcd_valid_area_m2 = msg.data
    rospy.loginfo("[cov] BCD valid area received: %.2f m^2" % _bcd_valid_area_m2)

def compute_from_grid(grid_data: np.ndarray, resolution: float):
    """
    Compute coverage from an OccupancyGrid-like array.

    Supports two encodings:
      A) Live simulator encoding:
         cleaned=50, free=0, unknown=-1, obstacles=100
      B) Image-like encoding that appears after saving to PGM and reloading:
         cleaned≈205 (gray), free≈254 (white), blocked=0 (black)

    The valid-area denominator is taken from /bcd_valid_area_m2 (published by
    the BCD planner) when available.  This ensures valid area = the
    EDT-inflated free space the planner actually planned on (white area only,
    obstacle clearance band excluded), matching sensei's definition.
    Falls back to cleaned+free cell count if the planner topic hasn't arrived.
    """
    uniq = np.unique(grid_data)

    # ── Mode A: live OccupancyGrid convention ────────────────────────────────
    if 50 in uniq:
        cleaned_cells = int(np.sum(grid_data == 50))
        free_cells    = int(np.sum(grid_data == 0))
        mode = "LIVE_OCCUPANCYGRID (cleaned=50, free=0)"

    # ── Mode B: image-intensity-like map ─────────────────────────────────────
    else:
        cleaned_candidates = [205, 204, 206]
        free_candidates    = [254, 253, 255]

        cleaned_cells = sum(int(np.sum(grid_data == v)) for v in cleaned_candidates)
        free_cells    = sum(int(np.sum(grid_data == v)) for v in free_candidates)

        if (cleaned_cells + free_cells) == 0:
            valid_mask    = (grid_data != 0) & (grid_data != -1)
            cleaned_mask  = (grid_data >= 170) & (grid_data <= 230)
            cleaned_cells = int(np.sum(cleaned_mask))
            free_cells    = int(np.sum(valid_mask) - cleaned_cells)
            mode = "HEURISTIC_IMAGE (gray≈cleaned, nonzero/non-(-1)=valid)"
        else:
            mode = "IMAGE_INTENSITY (cleaned≈205, free≈254, blocked=0)"

    area_per_cell = resolution ** 2
    cleaned_area  = cleaned_cells * area_per_cell

    # ── Denominator: prefer BCD-inflated area, fall back to cell count ───────
    if _bcd_valid_area_m2 is not None and _bcd_valid_area_m2 > 0:
        total_valid_area = _bcd_valid_area_m2
        denom_source     = "BCD-inflated"
    else:
        total_valid_area = (cleaned_cells + free_cells) * area_per_cell
        denom_source     = "fallback (raw free cells)"

    coverage = (cleaned_area / total_valid_area * 100.0) if total_valid_area > 0 else 0.0

    return mode, cleaned_area, total_valid_area, coverage, cleaned_cells, free_cells, denom_source


def cb(msg: OccupancyGrid):
    grid = np.array(msg.data, dtype=np.int16).reshape(
        (msg.info.height, msg.info.width))
    mode, cleaned_area, total_valid_area, coverage, cleaned_cells, free_cells, denom_source = \
        compute_from_grid(grid, msg.info.resolution)

    rospy.loginfo("[MODE] %s" % mode)
    rospy.loginfo("Cleaned Area: %.2f m^2 (cells=%d)" % (cleaned_area, cleaned_cells))
    rospy.loginfo("Total Valid Area: %.2f m^2 [%s]" % (total_valid_area, denom_source))
    rospy.loginfo("Coverage: %.2f%%" % coverage)


def main():
    rospy.init_node("coverage_calculator_smart", anonymous=True)

    # Subscribe to the BCD valid area FIRST (latched) so we have the
    # denominator before the first /cleaned_map message arrives.
    rospy.Subscriber("/bcd_valid_area_m2", Float32, valid_area_cb, queue_size=1)

    topic = rospy.get_param("~topic", "/cleaned_map")
    rospy.loginfo("Subscribing to %s" % topic)
    rospy.Subscriber(topic, OccupancyGrid, cb, queue_size=1)

    rospy.spin()


if __name__ == "__main__":
    main()