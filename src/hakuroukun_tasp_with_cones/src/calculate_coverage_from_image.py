#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid

def compute_from_grid(grid_data: np.ndarray, resolution: float):
    """
    Compute coverage from an OccupancyGrid-like array.

    Supports two encodings:
      A) Live simulator encoding:
         cleaned=50, free=0, unknown=-1, obstacles=100
      B) Image-like encoding that appears after saving to PGM and reloading:
         cleaned≈205 (gray), free≈254 (white), blocked=0 (black)
    """
    # Gather unique values (small sample for speed)
    uniq = np.unique(grid_data)

    # Mode A: live OccupancyGrid convention
    if 50 in uniq:
        cleaned_cells = np.sum(grid_data == 50)
        free_cells = np.sum(grid_data == 0)
        mode = "LIVE_OCCUPANCYGRID (cleaned=50, free=0)"
    else:
        # Mode B: image-intensity-like map.
        # Your case: cleaned=205, free=254, blocked=0.
        # We'll be tolerant in case cleaned is slightly off due to conversion.
        cleaned_candidates = [205, 204, 206]
        free_candidates = [254, 253, 255]

        cleaned_cells = sum(np.sum(grid_data == v) for v in cleaned_candidates)
        free_cells = sum(np.sum(grid_data == v) for v in free_candidates)

        # If nothing matches, fall back to a heuristic:
        # consider "valid" = anything not blocked(0) and not unknown(-1),
        # and "cleaned" = mid-gray range [170..230] if present.
        if (cleaned_cells + free_cells) == 0:
            valid_mask = (grid_data != 0) & (grid_data != -1)
            # mid-gray band for cleaned strokes
            cleaned_mask = (grid_data >= 170) & (grid_data <= 230)
            cleaned_cells = int(np.sum(cleaned_mask))
            free_cells = int(np.sum(valid_mask) - cleaned_cells)
            mode = "HEURISTIC_IMAGE (gray≈cleaned, nonzero/non-(-1)=valid)"
        else:
            mode = "IMAGE_INTENSITY (cleaned≈205, free≈254, blocked=0)"

    total_valid_cells = cleaned_cells + free_cells
    area_per_cell = resolution ** 2
    cleaned_area = cleaned_cells * area_per_cell
    total_valid_area = total_valid_cells * area_per_cell
    coverage = (cleaned_area / total_valid_area) * 100 if total_valid_area > 0 else 0.0

    return mode, cleaned_area, total_valid_area, coverage, cleaned_cells, free_cells

def cb(msg: OccupancyGrid):
    grid = np.array(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))
    mode, cleaned_area, total_valid_area, coverage, cleaned_cells, free_cells = compute_from_grid(
        grid, msg.info.resolution
    )

    rospy.loginfo(f"[MODE] {mode}")
    rospy.loginfo(f"Cleaned Area: {cleaned_area:.2f} m^2 (cells={cleaned_cells})")
    rospy.loginfo(f"Total Valid Area: {total_valid_area:.2f} m^2 (cleaned+free cells={cleaned_cells+free_cells})")
    rospy.loginfo(f"Coverage: {coverage:.2f}%")

def main():
    rospy.init_node("coverage_calculator_smart", anonymous=True)
    topic = rospy.get_param("~topic", "/cleaned_map")
    rospy.loginfo(f"Subscribing to {topic}")
    rospy.Subscriber(topic, OccupancyGrid, cb, queue_size=1)
    rospy.spin()

if __name__ == "__main__":
    main()