#!/usr/bin/env python3
import heapq
import rospy
import math

class SimpleOccupancyGrid:
    """
    A lightweight wrapper for storing map info. Typically, you'd get these fields from:
      - occupancy_grid.info.width
      - occupancy_grid.info.height
      - occupancy_grid.info.resolution
      - occupancy_grid.info.origin.position.x
      - occupancy_grid.info.origin.position.y
      - occupancy_grid.data (list of int: 0=free, 100=occ, -1=unknown)
    """
    def __init__(self, width, height, resolution, origin_x, origin_y, data):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.data = data  # list length width*height


def world_to_grid(ogrid, wx, wy):
    gx = int((wx - ogrid.origin_x) / ogrid.resolution)
    gy = int((wy - ogrid.origin_y) / ogrid.resolution)
    if gx < 0 or gx >= ogrid.width or gy < 0 or gy >= ogrid.height:
        return None
    return (gx, gy)


def grid_to_world(ogrid, gx, gy):
    wx = ogrid.origin_x + (gx + 0.5) * ogrid.resolution
    wy = ogrid.origin_y + (gy + 0.5) * ogrid.resolution
    return (wx, wy)


def is_occupied(ogrid, gx, gy, treat_unknown_as_occupied=False):
    if gx < 0 or gx >= ogrid.width or gy < 0 or gy >= ogrid.height:
        return True
    idx = gy * ogrid.width + gx
    val = ogrid.data[idx]  # 0=free, 100=occupied, -1=unknown
    if val < 0:
        return treat_unknown_as_occupied
    return (val >= 50)


def get_neighbors(gx, gy, connectivity=8):
    if connectivity == 4:
        return [(gx-1, gy), (gx+1, gy), (gx, gy-1), (gx, gy+1)]
    else:
        return [
            (gx-1, gy), (gx+1, gy), (gx, gy-1), (gx, gy+1),
            (gx-1, gy-1), (gx-1, gy+1), (gx+1, gy-1), (gx+1, gy+1)
        ]


def heuristic(a, b, diag_ok=True):
    (ax, ay) = a
    (bx, by) = b
    dx = abs(ax - bx)
    dy = abs(ay - by)
    if diag_ok:
        return max(dx, dy)      # Chebyshev (good for 8-connect)
    else:
        return dx + dy          # Manhattan (good for 4-connect)


def astar_plan(ogrid, wx_start, wy_start, wx_goal, wy_goal,
               connectivity=8, max_expansions=200000):
    """
    Simple A* from start(world) -> goal(world).
    Returns list[(wx, wy)] or [].
    Returns "GOAL_OCCUPIED" if goal cell blocked (including unknown if treat_unknown_as_occupied=True).
    max_expansions prevents worst-case infinite-ish runtimes on big maps.
    """

    # 1) world -> grid
    start_g = world_to_grid(ogrid, wx_start, wy_start)
    goal_g  = world_to_grid(ogrid, wx_goal, wy_goal)
    if not start_g or not goal_g:
        rospy.logwarn("[A*] Start or goal out of map bounds.")
        return []

    if is_occupied(ogrid, start_g[0], start_g[1], treat_unknown_as_occupied=True):
        rospy.logwarn("[A*] Start cell is occupied/unknown!")
        return []

    if is_occupied(ogrid, goal_g[0], goal_g[1], treat_unknown_as_occupied=True):
        rospy.logwarn("[A*] Goal cell is occupied/unknown!")
        return "GOAL_OCCUPIED"

    diag_ok = (connectivity == 8)

    # 2) open/closed
    open_set = []
    heapq.heappush(open_set, (0.0, start_g))
    came_from = {}
    g_score = {start_g: 0.0}
    visited = set()

    expansions = 0

    while open_set:
        expansions += 1
        if expansions > max_expansions:
            rospy.logwarn(f"[A*] abort: too many expansions ({max_expansions}).")
            return []

        _, current = heapq.heappop(open_set)
        if current in visited:
            continue
        visited.add(current)

        if current == goal_g:
            return reconstruct_path(came_from, current, ogrid)

        cx, cy = current

        for nx, ny in get_neighbors(cx, cy, connectivity):
            neighbor = (nx, ny)

            if neighbor in visited:
                continue

            # occupied/unknown treated as blocked
            if is_occupied(ogrid, nx, ny, treat_unknown_as_occupied=True):
                continue

            dx = nx - cx
            dy = ny - cy

            # corner-cutting prevention for diagonal moves
            if diag_ok and abs(dx) == 1 and abs(dy) == 1:
                if is_occupied(ogrid, cx + dx, cy, True) or is_occupied(ogrid, cx, cy + dy, True):
                    continue

            # move cost
            cost_move = 1.0 if (abs(dx) + abs(dy) == 1) else math.sqrt(2.0)

            tentative_g = g_score[current] + cost_move
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                h = heuristic(neighbor, goal_g, diag_ok=diag_ok)
                f = tentative_g + h
                heapq.heappush(open_set, (f, neighbor))

    rospy.logwarn("[A*] No path found.")
    return []


def reconstruct_path(came_from, current, ogrid):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()

    world_path = []
    for (gx, gy) in path:
        wx, wy = grid_to_world(ogrid, gx, gy)
        world_path.append((wx, wy))
    return world_path