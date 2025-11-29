import numpy as np
import math

tile_size = 0.585  # meters
lane_width = tile_size / 2  # meters

x_min = 0
x_mid_1 = tile_size
x_mid_2 = 2 * tile_size
x_max = 3 * tile_size

y_min = 0
y_mid_1 = tile_size
y_mid_2 = 2 * tile_size
y_max = 3 * tile_size

# TODO: rewrite using tile pose information (x, y and orientation + relative to map), tile size and tile type
def is_out_of_lane(x, y):
    tile_num = tile_number(x, y)
    if tile_num == -1:
        return True
    if tile_num in [1, 3, 5, 7]:  # vertical and horizontal lanes -> easiest cases
        if tile_num == 1:
            lane_min_x = x_min
            lane_max_x = x_min + lane_width
            if not (lane_min_x <= x <= lane_max_x):
                return True
        elif tile_num == 3:
            lane_min_y = y_min
            lane_max_y = y_min + lane_width
            if not (lane_min_y <= y <= lane_max_y):
                return True
        elif tile_num == 5:
            lane_min_y = y_mid_2 + lane_width
            lane_max_y = y_max
            if not (lane_min_y <= y <= lane_max_y):
                return True
        elif tile_num == 7:
            lane_min_x = x_mid_2 + lane_width
            lane_max_x = x_max
            if not (lane_min_x <= x <= lane_max_x):
                return True
    else:  # corner tiles -> assuming tiles are quarter circles
        center_x = center_y = None
        if tile_num == 0:
            center_x = x_mid_1
            center_y = y_mid_1
        elif tile_num == 2:
            center_x = x_mid_1
            center_y = y_mid_2
        elif tile_num == 6:
            center_x = x_mid_2
            center_y = y_mid_1
        elif tile_num == 8:
            center_x = x_mid_2
            center_y = y_mid_2
        #print('############################################################')
        #print(f"(x, y) = ({round(x,2)}, {round(y, 2)})")
        #print(f"center_x =", center_x)
        #print(f"center_y =", center_y)
        #print(f"x**2 + y**2 =", (x - center_x) ** 2 + (y - center_y) ** 2)
        #print("tile_size ** 2 =", tile_size ** 2)
        #print("lane_width ** 2 =", lane_width ** 2)
        #print((tile_size ** 2) < ((x - center_x) ** 2 + (y - center_y) ** 2) < (lane_width) ** 2)

        r_square = (x - center_x) ** 2 + (y - center_y) ** 2
        if (tile_size ** 2) < r_square or r_square < (lane_width) ** 2:
                return True    
    return False

# TODO: compute distance from lane center (d) and orientation error (theta)
def compute_d(x, y):
    if is_out_of_lane(x, y):
        return -1
    
    # assuming the duckiebot is in line
    tile_num = tile_number(x, y)
    if tile_num in [1, 3, 5, 7]:  # vertical and horizontal lanes -> easiest cases
        if tile_num == 1: # vertical tile, distance in x
            center_road = x_min + lane_width / 2
            return abs(x - center_road)
        elif tile_num == 3:
            center_road = y_min + lane_width / 2
            return abs(y - center_road)
        elif tile_num == 5:
            center_road = y_mid_2 + 3 * lane_width / 2
            return abs(y - center_road)
        elif tile_num == 7:
            center_road = x_mid_2 + 3 * lane_width / 2
            return abs(x - center_road)
    else:
        center_x = center_y = None
        if tile_num == 0:
            center_x = x_mid_1
            center_y = y_mid_1
        elif tile_num == 2:
            center_x = x_mid_1
            center_y = y_mid_2
        elif tile_num == 6:
            center_x = x_mid_2
            center_y = y_mid_1
        elif tile_num == 8:
            center_x = x_mid_2
            center_y = y_mid_2
        
        r = np.sqrt((x - center_x) ** 2 + (y - center_y) ** 2)
        return abs(r - 3 * lane_width / 2)
    
def compute_theta(x, y, yaw):
    def wrap_to_pi(angle):
        """Wrap any angle to [-pi, pi]."""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    # TODO: implement termination if theta is greater or less than pi / 2
    if is_out_of_lane(x, y):
        return 1000

    tile_num = tile_number(x, y)

    # Straight tiles: fixed desired heading.
    if tile_num in [1, 3, 5, 7]:
        if tile_num == 1:
            desired_heading = -math.pi / 2
        elif tile_num == 3:
            desired_heading = 0
        elif tile_num == 5:
            desired_heading = math.pi
        elif tile_num == 7:
            desired_heading = math.pi / 2
        return wrap_to_pi(desired_heading - yaw)

    # Corner tiles: heading follows the tangent of the quarter-circle centerline (CCW loop).
    if tile_num == 0:
        center_x = x_mid_1
        center_y = y_mid_1
    elif tile_num == 2:
        center_x = x_mid_1
        center_y = y_mid_2
    elif tile_num == 6:
        center_x = x_mid_2
        center_y = y_mid_1
    elif tile_num == 8:
        center_x = x_mid_2
        center_y = y_mid_2
    else:
        # Should be unreachable if tile_number() is consistent.
        return 1000

    phi = math.atan2(y - center_y, x - center_x)
    # For tile 0 the atan2 range is [-pi, -pi/2]; unwrap so phi increases along the CCW path.
    if tile_num == 0 and phi < 0:
        phi += 2 * math.pi

    desired_heading = phi + math.pi / 2  # tangent for CCW traversal
    return wrap_to_pi(desired_heading - yaw)

def is_out_of_map(x, y):
    if x < x_min or x > x_max or y < y_min or y > y_max:
        return True
    if (y_mid_1 < y < y_mid_2) and (x_mid_1 < x < x_mid_2): # in the hole
        return True
    return False

def tile_number(x, y):
    def in_tile_0_0(x, y):
        return (x_min <= x <= x_mid_1) and (y_min <= y <= y_mid_1)

    def in_tile_0_1(x, y):
        return (x_min <= x <= x_mid_1) and (y_mid_1 <= y <= y_mid_2)

    def in_tile_0_2(x, y):
        return (x_min <= x <= x_mid_1) and (y_mid_2 <= y <= y_max)

    def in_tile_1_0(x, y):
        return (x_mid_1 <= x <= x_mid_2) and (y_min <= y <= y_mid_1)

    def in_tile_1_2(x, y):
        return (x_mid_1 <= x <= x_mid_2) and (y_mid_2 <= y <= y_max)

    def in_tile_2_0(x, y):
        return (x_mid_2 <= x <= x_max) and (y_min <= y <= y_mid_1)

    def in_tile_2_1(x, y):
        return (x_mid_2 <= x <= x_max) and (y_mid_1 <= y <= y_mid_2)

    def in_tile_2_2(x, y):
        return (x_mid_2 <= x <= x_max) and (y_mid_2 <= y <= y_max)
    
    if is_out_of_map(x, y):
        return -1
    if in_tile_0_0(x, y):
        return 0
    if in_tile_0_1(x, y):
        return 1
    if in_tile_0_2(x, y):
        return 2
    if in_tile_1_0(x, y):
        return 3
    if in_tile_1_2(x, y):        
        return 5
    if in_tile_2_0(x, y):
        return 6
    if in_tile_2_1(x, y):
        return 7
    if in_tile_2_2(x, y):
        return 8
    raise RuntimeError("Unreachable")
