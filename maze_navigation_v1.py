# maze_navigation_v1.py

from robot_controller import RobotController
from ultrasonic_sensor import measure_distance_front, measure_distance_left, measure_distance_right, cleanup
import time

# Direction constants: 0 = N, 1 = E, 2 = S, 3 = W
DIRECTIONS = [(0, 1), (1, 0), (0, -1), (-1, 0)]

# Rotation function matching simulation logic
def turn_to_real_robot(current_dir, target_dir, robot):
    turns = (target_dir - current_dir) % 4
    if turns == 1:
        robot.robot_rotate_right(90)
    elif turns == 2:
        robot.robot_rotate_right(180)
    elif turns == 3:
        robot.robot_rotate_left(90)
    return target_dir

# Direction helper
def direction_to(from_x, from_y, to_x, to_y):
    dx = to_x - from_x
    dy = to_y - from_y
    for d, (ddx, ddy) in enumerate(DIRECTIONS):
        if (dx, dy) == (ddx, ddy):
            return d
    return None

# Wall detection based on ultrasonic readings
def is_wall_in_direction(current_dir, target_dir):
    rel = (target_dir - current_dir) % 4
    front = measure_distance_front()
    left = measure_distance_left()
    right = measure_distance_right()

    if rel == 0:
        return front < 6
    elif rel == 1:
        return right < 10
    elif rel == 2:
        # No rear sensor, assume backtracking allowed
        return False
    elif rel == 3:
        return left < 10

# Main maze navigation
robot = RobotController()
robot.calibrate_gyro()

visited = set()
path_stack = []

x, y = 0, 0
direction = 0  # Facing North
visited.add((x, y))
path_stack.append((x, y))

try:
    while path_stack:
        found = False

        for d in range(4):  # Try all directions
            nx, ny = x + DIRECTIONS[d][0], y + DIRECTIONS[d][1]
            if (nx, ny) not in visited and not is_wall_in_direction(direction, d):
                direction = turn_to_real_robot(direction, d, robot)
                robot.move_forward(30)  # Move one cell
                x, y = nx, ny
                visited.add((x, y))
                path_stack.append((x, y))
                found = True
                break

        if not found:
            # Backtracking
            path_stack.pop()
            if not path_stack:
                break
            tx, ty = path_stack[-1]
            back_dir = direction_to(x, y, tx, ty)
            direction = turn_to_real_robot(direction, back_dir, robot)
            robot.move_forward(30)
            x, y = tx, ty

finally:
    cleanup()  # Release GPIO safely
    print("\n✅ Maze exploration complete.")
