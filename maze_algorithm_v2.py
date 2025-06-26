# =========================
# 📌 Features added:
# - Encoder-based movement for consistent tile steps
# - IMU gyro calibration and 90°/180° rotation
# - Ultrasonic sensors (front/left/right) for wall detection
# - Depth-first maze exploration using DFS
# - ✅ NEW: Tile-by-tile centering & heading correction after each move
# =========================

from gpiozero import DistanceSensor
import time
from control import move_forward, stop, robot_rotate_left, robot_rotate_right, calibrate_gyro

# =========================
# 🔊 Ultrasonic Setup (GPIO Zero)
# =========================
ULTRASONIC_SENSORS = {
    "front": DistanceSensor(echo=3, trigger=2),
    "left": DistanceSensor(echo=17, trigger=4),
    "right": DistanceSensor(echo=22, trigger=27),
}

def read_distance(direction):
    dist = ULTRASONIC_SENSORS[direction].distance * 100  # convert to cm
    return dist

def check_wall(direction):
    dist = read_distance(direction)
    if direction == "front":
        return dist < 5
    else:
        return dist < 10

# =========================
# 🧹 Maze Logic
# =========================
DIRECTIONS = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # N, E, S, W
direction_labels = ["N", "E", "S", "W"]

def turn_to(current_dir, target_dir):
    turns = (target_dir - current_dir) % 4
    if turns == 1:
        robot_rotate_right(90)
    elif turns == 2:
        robot_rotate_right(90)
        robot_rotate_right(90)
    elif turns == 3:
        robot_rotate_left(90)
    return target_dir

def direction_to(from_x, from_y, to_x, to_y):
    dx = to_x - from_x
    dy = to_y - from_y
    for d, (ddx, ddy) in enumerate(DIRECTIONS):
        if (dx, dy) == (ddx, ddy):
            return d
    return None

def center_and_straighten():
    left = read_distance("left")
    right = read_distance("right")
    tolerance = 1.0

    if abs(left - right) > tolerance:
        if left < right:
            robot_rotate_left(3)
        else:
            robot_rotate_right(3)
        stop()

# =========================
# 🧭 Main DFS Navigation
# =========================

def main():
    calibrate_gyro()

    visited = set()
    path_stack = []

    x, y = 0, 0
    dir = 0  # Start facing North

    visited.add((x, y))
    path_stack.append((x, y))
    print("Starting maze exploration from (0,0)...")

    while path_stack:
        found = False

        for d in range(4):
            dx, dy = DIRECTIONS[d]
            nx, ny = x + dx, y + dy
            if (nx, ny) not in visited and not check_wall_by_relative(dir, d):
                print(f"At ({x},{y}) facing {direction_labels[dir]} → Moving to ({nx},{ny}) dir {direction_labels[d]}")
                dir = turn_to(dir, d)
                move_forward(30)
                center_and_straighten()
                x, y = nx, ny
                visited.add((x, y))
                path_stack.append((x, y))
                found = True
                break

        if not found:
            print(f"Backtracking from ({x},{y})")
            path_stack.pop()
            if not path_stack:
                break
            tx, ty = path_stack[-1]
            back_dir = direction_to(x, y, tx, ty)
            dir = turn_to(dir, back_dir)
            move_forward(30)
            center_and_straighten()
            x, y = tx, ty

    print("✅ Maze fully explored. Robot stopped.")
    stop()

def check_wall_by_relative(current_dir, target_dir):
    rel = (target_dir - current_dir) % 4
    if rel == 0:
        return check_wall("front")
    elif rel == 1:
        return check_wall("right")
    elif rel == 2:
        return True  # never move backward
    elif rel == 3:
        return check_wall("left")

if __name__ == "__main__":
    main()
