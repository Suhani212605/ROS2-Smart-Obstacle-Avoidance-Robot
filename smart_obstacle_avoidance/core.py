"""Pure-Python core logic (no ROS imports) so it can be unit-tested anywhere.

* ray_distance / make_scan : the SIMULATED LiDAR (ray casting against virtual obstacles)
* AvoidanceController      : decides forward / stop-and-turn from scan ranges
"""
import math
from collections import namedtuple

WORLD_MIN, WORLD_MAX = 0.0, 11.0          # Turtlesim world is 11 x 11 units

# Virtual obstacles: (x, y, radius). Radius ~0.5 matches the turtle icon size,
# because launch/avoidance.launch.py draws each one as a parked turtle.
OBSTACLES = [(3.0, 3.0, 0.5), (8.0, 5.5, 0.5), (5.0, 8.5, 0.5), (2.5, 7.5, 0.5)]

FOV = math.pi            # 180 degree forward-facing sensor: -90 .. +90 degrees


def ray_distance(x, y, angle, range_max, obstacles=OBSTACLES):
    """Distance from (x, y) along `angle` (world frame) to the nearest obstacle or wall."""
    dx, dy = math.cos(angle), math.sin(angle)
    best = range_max
    for cx, cy, r in obstacles:                      # ray vs circle
        fx, fy = x - cx, y - cy
        b = fx * dx + fy * dy
        disc = b * b - (fx * fx + fy * fy - r * r)
        if disc >= 0.0:
            t = -b - math.sqrt(disc)
            if 0.0 <= t < best:
                best = t
    for d, p in ((dx, x), (dy, y)):                  # ray vs the 4 world walls
        if abs(d) > 1e-9:
            t = ((WORLD_MAX if d > 0 else WORLD_MIN) - p) / d
            if 0.0 <= t < best:
                best = t
    return best


def make_scan(x, y, theta, num_beams=37, range_max=3.0, obstacles=OBSTACLES):
    """Simulated 2D LiDAR. Returns (ranges, angle_min, angle_max, angle_increment).
    Beam angles are relative to robot heading; positive = left (counter-clockwise)."""
    angle_min, angle_max = -FOV / 2, FOV / 2
    inc = (angle_max - angle_min) / (num_beams - 1)
    ranges = [ray_distance(x, y, theta + angle_min + i * inc, range_max, obstacles)
              for i in range(num_beams)]
    return ranges, angle_min, angle_max, inc


Decision = namedtuple("Decision", "linear angular state front_min left_min right_min")


class AvoidanceController:
    """Forward when clear; stop and turn toward the more open side when blocked.
    Hysteresis (stop_dist < clear_dist) stops the robot jittering at the threshold."""

    def __init__(self, stop_dist=1.2, clear_dist=1.6, front_half_angle=math.radians(30),
                 forward_speed=1.5, turn_speed=1.5):
        self.stop_dist, self.clear_dist = stop_dist, clear_dist
        self.front_half = front_half_angle
        self.forward_speed, self.turn_speed = forward_speed, turn_speed
        self.turning, self.turn_dir = False, 1.0

    def decide(self, ranges, angle_min, angle_increment):
        front, left, right = [], [], []
        for i, r in enumerate(ranges):
            r = r if math.isfinite(r) else 1e3          # inf = nothing detected
            a = angle_min + i * angle_increment
            if abs(a) <= self.front_half:
                front.append(r)
            elif a > 0:
                left.append(r)
            else:
                right.append(r)
        f = min(front) if front else 1e3
        l, rt = (min(left) if left else 1e3), (min(right) if right else 1e3)

        if f > (self.clear_dist if self.turning else self.stop_dist):
            self.turning = False
            return Decision(self.forward_speed, 0.0, "CLEAR -> moving forward", f, l, rt)

        if not self.turning:                              # pick a direction once, then keep it
            self.turn_dir = 1.0 if sum(left) / max(len(left), 1) >= sum(right) / max(len(right), 1) else -1.0
            self.turning = True
        side = "left" if self.turn_dir > 0 else "right"
        return Decision(0.0, self.turn_dir * self.turn_speed,
                        f"OBSTACLE at {f:.2f} m -> stop, turning {side}", f, l, rt)
