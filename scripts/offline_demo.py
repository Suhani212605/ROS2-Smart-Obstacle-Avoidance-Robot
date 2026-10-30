"""Offline simulation (NO ROS needed): runs the SAME core logic as the ROS 2 nodes with a simple
unicycle motion model, prints a log, checks for collisions and saves docs/simulation_demo.png.
This is a stand-in for the Turtlesim window, not a screenshot of it."""
import math, os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from smart_obstacle_avoidance.core import (AvoidanceController, make_scan, OBSTACLES,
                                           WORLD_MIN, WORLD_MAX)

DT, T_END, ROBOT_R = 0.02, 60.0, 0.3
x, y, th = 5.544, 5.544, 0.0                  # same start pose as Turtlesim's turtle1
ctrl, cmd = AvoidanceController(), (0.0, 0.0)
path, log, last, min_clear, t = [], [], None, 1e9, 0.0
next_scan, next_log = 0.0, 0.0
while t < T_END:
    if t >= next_scan:                        # 10 Hz LiDAR + controller
        ranges, a0, _, inc = make_scan(x, y, th)
        d = ctrl.decide(ranges, a0, inc)
        cmd = (d.linear, d.angular)
        next_scan += 0.1
        if last is None or d.state[:5] != last[:5] or t >= next_log:
            log.append(f"t={t:5.1f}s pos=({x:5.2f},{y:5.2f}) front={d.front_min:4.2f}m "
                       f"left={d.left_min:4.2f}m right={d.right_min:4.2f}m | v={d.linear:.1f} w={d.angular:+.1f} | {d.state}")
            next_log = t + 2.0
        last = d.state
    x += cmd[0] * math.cos(th) * DT
    y += cmd[0] * math.sin(th) * DT
    th += cmd[1] * DT
    path.append((x, y, cmd[0] > 0))
    min_clear = min([math.hypot(x - cx, y - cy) - r - ROBOT_R for cx, cy, r in OBSTACLES]
                    + [x - WORLD_MIN - ROBOT_R, WORLD_MAX - x - ROBOT_R, y - WORLD_MIN - ROBOT_R, WORLD_MAX - y - ROBOT_R]
                    + [min_clear])
    t += DT

summary = f"SIMULATED run: {T_END:.0f}s, minimum clearance to any obstacle/wall = {min_clear:.2f} m -> " \
          + ("NO COLLISION" if min_clear > 0 else "COLLISION")
out = os.path.join(os.path.dirname(__file__), '..', 'docs')
with open(os.path.join(out, 'sample_output.txt'), 'w') as f:
    f.write("# Output of scripts/offline_demo.py (simulated, no ROS)\n" + "\n".join(log) + "\n" + summary + "\n")
print("\n".join(log[:12]), "...", summary, sep="\n")

fig, ax = plt.subplots(figsize=(6.5, 6.5))
for (px, py, drive), (qx, qy, _) in zip(path[:-1], path[1:]):
    ax.plot([px, qx], [py, qy], color='#1f77b4' if drive else '#ff7f0e', lw=1.5)
for cx, cy, r in OBSTACLES:
    ax.add_patch(plt.Circle((cx, cy), r, color='#444'))
ax.plot(5.544, 5.544, 'g*', ms=14, label='start')
ax.plot([], [], color='#1f77b4', label='moving forward'); ax.plot([], [], color='#ff7f0e', label='stopped / turning')
ax.add_patch(plt.Circle((0, 0), 0, color='#444', label='virtual obstacle'))
ax.set_xlim(0, 11); ax.set_ylim(0, 11); ax.set_aspect('equal'); ax.grid(alpha=.3)
ax.set_title("Simulated robot path (offline demo, same logic as the ROS 2 nodes)\n" + summary, fontsize=8)
ax.legend(loc='upper right', fontsize=8)
fig.savefig(os.path.join(out, 'simulation_demo.png'), dpi=130, bbox_inches='tight')
sys.exit(0 if min_clear > 0 else 1)
