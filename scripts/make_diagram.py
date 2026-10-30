"""Draws docs/architecture.png"""
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

fig, ax = plt.subplots(figsize=(8, 4.6)); ax.axis('off'); ax.set_xlim(-1.6, 7.6); ax.set_ylim(-0.6, 4.2)
def box(cx, cy, txt, col):
    ax.add_patch(FancyBboxPatch((cx - 1.4, cy - 0.45), 2.8, 0.9, boxstyle='round,pad=0.05', fc=col, ec='k'))
    ax.text(cx, cy, txt, ha='center', va='center', fontsize=9, weight='bold')
box(3, 0.2, 'turtlesim_node\n(simulated robot)', '#d9ead3')
box(0.5, 3.2, 'lidar_sim_node\n(SIMULATED LiDAR)', '#cfe2f3')
box(5.5, 3.2, 'avoidance_node\n(controller)', '#fce5cd')
def arrow(a, b, label, pos, ha):
    ax.add_patch(FancyArrowPatch(a, b, arrowstyle='-|>', mutation_scale=16, lw=1.6))
    ax.text(*pos, label, ha=ha, va='center', fontsize=8)
arrow((2.0, 0.75), (0.7, 2.7), '/turtle1/pose\nturtlesim/msg/Pose', (0.9, 1.6), 'right')
arrow((1.9, 3.2), (4.1, 3.2), '/scan\nsensor_msgs/LaserScan', (3.0, 3.65), 'center')
arrow((5.3, 2.7), (4.0, 0.75), '/turtle1/cmd_vel\ngeometry_msgs/Twist', (5.1, 1.6), 'left')
ax.set_title('Architecture: ROS 2 nodes and topics', fontsize=11)
fig.savefig(os.path.join(os.path.dirname(__file__), '..', 'docs', 'architecture.png'), dpi=130, bbox_inches='tight')
