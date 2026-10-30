# Resume description (honest wording)

**ROS 2 Smart Obstacle Detection and Avoidance (simulation)** | Python, ROS 2, rclpy, Turtlesim
- Built a ROS 2 package with a simulated 2D LiDAR node (ray casting, `sensor_msgs/LaserScan`) and a reactive obstacle-avoidance controller publishing `geometry_msgs/Twist` velocity commands to a simulated Turtlesim robot.
- Implemented stop-and-turn avoidance with hysteresis, parameterised via YAML and a launch file; unit-tested the sensing and decision logic with pytest.
- Simulation only; no physical robot or LiDAR hardware.

(Add "verified in Turtlesim on ROS 2 <Humble/Jazzy>" only after you have run it yourself.)

# ROS 2 concepts demonstrated
Nodes; topics; publishers and subscribers; message types (`LaserScan`, `Twist`, `Pose`); timers; parameters and YAML config; launch files; `ament_python` packaging with `colcon`; sensor data processing; velocity control; reactive obstacle avoidance; logging with throttling.

# Interview Q&A

1. **What does the project do?** A simulated robot drives forward, reads a simulated LiDAR scan, and stops and turns when something is within 1.2 m ahead, then resumes when the path clears.
2. **Is the LiDAR real?** No. `lidar_sim_node` ray-casts from the turtle's pose against virtual circles and walls and publishes a standard `LaserScan`, so the controller would work unchanged with a real driver publishing `/scan`.
3. **Why separate nodes?** Sensor and controller are decoupled through the `/scan` topic, so either can be swapped or tested independently.
4. **Why two thresholds (1.2 m / 1.6 m)?** Hysteresis: without it the robot would flicker between "turn" and "go" right at the threshold.
5. **How does it pick a turn direction?** It compares the average range on the left and right sides and turns toward the more open one, then keeps that direction until the front is clear.
6. **Why is the logic in `core.py` with no ROS imports?** So it can be unit-tested without a ROS install and reused by the offline demo.
7. **Which message fields matter in `LaserScan`?** `angle_min`, `angle_increment`, `ranges`, `range_max`; the angle of beam *i* is `angle_min + i * angle_increment`.
8. **How would you improve it?** Add Gazebo with a real physics simulator, a lidar driver, a costmap with Nav2, smoother velocity control (PID or proportional slowdown), and handling of dead-end cases.
9. **Limitations?** 2D, reactive only (no memory or planning), idealised noise-free sensor.
