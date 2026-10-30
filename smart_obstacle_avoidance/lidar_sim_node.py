"""Simulated LiDAR node.  /turtle1/pose (Pose)  ->  /scan (LaserScan).
The readings are SIMULATED by ray casting against virtual obstacles in core.py."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from turtlesim.msg import Pose

from smart_obstacle_avoidance.core import make_scan


class LidarSimNode(Node):
    def __init__(self):
        super().__init__('lidar_sim_node')
        self.declare_parameter('num_beams', 37)
        self.declare_parameter('range_max', 3.0)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.num_beams = self.get_parameter('num_beams').value
        self.range_max = self.get_parameter('range_max').value
        rate = self.get_parameter('publish_rate_hz').value

        self.pose = None                                   # latest robot pose
        self.create_subscription(Pose, '/turtle1/pose', self.on_pose, 10)
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.create_timer(1.0 / rate, self.publish_scan)
        self.scan_time = 1.0 / rate
        self.get_logger().info('Simulated LiDAR started (virtual obstacles, no hardware).')

    def on_pose(self, msg):
        self.pose = msg

    def publish_scan(self):
        if self.pose is None:                              # wait for turtlesim
            return
        ranges, a_min, a_max, inc = make_scan(self.pose.x, self.pose.y, self.pose.theta,
                                              self.num_beams, self.range_max)
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser'
        scan.angle_min, scan.angle_max, scan.angle_increment = a_min, a_max, inc
        scan.time_increment, scan.scan_time = 0.0, self.scan_time
        scan.range_min, scan.range_max = 0.05, float(self.range_max)
        scan.ranges = [float(r) for r in ranges]
        self.pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = LidarSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
