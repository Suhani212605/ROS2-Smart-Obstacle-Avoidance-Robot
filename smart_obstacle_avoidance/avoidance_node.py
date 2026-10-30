"""Obstacle avoidance controller.  /scan (LaserScan)  ->  /turtle1/cmd_vel (Twist)."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from smart_obstacle_avoidance.core import AvoidanceController


class AvoidanceNode(Node):
    def __init__(self):
        super().__init__('avoidance_node')
        self.declare_parameter('stop_distance', 1.2)       # metres: stop/turn below this
        self.declare_parameter('clear_distance', 1.6)      # metres: resume above this
        self.declare_parameter('forward_speed', 1.5)
        self.declare_parameter('turn_speed', 1.5)
        p = lambda n: self.get_parameter(n).value
        self.ctrl = AvoidanceController(p('stop_distance'), p('clear_distance'),
                                        forward_speed=p('forward_speed'), turn_speed=p('turn_speed'))
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.last_state = None
        self.get_logger().info('Avoidance controller started.')

    def on_scan(self, scan):
        d = self.ctrl.decide(scan.ranges, scan.angle_min, scan.angle_increment)
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = d.linear, d.angular
        self.pub.publish(cmd)

        msg = (f'front={d.front_min:.2f} m  left={d.left_min:.2f} m  right={d.right_min:.2f} m'
               f'  | cmd: v={d.linear:.1f} w={d.angular:+.1f}  | {d.state}')
        if d.state[:5] != (self.last_state or '')[:5]:     # always log decision changes
            self.get_logger().info(msg)
        else:                                              # otherwise at most once a second
            self.get_logger().info(msg, throttle_duration_sec=1.0)
        self.last_state = d.state


def main(args=None):
    rclpy.init(args=args)
    node = AvoidanceNode()
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
