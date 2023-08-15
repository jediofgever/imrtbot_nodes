#!/usr/bin/env python

import rclpy
import rclpy.node
import rclpy.qos
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)


class IMRTTeleop(rclpy.node.Node):
    def __init__(self):
        super(IMRTTeleop, self).__init__("IMRTTeleop")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1,
        )

        self._cmd_pub = self.create_publisher(Twist, "teleop/cmd_vel", qos_profile)
        self.create_subscription(Joy, "joy", self._joy_callback, 1)
        self.get_logger().info("IMRTTeleop started")

        self.vx = 2.0
        self.wz = 1.0

    def _joy_callback(self, joy_msg):
        vx = joy_msg.axes[0]
        wz = -joy_msg.axes[3]

        twist_msg = Twist()
        twist_msg.linear.x = vx * self.vx
        twist_msg.angular.z = wz * self.wz
        self._cmd_pub.publish(twist_msg)


def main():
    rclpy.init()
    node = IMRTTeleop()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
