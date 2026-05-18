#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class ToolVelocityMapperNode(Node):
    def __init__(self):
        super().__init__('tool_velocity_mapper')
        self.max_tool_speed_rad_s = 35.0
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel_joy_tools',
            self.cmd_vel_callback,
            10,
        )
        self.publisher = self.create_publisher(
            Float64MultiArray,
            'controller_steadydrive/commands',
            10,
        )
        self.get_logger().info('ToolVelocityMapperNode started.')

    def cmd_vel_callback(self, msg: Twist):
        left_motor = (msg.linear.x - msg.angular.z) * self.max_tool_speed_rad_s
        right_motor = (msg.linear.x + msg.angular.z) * self.max_tool_speed_rad_s

        motor_msg = Float64MultiArray()
        motor_msg.data = [left_motor, right_motor]
        self.publisher.publish(motor_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ToolVelocityMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
