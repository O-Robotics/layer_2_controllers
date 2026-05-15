#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class TwistStamperNode(Node):
    def __init__(self):
        super().__init__("twist_stamper")

        self.declare_parameter("input_topic", "cmd_vel_wheels")
        self.declare_parameter("output_topic", "diff_cont/cmd_vel")
        self.declare_parameter("frame_id", "base_footprint")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._frame_id = self.get_parameter("frame_id").value

        self._publisher = self.create_publisher(TwistStamped, output_topic, 10)
        self._subscription = self.create_subscription(
            Twist,
            input_topic,
            self._stamp_twist,
            10,
        )

        self.get_logger().info(
            f"Stamping Twist commands from '{input_topic}' to '{output_topic}'"
        )

    def _stamp_twist(self, msg):
        stamped_msg = TwistStamped()
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        stamped_msg.header.frame_id = self._frame_id
        stamped_msg.twist = msg
        self._publisher.publish(stamped_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TwistStamperNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
