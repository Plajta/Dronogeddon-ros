#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from drone_interfaces.msg import ToFDistances
import math

class UntelemtryNode(Node):
    def __init__(self):
        super().__init__("untelemtry_node")
        self.telemetry_publisher = self.create_publisher(ToFDistances, 'ToF_distances', 10)
        telemetry_timer_period = 1 / 10  # period of publishing
        self.timer = self.create_timer(telemetry_timer_period, self.telemetry_callback)

        self.degree = 0
        self.size = 4000

    def telemetry_callback(self):
        msg = ToFDistances()
        angle_rad = math.radians(self.degree % 90)

        if self.degree % 90 <= 45:
            dist = int(1 / abs(math.cos(angle_rad)) * self.size)
        else:
            dist = int(1 / abs(math.sin(angle_rad)) * self.size)

        msg.front = dist
        msg.left = dist
        msg.right = dist
        msg.back = dist
        msg.degree = self.degree
        self.telemetry_publisher.publish(msg)
        self.get_logger().info(f'Publishing telemetry [{self.degree}, {dist}]')

        self.degree = (self.degree + 5) % 360


def main(args=None):
    rclpy.init(args=args)

    # initialize model
    telemtry = UntelemtryNode()
    rclpy.spin(telemtry)
    telemtry.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
