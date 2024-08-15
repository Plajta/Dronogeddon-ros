# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
import random
from djitellopy import Tello

from drone_interfaces.msg import Telemetry   


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Telemetry, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.tello = Tello()

    def timer_callback(self):
        msg = Telemetry()
        data = self.mesurments()

        msg.front = data[0]
        msg.left = data[2]
        msg.right = data[3]
        msg.back = data[4]
        msg.degree = data[5]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing telemetry')

    def mesurments(self):
        try:
            responses = self.tello.send_read_command('EXT tof?').split()
            if len(responses) < 6:
                return self.mesurments()
            else:
                return [int(responses[1]),int(responses[4]),int(responses[3]),int(responses[2]),int(responses[5]),self.tello.get_yaw()+180]
        except Exception as e:
            return self.mesurments()
        


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
