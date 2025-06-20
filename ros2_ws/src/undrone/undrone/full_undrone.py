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
import rclpy.executors
from rclpy.node import Node
from djitellopy import Tello
from threading import Lock
import threading
from time import sleep

from drone_interfaces.msg import ToFDistances, RCcommands, TelemetryData
from drone_interfaces.srv import HeightCommands
from std_msgs.msg import String 
from sensor_msgs.msg import Image, LaserScan, PointCloud2, PointField

from cv_bridge import CvBridge
import cv2

from undrone.unlib import locationSim


class DroneComm(Node):

    def __init__(self):
        super().__init__('drone_comm')

        #publisher for telemetry data to "ToF_distances" topic
        self.telemetry_publisher = self.create_publisher(TelemetryData, 'telemetry', 10)
        telemetry_timer_period = 1/10  #period of publishing
        self.timer = self.create_timer(telemetry_timer_period, self.telemetry_callback)

        #publisher for ToF data to "ToF_distances" topic
        self.tof_publisher = self.create_publisher(ToFDistances, 'ToF_distances', 10)
        tof_timer_period = 1/10  #period of publishing
        self.timer = self.create_timer(tof_timer_period, self.tof_callback)
        
        #publisher for streaming video to "video_frames" topic
        self.video_publisher = self.create_publisher(Image, 'video_frames', 10)
        video_timer_period = 1/10
        self.video_publisher_thread = threading.Thread(target=self.publish_video_frame, args=(video_timer_period,))
        self.video_publisher_thread.daemon = True
        
        
        self.bridge = CvBridge()

        self.incoming_commands = self.create_subscription(
            RCcommands,
            'rc_commands',
            self.rc_command_callback,
            1)
        self.srv = self.create_service(HeightCommands, 'set_height', self.height_command_callback)


        self.tello_lock = Lock()
        self.tello = locationSim(fps=10)
        
        
        #self.video_publisher_thread.start()

    def telemetry_callback(self):
        msg = TelemetryData()

        msg.yaw = int(-self.tello.angle)
        msg.vgx = int(self.tello.forward_speed)
        msg.vgy = int(-self.tello.side_speed)
        msg.h = int(200)
        self.telemetry_publisher.publish(msg)
        self.get_logger().info('Publishing Telemetry')

    def tof_callback(self):
        msg = ToFDistances()
        with self.tello_lock:
            data = self.mesurments()

        msg.front = data[0]
        msg.left = data[1]
        msg.right = data[2]
        msg.back = data[3]
        msg.degree = data[5]
        msg.matrix = data[6]
        self.tof_publisher.publish(msg)
        self.get_logger().info('Publishing ToF')
        
    def publish_video_frame(self,interval):
        self.frame_read = self.tello.get_frame_read()
        while rclpy.ok():
            frame = self.frame_read.frame
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            # Convert OpenCV image to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.video_publisher.publish(ros_image)
            self.get_logger().info('Publishing video frame')
            sleep(interval)
            

    def mesurments(self):
        return self.tello.mesurmets()
        
    def rc_command_callback(self,msg):
        with self.tello_lock:
            self.get_logger().info(f"←→ {msg.left_right_velocity}  -->{msg.forward_backward_velocity} ↓{msg.up_down_velocity} ø{msg.yaw_velocity}")
            self.tello.send_rc_control(msg.left_right_velocity, msg.forward_backward_velocity, msg.up_down_velocity, msg.yaw_velocity)

    def height_command_callback(self,request, response):
        try:
            with self.tello_lock:
                response.success = True
    
                if request.command == 1:
                    self.get_logger().info(f'Received command: Takeoff')
                    self.tello.takeoff()
                elif request.command == 0:
                    self.get_logger().info(f'Received command: Land')
                    self.tello.send_rc_control(0,0,0,0)
                    self.tello.move_back(20)
                    self.tello.land()
                elif request.command == 90:
                    self.get_logger().info(f'Received command: rotate left')
                    self.tello.rotate_counter_clockwise(90)
                elif request.command == -90:
                    self.get_logger().info(f'Received command: rotate right')
                    self.tello.rotate_clockwise(90)
                else:
                    response.success = False
                    
                return response
            
        except:
            response.success = False
            return response

    def sevice_command_callback(self,request,response):
        
        return response


def main(args=None):
    rclpy.init(args=args)

    drone_comm = DroneComm()



    rclpy.spin(drone_comm)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    drone_comm.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
