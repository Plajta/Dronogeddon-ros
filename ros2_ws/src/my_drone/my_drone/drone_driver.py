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
import random
from djitellopy import Tello
from threading import Lock
import threading
from time import sleep
import socket

from drone_interfaces.msg import ToFDistances
from drone_interfaces.msg import TelemetryData
from drone_interfaces.msg import RCcommands
from drone_interfaces.srv import HeightCommands
from std_msgs.msg import String 
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2

class DroneComm(Node):

    def __init__(self):
        super().__init__('drone_comm')

        #publisher for distances from ToF data to "ToF_distances" topic
        self.tof_publisher = self.create_publisher(ToFDistances, 'ToF_distances', 10)
        tof_timer_period = 1/10  #period of publishing
        self.timer = self.create_timer(tof_timer_period, self.tof_callback)

        #publisher for tello telemetry data to "telemetry" topic
        self.telemetry_publisher = self.create_publisher(TelemetryData, 'telemetry', 10)
        telemetry_timer_period = 1/10  #period of publishing
        self.timer = self.create_timer(telemetry_timer_period, self.telemetry_callback)
        
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
        self.tello = Tello()
        self.tello.connect()
        self.tello.streamon()
         
        
        self.video_publisher_thread.start()


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
        self.get_logger().info('Publishing ToF readings')

    def telemetry_callback(self):
        try:
            data = self.tello.get_current_state()
            self.tello.get_current_state()

            msg = TelemetryData()
            msg.pitch = int(data.get('pitch', 0))
            msg.roll = int(data.get('roll', 0))
            msg.yaw = int(data.get('yaw', 0))

            msg.vgx = int(data.get('vgx', 0))
            msg.vgy = int(data.get('vgy', 0))
            msg.vgz = int(data.get('vgz', 0))

            msg.templ = int(data.get('templ', 0))
            msg.temph = int(data.get('temph', 0))

            msg.tof = int(data.get('tof', 0))
            msg.h = int(data.get('h', 0))
            msg.bat = int(data.get('bat', 0))

            msg.baro = float(data.get('baro', 0.0))
            msg.time = int(data.get('time', 0))

            msg.agx = float(data.get('agx', 0.0))
            msg.agy = float(data.get('agy', 0.0))
            msg.agz = float(data.get('agz', 0.0))

            self.telemetry_publisher.publish(msg)
            self.get_logger().info('Publikována telemetrie.')

        except:
            pass

        
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
        try:
            recived = self.tello.send_read_command('EXT tof?')
            try:
                data_string = self.extract_data_string(recived)
                data = self.extract_data(data_string)
                new_data = []
                matrix = 8
                for i in range(matrix,0,-1):
                    for j in range(0,matrix):
                        new_data.append(data[i+j*matrix-1])
                data = new_data

            except Exception as e:
                print(f"exception while decoding matrix: {e}")
                data=[]
                

            responses = recived.split()
            if len(responses) < 6:
                print("number of responses under 6")
                return self.mesurments()
            else:
                if int(responses[1]) > 1200:
                    responses[1] = "1200"
                            #front               left               right            back           accuraci            degrees
                return [int(responses[1]),int(responses[4]),int(responses[3]),int(responses[2]),int(responses[5]),self.tello.get_yaw()+180,data]
        
        except Exception as e:
            print(f"exception ocurred while geting mesurments:\t{e}")
            return self.mesurments()
        

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
    
    def unpack_chars_using_95(self,high, mid, low):
        pack = (ord(low) - 32) + (ord(mid) - 32) * 95 + (ord(high) - 32) * 95 * 95


        num1 = pack & 0b111111111
        num2 = (pack >> 9) & 0b111111111

        return(num2,num1)

    def extract_data(self,string):
        data = []

        for i in range(0,96,3):
            try:
                duet = self.unpack_chars_using_95(string[i],string[i+1],string[i+2])
                for i in duet:
                    data.append(i)
            except:
                pass

        return data


    def extract_data_string(self,string):
        source_split = string.split()

        seam = 0
        for i in range(6):
            seam+= len(source_split[i])+1

        export = string[seam:]

        return export


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
