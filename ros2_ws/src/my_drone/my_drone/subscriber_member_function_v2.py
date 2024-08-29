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
from drone_interfaces.msg import Telemetry   
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import threading
import cv2
import time

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')

        self.lock = threading.Lock()
        self.data = None


        self.telemetry_subscription = self.create_subscription(
            Telemetry,
            'topic',
            self.listener_callback,
            10)
        #self.subscription  # prevent unused variable warning
        
        self.video_subscription = self.create_subscription(
            Image,
            'video_frames',
            self.video_callback,
            10)
        self.subscriptions  # prevent unused variable warning

        self.bridge = CvBridge()

        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.start_time = time.time()
        self.log_time = time.strftime("%Y_%m_%d_%H_%M_%S", time.gmtime())

        

        




    def listener_callback(self, msg):
        self.get_logger().info(f"Drone distance ↑{msg.front} ←{msg.left} →{msg.right} ↓{msg.back} ø{msg.degree}")
        with self.lock:
            self.data = msg

    def video_callback(self,msg):

        try:
            # Convert ROS Image message to OpenCV image
            
            

          
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            if not self.data == None:
                cv2.putText(frame, 
                    f"{round(time.time()-self.start_time, 2)}s", 
                    (10, 20), 
                    self.font, 1/2, 
                    (0, 255, 255),
                    2,
                    cv2.LINE_4) 
        
                cv2.putText(frame, 
                            f"{self.log_time}", 
                            (770, 20), 
                            self.font, 1/2, 
                            (0, 255, 255), 
                            2, 
                            cv2.LINE_4) 
                
                with self.lock:
                    cv2.putText(frame, 
                            f"left: {self.data.left} front: {self.data.front} right: {self.data.right}", 
                            (10, 700), 
                            self.font, 1/2, 
                            (0, 255, 255), 
                            2, 
                            cv2.LINE_4) 

            cv2.imshow('Tello Video Stream', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                pass

        except CvBridgeError as e:
            self.get_logger().error(f"Error converting ROS Image to OpenCV Image: {e}")


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
