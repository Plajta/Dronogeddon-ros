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
import threading
import cv2
import time
from djitellopy import Tello

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')

        self.lock = threading.Lock()
        self.data = None

        self.tello = Tello()
        self.tello.connect()
        self.keepRecording = True
        self.tello.streamon()
        self.frame_read = self.tello.get_frame_read()

        self.keepRecording = True
        self.recorder_thread = threading.Thread(target=self.video_recorder)
        self.recorder_thread.start()

        self.subscription = self.create_subscription(
            Telemetry,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        

        

        




    def listener_callback(self, msg):
        self.get_logger().info(f"Drone distance ↑{msg.front} ←{msg.left} →{msg.right} ↓{msg.back} ø{msg.degree}")
        with self.lock:
            self.data = msg

    def video_recorder(self):
        font = cv2.FONT_HERSHEY_SIMPLEX
        start_time = time.time()
        log_time = time.strftime("%Y_%m_%d_%H_%M_%S", time.gmtime())

        while self.keepRecording:
            frame = self.frame_read.frame
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            if not self.data == None:
                cv2.putText(frame, 
                    f"{round(time.time()-start_time, 2)}s", 
                    (10, 20), 
                    font, 1/2, 
                    (0, 255, 255),
                    2,
                    cv2.LINE_4) 
        
                cv2.putText(frame, 
                            f"{log_time}", 
                            (770, 20), 
                            font, 1/2, 
                            (0, 255, 255), 
                            2, 
                            cv2.LINE_4) 
                
                with self.lock:
                    cv2.putText(frame, 
                            f"left: {self.data.left} front: {self.data.front} right: {self.data.right}", 
                            (10, 700), 
                            font, 1/2, 
                            (0, 255, 255), 
                            2, 
                            cv2.LINE_4) 

            cv2.imshow('Tello Video Stream', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            time.sleep(1 / 30)

        cv2.destroyAllWindows()

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
