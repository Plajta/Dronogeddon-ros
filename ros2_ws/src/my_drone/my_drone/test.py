# tello_drone/tello_node.py
import time
import cv2
from threading import Thread
from djitellopy import Tello
import rclpy
from rclpy.node import Node

class TelloNode(Node):
    def __init__(self):
        super().__init__('tello_node')
        self.tello = Tello()

        self.tello.connect()
        self.keepRecording = True
        self.tello.streamon()
        self.frame_read = self.tello.get_frame_read()

        #self.recorder_thread = Thread(target=self.video_recorder)
        #self.recorder_thread.start()

        self.video_recorder()

        # self.tello.takeoff()
        # self.tello.move_up(100)
        # self.tello.rotate_counter_clockwise(360)
        # self.tello.land()
        #time.sleep(20)

        #self.keepRecording = False
        #self.recorder_thread.join()

    def video_recorder(self):
        while self.keepRecording:
            frame = self.frame_read.frame
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            cv2.imshow('Tello Video Stream', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            time.sleep(1 / 30)

        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    tello_node = TelloNode()
    rclpy.spin(tello_node)
    tello_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
