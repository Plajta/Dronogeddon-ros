# tello_drone/tello_node.py
import time
import cv2
from threading import Thread
from djitellopy import Tello
import rclpy
from rclpy.node import Node

def video_recorder(tello):

    for i in range(3600):
        frame = tello.get_frame_read().frame
        cv2.imshow('Tello Video Stream', frame)
        print(i)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(1 / 30)

    cv2.destroyAllWindows()


tello = Tello()

tello.connect()

tello.streamon()
frame_read = tello.get_frame_read()

recorder_thread = Thread(target=video_recorder,args=(tello,))
recorder_thread.start()

# tello.takeoff()
# tello.move_up(100)
# tello.rotate_counter_clockwise(360)
# tello.land()

keepRecording = False
recorder_thread.join()
tello.streamoff()

