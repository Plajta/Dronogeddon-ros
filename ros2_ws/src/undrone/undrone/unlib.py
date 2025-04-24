import math
from itertools import product

import rclpy
from rclpy.node import Node
import threading
from threading import Lock
from time import sleep

class locationSim():
    def __init__(self,x=500,y=500,size=1000,fps=5):
        #super().__init__("location_sim")
        self.lock = Lock()

        self.x = x
        self.y = y
        self.angle = 0
        self.room_size = size
        self.fps = fps
        
        self.forward_speed = 0
        self.side_speed = 0
        self.angle_speed = 0

        self.video_publisher_thread = threading.Thread(target=self.control_loop)
        self.video_publisher_thread.daemon = True
        self.video_publisher_thread.start()
        sleep(1/10)
        
    def distance_to_wall(self,angle, pitch=0.0):
        
        theta = math.radians(angle%360)
        
        dx = math.cos(theta)
        dy = math.sin(theta)
        dz = math.cos(math.radians(pitch))
        
        distance = 0.0
        
        while True:
            with self.lock:
                new_x = self.x + distance * dx
                new_y = self.y + distance * dy
            
            if new_x <= 0 or new_x >= self.room_size or new_y <= 0 or new_y >= self.room_size:
                break
            
            distance += 1.0
        
        return int(distance/dz)

    def mesurmets(self):
        angle = self.get_angle()
        pitch = 0
        dists = []

        for i, j in product([-22.5, -16.07, -9.64, -3.21, 3.21, 9.64, 16.07, 22.5], repeat=2):
            dists.append(self.distance_to_wall(angle + j, pitch + i))

        return [self.distance_to_wall(angle),self.distance_to_wall(angle-90),self.distance_to_wall(angle+90),self.distance_to_wall(angle+180),0,int(angle),dists]

    def set_angle_speed(self,aS):
        with self.lock:
            self.angle_speed = aS
    
    def set_forward_speed(self,fS):
        with self.lock:
            self.forward_speed = fS

    def set_side_speed(self,sS):
        with self.lock:
            self.side_speed = sS


    def get_angle(self):
        with self.lock:
            return self.angle

    def send_rc_control(self,side,forward,up,yaw):
        self.set_side_speed(side)
        self.set_forward_speed(forward)
        self.set_angle_speed(yaw)
        

            
    def control_loop(self):
        while True:
            with self.lock:
                forward_speed = self.forward_speed
                side_speed = self.side_speed
                angle_speed = self.angle_speed

            theta = math.radians(self.angle)
            
            dx = math.cos(theta)
            dy = math.sin(theta)
                

            with self.lock:
                self.angle += angle_speed / self.fps
                self.angle = self.angle%360
                self.x += forward_speed*8/2 * dx / self.fps
                self.y += forward_speed*8/2 * dy / self.fps

                self.y += side_speed*8/2 * dx / self.fps
                self.x -= side_speed*8/2 * dy / self.fps
            

            with self.lock:
                #print(f"x = {int(self.x)}\ty = {int(self.y)}\tangle = {int(self.angle)}\tangle speed = {angle_speed} \tforward speed = {forward_speed}\tside speed = {side_speed}")
                pass

            sleep(1/self.fps)


if __name__ == '__main__':
    gg = locationSim(10,500,1000,5)


    print(gg.mesurmets())

