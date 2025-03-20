import time
import random

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from drone_interfaces.action import Movement
from my_drone.lib.drone_controls import DroneControls

class MovementServer(Node):

    def __init__(self):
        super().__init__('Movement_server')

        #self.dc = DroneControls(self)

        self._action_server = ActionServer(
            self,
            Movement,
            'movement',
            self.execute_callback)






    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        order =goal_handle.request.order
        end_value = goal_handle.request.value

        
        self.get_logger().info(f"ordered to {order} || to value {end_value}")
        
        
        feedback_msg = Movement.Feedback()
        feedback_msg.status = "Waiting..."
        feedback_msg.completion = 0
        feedback_msg.speed = 0
        
        goal_handle.publish_feedback(feedback_msg)

        time.sleep(2)

        # for i in range(end_value):
        #     feedback_msg.status = "on Going..."
        #     feedback_msg.completion = int(i*100/end_value)
        #     feedback_msg.speed = 30 + random.randint(-5,5)
            
        #     goal_handle.publish_feedback(feedback_msg)
        #     time.sleep(1)

        goal_handle.succeed()

        result = Movement.Result()
        result.success = True
        return result





    


def main(args=None):
    rclpy.init(args=args)

    mov = MovementServer()

    rclpy.spin(mov)


if __name__ == '__main__':
    main()