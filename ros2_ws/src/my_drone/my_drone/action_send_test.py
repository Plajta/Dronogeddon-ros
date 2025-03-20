import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from drone_interfaces.action import Movement


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('test_movement_action_client')
        self._action_client = ActionClient(self, Movement, 'movement')

    def send_goal(self, order, value):
        goal_msg = Movement.Goal()
        goal_msg.order = order
        goal_msg.value = value

        self.get_logger().info(f'Sending action: goal {goal_msg.order} value {goal_msg.value}')

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Competed: '.format(result.success))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        status = ""
        completion = -1
        speed = -1
        
        try:
            status = feedback_msg.status
        except:
            self.get_logger().info('status error')


        try:
            completion = feedback_msg.completion
        except:
            self.get_logger().info('completion error')
            
        try:
            speed = feedback_msg.speed 
        except:
            self.get_logger().info('speed error')


        self.get_logger().info(f'Received feedback: {status} completed {completion} at speed {speed}')
 
def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    action_client.send_goal("Hoja testik",20)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()