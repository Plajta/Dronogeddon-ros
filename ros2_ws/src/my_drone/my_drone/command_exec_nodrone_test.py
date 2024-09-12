import rclpy
from rclpy.node import Node


from drone_interfaces.msg import RCcommands
from drone_interfaces.srv import HeightCommands

from time import sleep


class StaticCommandsExec(Node):

    def __init__(self):
        super().__init__("static_commands_exec")

        self.telemetry_subscription = self.create_subscription(
            RCcommands,
            'rc_commands',
            self.listener_callback,
            10)

        self.srv = self.create_service(HeightCommands, 'set_height', self.srv)

    def listener_callback(self, msg):
         self.get_logger().info(f"← {msg.left_right_velocity} -----> {msg.forward_backward_velocity} ↓{msg.up_down_velocity} ø{msg.yaw_velocity}")


    def srv(self,request, response):
        self.get_logger().info(f'Received command: {request.command}')
        sleep(2)
        response.success = True
        
        return response

def main(args=None):
    rclpy.init(args=args)

    static_command_exec = StaticCommandsExec()

    rclpy.spin(static_command_exec)

    static_command_exec.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
