import rclpy
from rclpy.node import Node


from drone_interfaces.msg import RCcommands
from drone_interfaces.srv import HeightCommands
from drone_interfaces.msg import Telemetry   

from time import sleep
from djitellopy import Tello

class StaticCommandsExec(Node):

    def __init__(self):
        super().__init__("static_commands_exec")

        self.telemetry_subscription = self.create_subscription(
            RCcommands,
            'rc_commands',
            self.listener_callback,
            10)

        self.telemetry_publisher = self.create_publisher(Telemetry, 'telemtetry', 10)
        telemetry_timer_period = 1/2  #period of publishing
        self.timer = self.create_timer(telemetry_timer_period, self.telemetry_callback)

        self.srv = self.create_service(HeightCommands, 'set_height', self.srv)

        self.tello = Tello()
        self.tello.connect()

    def listener_callback(self, msg):
         self.get_logger().info(f"← {msg.left_right_velocity} -----> {msg.forward_backward_velocity} ↓{msg.up_down_velocity} ø{msg.yaw_velocity}")


    def srv(self,request, response):
        self.get_logger().info(f'Received command: {request.command}')
        sleep(2)
        response.success = True
        self.get_logger().info(f'Return command: {response.success}')
        
        return response

    def telemetry_callback(self):
        msg = Telemetry()
        data = self.mesurments()

        msg.front = data[0]
        msg.left = data[1]
        msg.right = data[2]
        msg.back = data[3]
        msg.degree = data[5]
        self.telemetry_publisher.publish(msg)
        self.get_logger().info('Publishing telemetry')

    def mesurments(self):
        try:
            responses = self.tello.send_read_command('EXT tof?').split()
            if len(responses) < 6:
                return self.mesurments()
            else:
                return [int(responses[1]),int(responses[4]),int(responses[3]),int(responses[2]),int(responses[5]),self.tello.get_yaw()+180]
        except Exception as e:
            return self.mesurments()

def main(args=None):
    rclpy.init(args=args)

    static_command_exec = StaticCommandsExec()

    rclpy.spin(static_command_exec)

    static_command_exec.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
