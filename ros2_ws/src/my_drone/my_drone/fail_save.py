import rclpy
from rclpy.node import Node
from drone_interfaces.msg import ToFDistances   
from drone_interfaces.msg import RCcommands
from drone_interfaces.srv import HeightCommands
from time import sleep

from my_drone.lib.drone_controls import DroneControls

class FailSave(Node):
    def __init__(self):
        super().__init__('fail_save')

        self.dc = DroneControls(self)
        
        self.telemetry_subscription = self.create_subscription(
            ToFDistances,
            'ToF_distances',
            self.listener_callback,
            1)

    def listener_callback(self, msg):
        if msg.front < 100:
            rp = self.dc.send_height(0)
        self.get_logger().info(f"recived front data: {msg.front}")


def main(args=None):
    rclpy.init(args=args)

    fail_save = FailSave()

    rclpy.spin(fail_save)

    
    fail_save.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
