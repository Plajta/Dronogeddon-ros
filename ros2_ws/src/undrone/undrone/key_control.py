import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import getch  # Import getch for real-time key press detection
from drone_interfaces.msg import RCcommands

class RealTimeKeyboardNode(Node):
    def __init__(self):
        super().__init__('realtime_keyboard_node')
        
        self.static_publisher_noResponse = self.create_publisher(RCcommands, 'rc_commands', 1)
        self.create_timer(0.1, self.check_key_press)
        self.speed = 20

        self.get_logger().info('Real-Time Keyboard Input Node initialized. Press any key...')

    def check_key_press(self):
        self.get_logger().info(f'Check trigered')
        key = getch.getch()  # Use getch to capture a single key press
        if key:
            self.publish_key(key)
        else:
            self.send_rc_commands(0,0,0,0)

    def send_rc_commands(self,LR,FB,UD,Y):
        msg = RCcommands()
        msg.left_right_velocity = LR
        msg.forward_backward_velocity = FB
        msg.up_down_velocity = UD
        msg.yaw_velocity = Y
        
        self.get_logger().info(f'Publishing {msg}')
        self.static_publisher_noResponse.publish(msg)

    def publish_key(self, key):
        switch_dict = {
        'w': [0,self.speed,0,0],
        's': [0,-self.speed,0,0],
        'a': [-self.speed,0,0,0],
        'd': [self.speed,0,0,0],
        'q': [0,0,0,int(-self.speed)],
        'e': [0,0,0,int(self.speed)]
            }
        movement = switch_dict.get(key, [0,0,0,0])
        self.get_logger().info(f'Published: "{movement}"')
        self.send_rc_commands(*movement)



def main(args=None):
    rclpy.init(args=args)
    node = RealTimeKeyboardNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Real-Time Keyboard Node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
