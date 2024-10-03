import rclpy
from rclpy.node import Node
from time import sleep

from drone_interfaces.msg import RCcommands
from drone_interfaces.srv import HeightCommands


class StaticCommandsPublisher(Node):

    def __init__(self):
        super().__init__("static_commands_publisher")

        self.static_publisher_response = self.create_client(HeightCommands, 'set_height')
        self.static_publisher_noResponse = self.create_publisher(RCcommands, 'rc_commands', 1)

        while not self.static_publisher_response.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.request = HeightCommands.Request()



    def send_height(self,arg):
        self.request.command = arg
        self.get_logger().info(f'Sending command: {arg}')
        
        future = self.static_publisher_response.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()


    def send_rc_commands(self,LR,FB,UD,Y):
        msg = RCcommands()
        msg.left_right_velocity = LR
        msg.forward_backward_velocity = FB
        msg.up_down_velocity = UD
        msg.yaw_velocity = Y
        
        self.get_logger().info(f'Publishing {msg}')
        self.static_publisher_noResponse.publish(msg)
        

    def sendComands(self):
        if self.send_height(1):
            self.get_logger().info("takeoff ok")
        else:
            self.get_logger().info("takeoff bad")
            
        self.send_rc_commands(0,50,0,0)
        self.get_logger().info("forward")
        sleep(6)
        
        self.get_logger().info("right")
        self.send_height(-90)
        self.send_height(-90)
        self.send_height(-90)
        self.send_height(-90)
        self.send_height(-90)
        
        self.get_logger().info("forward")
        self.send_rc_commands(0,50,0,0)
        sleep(3)
        
        self.get_logger().info("right")
        self.send_height(-90)
        
        self.get_logger().info("forward")
        self.send_rc_commands(0,50,0,0)
        sleep(6)

        self.get_logger().info("right")
        self.send_height(90)
        self.send_height(90)
        self.send_height(90)

        self.get_logger().info("forward")
        self.send_rc_commands(0,50,0,0)
        sleep(3)


        if self.send_height(0):
            self.get_logger().info("landing ok")
        else:
            self.get_logger().info("landing bad")
            

def main(args=None):
     rclpy.init(args=args)
     
     static_commands_publisher = StaticCommandsPublisher()

     static_commands_publisher.sendComands()

     static_commands_publisher.destroy_node()
     rclpy.shutdown()

if __name__ == '__main__':
    main()
