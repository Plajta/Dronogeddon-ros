import rclpy

from drone_interfaces.msg import RCcommands
from drone_interfaces.srv import HeightCommands

class DroneControls():
    def __init__(self,node):
        self.node = node
        
        self.static_publisher_response = node.create_client(HeightCommands, 'set_height')
        self.static_publisher_noResponse = node.create_publisher(RCcommands, 'rc_commands', 1)

        while not self.static_publisher_response.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Service not available, waiting again...')

        

    def send_height(self,arg):
        request = HeightCommands.Request()
        request.command = arg
        self.node.get_logger().info(f'Sending command: {arg}')
        
        # future = self.static_publisher_response.call_async(request)
        # self.node.get_logger().info(f'Future: {future}')
        # rclpy.spin_until_future_complete(self.node, future)

        # Volání asynchronní služby
        future = self.static_publisher_response.call_async(request)

        # Přiřazení callbacku k future, který bude zpracovávat odpověď
        future.add_done_callback(self.service_response_callback)

        return True

        
        # self.node.get_logger().info(f'Geting response: {future.result()}')
        # return future.result()


    def service_response_callback(self, future):
        # Zpracování odpovědi ze služby
        try:
            response = future.result()
            self.node.get_logger().info(f'Odpověď služby: {response.success}')
        except Exception as e:
            self.node.get_logger().error(f'Chyba při volání služby: {str(e)}')


    def send_rc_commands(self,LR,FB,UD,Y):
        msg = RCcommands()
        msg.left_right_velocity = LR
        msg.forward_backward_velocity = FB
        msg.up_down_velocity = UD
        msg.yaw_velocity = Y
        
        self.node.get_logger().info(f'Publishing {msg}')
        self.static_publisher_noResponse.publish(msg)