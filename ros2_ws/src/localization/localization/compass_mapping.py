import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import tf_transformations

from drone_interfaces.msg import Telemetry

class CompassMapping(Node):

    def __init__(self):
        super().__init__("compass_mapping")

        self.subscriber = self.create_subscription(Telemetry, 'telemtetry', self.telemetry_callback, 1)
        self.publisher = self.create_publisher(Imu, 'compass/data', 10)

    def telemetry_callback(self, telemetry):
        imu_msg = Imu()

        yaw_angle = math.radians(telemetry.degree)
        quaternion = tf_transformations.quaternion_from_euler(0, 0, yaw_angle)
        imu_msg.orientation.x = quaternion[0]
        imu_msg.orientation.y = quaternion[1]
        imu_msg.orientation.z = quaternion[2]
        imu_msg.orientation.w = quaternion[3]
        self.publisher.publish(imu_msg)


def main(args=None):
     rclpy.init(args=args)
     
     compassMapping = CompassMapping()

     rclpy.spin(compassMapping)

     compassMapping.destroy_node()
     rclpy.shutdown()

if __name__ == '__main__':
    main()
