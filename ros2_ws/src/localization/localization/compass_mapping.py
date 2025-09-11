import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

# Import centralized tf_transformations fix
from .tf_transformations_fix import quaternion_from_euler

from drone_interfaces.msg import TelemetryData

class CompassMapping(Node):

    def __init__(self):
        super().__init__("compass_mapping")

        self.subscriber = self.create_subscription(TelemetryData, 'telemetry', self.telemetry_callback, 1)
        self.publisher = self.create_publisher(Imu, 'compass/data', 10)

    def telemetry_callback(self, telemetry):
        imu_msg = Imu()

        yaw_angle = math.radians(telemetry.yaw)
        quaternion = quaternion_from_euler(0, 0, yaw_angle)
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
