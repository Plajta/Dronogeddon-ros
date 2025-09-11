import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

# Import tf_transformations with numpy 1.21.5 compatibility
try:
    from .tf_transformations_fix import quaternion_from_euler
except ImportError:
    try:
        import tf_transformations
        quaternion_from_euler = tf_transformations.quaternion_from_euler
    except (ImportError, AttributeError):
        # Fallback implementation
        def quaternion_from_euler(roll, pitch, yaw):
            import math
            roll_half = roll * 0.5
            pitch_half = pitch * 0.5
            yaw_half = yaw * 0.5
            
            cr = math.cos(roll_half)
            sr = math.sin(roll_half)
            cp = math.cos(pitch_half)
            sp = math.sin(pitch_half)
            cy = math.cos(yaw_half)
            sy = math.sin(yaw_half)
            
            w = cr * cp * cy + sr * sp * sy
            x = sr * cp * cy - cr * sp * sy
            y = cr * sp * cy + sr * cp * sy
            z = cr * cp * sy - sr * sp * cy
            
            return [x, y, z, w]

from drone_interfaces.msg import TelemetryData

class CompassMapping(Node):

    def __init__(self):
        super().__init__("compass_mapping")

        self.subscriber = self.create_subscription(TelemetryData, 'telemetry', self.telemetry_callback, 1)
        self.publisher = self.create_publisher(Imu, 'compass/data', 10)

    def telemetry_callback(self, telemetry):
        imu_msg = Imu()

        yaw_angle = math.radians(telemetry.yaw)
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
