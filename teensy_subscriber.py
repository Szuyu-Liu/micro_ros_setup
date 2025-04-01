import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler
from rclpy.qos import QoSProfile, ReliabilityPolicy

class CarPositionNode(Node):
    def __init__(self):
        super().__init__('car_position_node')
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/car_angles',
            self.car_angles_callback,
            qos_profile)

        self.imu_subscription = self.create_subscription(
            Float32MultiArray,
            '/imu_euler',
            self.imu_listener_callback,
            qos_profile)

        self.publisher = self.create_publisher(Float32MultiArray, '/car_position', 10)
        self.movement_publisher = self.create_publisher(Float32MultiArray, '/movement', 10)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        
        # Car state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Initial orientation
        self.prev_right_angle = None
        self.prev_left_angle = None
        
        # Parameters (Set these based on your car's configuration)
        self.circumference = 20.892  # Example wheel circumference in meters
        self.turning_circumference = 33.929 # Example turning circumference in meters

        self.publish_initial_movement()
    
    def publish_initial_movement(self):
        """Publishes the initial movement command."""
        movement_msg = Float32MultiArray()
        movement_msg.data = [1.0, 0.0]
        self.movement_publisher.publish(movement_msg)
        self.get_logger().info('Published initial movement command: [1.0, 0.0]')

    def car_angles_callback(self, msg):
        right_angle, left_angle = msg.data
        
        if self.prev_right_angle is None or self.prev_left_angle is None:
            self.prev_right_angle = right_angle
            self.prev_left_angle = left_angle
            return
        
        # Compute delta angles
        delta_theta_right = right_angle - self.prev_right_angle
        delta_theta_left = left_angle - self.prev_left_angle
        
        # Update previous angles
        self.prev_right_angle = right_angle
        self.prev_left_angle = left_angle
        
        # Compute displacement and orientation change
        displacement = ((delta_theta_right + delta_theta_left) / (2 * 360)) * self.circumference
        if -1 < displacement < 1:
            displacement = 0

        orientation_change = ((delta_theta_left - delta_theta_right) / (2 * 360)) * (self.circumference / self.turning_circumference) * 360
        if -3 < orientation_change < 3:
            orientation_change = 0
        
        # Update position
        self.theta += orientation_change
        self.theta %= 360
        self.x += displacement * np.cos(np.radians(self.theta))
        self.y += displacement * np.sin(np.radians(self.theta))
        
        # Publish updated position
        position_msg = Float32MultiArray()
        position_msg.data = [self.x, self.y, self.theta]
        self.publisher.publish(position_msg)
        
        self.get_logger().info(f'Position: x={self.x:.3f}, y={self.y:.3f}, theta={self.theta:.2f}')
        # Convert theta to quaternion
        quaternion = quaternion_from_euler(0, 0, np.radians(self.theta))  # Roll, Pitch, Yaw

        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0  # 2D plane
        
        odom_msg.pose.pose.orientation = Quaternion(
            x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3]
        )

        self.odom_publisher.publish(odom_msg)

        self.get_logger().info(f'Odometry: x={self.x:.3f}, y={self.y:.3f}, theta={self.theta:.2f}')
        
        
    def imu_listener_callback(self, msg):
        imu_message = msg.data[0]
        if imu_message <= 360:
            self.get_logger().info('I heard: "%s"' % imu_message)

def main(args=None):
    rclpy.init(args=args)
    node = CarPositionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
