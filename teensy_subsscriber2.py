import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
from rclpy.qos import QoSProfile, ReliabilityPolicy

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node1')

        # QoS settings
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        # Subscribers of angles and IMU from the car
        self.subscription = self.create_subscription(Float32MultiArray, '/car_angles', self.car_angles_callback, qos_profile)
        self.imu_subscription = self.create_subscription(Float32MultiArray, '/imu_euler', self.imu_listener_callback, qos_profile)

        # Publishers current position, initialize car position and odometry
        self.movement_publisher = self.create_publisher(Float32MultiArray, '/movement', 10)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Car state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Initial orientation
        self.theta_imu = None # initial imu theta for sensor fusion
        self.theta_odom = 0.0 # initial odom theta for sensor fusion
        self.orientation_change = 0.0
        self.prev_right_angle = None
        self.prev_left_angle = None
        
        # Robot parameters 
        self.circumference = 20.892  
        self.turning_circumference = 33.929 

        # # IMU reference
        self.last_valid_heading = None
        self.initial_imu_heading = None  

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

        self.orientation_change = ((delta_theta_left - delta_theta_right) / (2 * 360)) * (self.circumference / self.turning_circumference) * 360
        if -3 < self.orientation_change < 3:
            self.orientation_change = 0
        
        # Update odometry theta
        self.theta_odom += self.orientation_change
        self.theta_odom %= 360

        # Update position using fused theta
        self.x += displacement * np.cos(np.radians(self.theta_odom))
        self.y += displacement * np.sin(np.radians(self.theta_odom))
        
        # Update raw odometry theta
        self.get_logger().info(f'Original Encoder Orientation: {self.theta_odom:.2f}')

    def fuse_angles(self, theta_odom, theta_imu, weight_odom, weight_imu):
        # Convert angles to radians
        rad_odom = np.radians(theta_odom)
        rad_imu = np.radians(theta_imu)

        # Turn into unit vectors
        x = weight_odom * np.cos(rad_odom) + weight_imu * np.cos(rad_imu)
        y = weight_odom * np.sin(rad_odom) + weight_imu * np.sin(rad_imu)

        # Convert back to angle
        fused_rad = np.arctan2(y, x)
        fused_deg = np.degrees(fused_rad) % 360
        return fused_deg

    def imu_listener_callback(self, msg):
        
        # Extract heading value
        current_heading = msg.data[0]  

        if self.initial_imu_heading is None:
            self.initial_imu_heading = current_heading  
            # self.last_valid_heading = current_heading
            self.get_logger().info(f'Storing initial IMU heading: {self.initial_imu_heading:.2f}')

        if not hasattr(self, 'last_imu_raw'):
            self.last_imu_raw = current_heading

        imu_diff = abs(current_heading - self.last_imu_raw)
        self.last_imu_raw = current_heading

        
        # Determine if IMU reading should be used
        use_imu = True
        
        # Check for the noise deviation
        if 7.5 <= imu_diff <= 8.5:
            self.get_logger().warn(f'Ignored IMU noise spike: {current_heading:.2f}° (Δ ~8.0°)')
            use_imu = False
            
        # Filter small drift
        elif imu_diff < 1.0:
            self.get_logger().info(f'Suppressed small drift: Δ={imu_diff:.2f}°')
            use_imu = False

        # If valid, update IMU heading
        if use_imu:
            # self.last_valid_heading = current_heading
            normalized_heading = (current_heading - self.initial_imu_heading) % 360
            self.theta_imu = normalized_heading
            self.get_logger().info(f'IMU heading accepted: {normalized_heading:.2f}°')
        else:
            self.get_logger().info(f'IMU ignored; keeping last theta_imu: {self.theta_imu}')
        

        # Fuse theta
        if self.theta_imu is not None:
            if self.orientation_change > 0:  # Turning Right
                self.theta = self.fuse_angles(self.theta_odom, self.theta_imu, 0.8, 0.2)
            elif self.orientation_change < 0:  # Turning Left
                self.theta = self.fuse_angles(self.theta_odom, self.theta_imu, 0.2, 0.8)
            else:  # Straight
                self.theta = self.theta_odom
        else:
            self.theta = self.theta_odom
        
        # Publish odometry
        self.publish_odometry()

    def publish_odometry(self):
        quat = quaternion_from_euler(0, 0, np.radians(self.theta))
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0 # 2D plane

        odom.pose.pose.orientation = Quaternion(
            x=quat[0], y=quat[1], z=quat[2], w=quat[3]
        )
        self.odom_publisher.publish(odom)
        
        # Broadcast TF transformation
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_link"
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = quat[0]
        tf_msg.transform.rotation.y = quat[1]
        tf_msg.transform.rotation.z = quat[2]
        tf_msg.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(tf_msg)

        self.get_logger().info(f'Odometry published: x={self.x:.2f}, y={self.y:.2f}, θ={self.theta:.2f}°')



def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
