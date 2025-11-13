#!/usr/bin/env python3
"""
Interactive vehicle controller that listens to Pose messages.
Allows real-time control of vehicle position and orientation.
"""
import math
import rclpy
import tf2_ros

from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Pose, PoseStamped, Twist


class VehicleController(Node):
    def __init__(self):
        super().__init__('vehicle_controller')
        
        # TF2 broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Timer for publishing transforms
        self.timer = self.create_timer(0.05, self.publish_transform)  # 20 Hz
        
        # Current vehicle pose
        self.current_pose = Pose()
        self.current_pose.position.z = -5.0  # Start underwater
        self.current_pose.orientation.w = 1.0
        
        # Subscribers for different control methods
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/lauv/target_pose',
            self.pose_callback,
            10
        )
        
        self.twist_sub = self.create_subscription(
            Twist,
            '/lauv/cmd_vel',
            self.twist_callback,
            10
        )
        
        # For velocity control
        self.last_time = self.get_clock().now()
        
        self.get_logger().info('Vehicle Controller started.')
        self.get_logger().info('Subscribe to:')
        self.get_logger().info('  /lauv/target_pose (geometry_msgs/PoseStamped) - for absolute positioning')
        self.get_logger().info('  /lauv/cmd_vel (geometry_msgs/Twist) - for velocity control')
        
    def pose_callback(self, msg):
        """Handle absolute pose commands"""
        self.current_pose = msg.pose
        self.get_logger().info(f'New target pose: x={msg.pose.position.x:.2f}, '
                              f'y={msg.pose.position.y:.2f}, z={msg.pose.position.z:.2f}')
        
    def twist_callback(self, msg):
        """Handle velocity commands for incremental movement"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Convert twist to pose update
        # This is a simplified integration - in real systems you'd use proper odometry
        
        # Extract current orientation (yaw)
        current_yaw = 2 * math.atan2(self.current_pose.orientation.z, 
                                    self.current_pose.orientation.w)
        
        # Update position based on velocity in vehicle frame
        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)
        
        # Transform velocity from body frame to world frame
        vel_x_world = msg.linear.x * cos_yaw - msg.linear.y * sin_yaw
        vel_y_world = msg.linear.x * sin_yaw + msg.linear.y * cos_yaw
        
        self.current_pose.position.x += vel_x_world * dt
        self.current_pose.position.y += vel_y_world * dt
        self.current_pose.position.z += msg.linear.z * dt
        
        # Update yaw
        new_yaw = current_yaw + msg.angular.z * dt
        self.current_pose.orientation.z = math.sin(new_yaw / 2.0)
        self.current_pose.orientation.w = math.cos(new_yaw / 2.0)
        
    def publish_transform(self):
        """Publish the current vehicle transform"""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'world'
        transform.child_frame_id = 'lauv/base_link'
        
        transform.transform.translation.x = self.current_pose.position.x
        transform.transform.translation.y = self.current_pose.position.y
        transform.transform.translation.z = self.current_pose.position.z
        
        transform.transform.rotation = self.current_pose.orientation
        
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    
    controller_node = VehicleController()
    
    try:
        rclpy.spin(controller_node)
    except KeyboardInterrupt:
        controller_node.get_logger().info('Vehicle Controller stopped')
    finally:
        controller_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()