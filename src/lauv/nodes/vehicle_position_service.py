#!/usr/bin/env python3
"""
Service-based vehicle position/orientation controller.
Provides a service to set vehicle pose and publishes the transform.
"""
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped, Pose
from std_srvs.srv import Empty
import math


class VehiclePositionService(Node):
    def __init__(self):
        super().__init__('vehicle_position_service')
        
        # TF2 broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Timer for publishing current transform
        self.timer = self.create_timer(0.1, self.publish_current_transform)
        
        # Current vehicle pose
        self.current_pose = Pose()
        self.current_pose.position.x = 0.0
        self.current_pose.position.y = 0.0
        self.current_pose.position.z = -5.0  # Start underwater
        self.current_pose.orientation.w = 1.0
        
        # Service to move to specific position
        self.move_service = self.create_service(
            Empty, 
            '/lauv/move_to_position', 
            self.move_to_position_callback
        )
        
        # Parameters for different preset positions
        self.declare_parameter('preset_positions', [
            [0.0, 0.0, -5.0, 0.0],      # [x, y, z, yaw]
            [10.0, 0.0, -5.0, 1.57],    # Move forward and turn right
            [10.0, 10.0, -10.0, 3.14],  # Move to corner and turn around
            [0.0, 10.0, -5.0, 4.71],    # Complete square pattern
        ])
        
        self.current_preset = 0
        self.preset_positions = self.get_parameter('preset_positions').value
        
        self.get_logger().info('Vehicle Position Service started.')
        self.get_logger().info('Call service: ros2 service call /lauv/move_to_position std_srvs/srv/Empty')
        
    def move_to_position_callback(self, request, response):
        """Service callback to move to next preset position"""
        position = self.preset_positions[self.current_preset]
        
        # Update current pose
        self.current_pose.position.x = position[0]
        self.current_pose.position.y = position[1] 
        self.current_pose.position.z = position[2]
        
        # Convert yaw to quaternion
        yaw = position[3]
        self.current_pose.orientation.x = 0.0
        self.current_pose.orientation.y = 0.0
        self.current_pose.orientation.z = math.sin(yaw / 2.0)
        self.current_pose.orientation.w = math.cos(yaw / 2.0)
        
        self.get_logger().info(f'Moving to position {self.current_preset + 1}: '
                              f'x={position[0]:.1f}, y={position[1]:.1f}, '
                              f'z={position[2]:.1f}, yaw={position[3]:.2f}')
        
        # Move to next preset for next call
        self.current_preset = (self.current_preset + 1) % len(self.preset_positions)
        
        return response
        
    def publish_current_transform(self):
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
    
    service_node = VehiclePositionService()
    
    try:
        rclpy.spin(service_node)
    except KeyboardInterrupt:
        service_node.get_logger().info('Vehicle Position Service stopped')
    finally:
        service_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()