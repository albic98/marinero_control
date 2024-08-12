#!/usr/bin/env python3

import math
import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Quaternion, TransformStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float64MultiArray

class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('odometry_listener')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.axle_positions = np.zeros(4)
        self.camera_positions = np.zeros(3)
        self.transforms = []
        
        self.odom_subscriber = self.create_subscription(Odometry, '/marinero/odom', self.odometry_callback, 50)
        self.pos_subscriber = self.create_subscription(Float64MultiArray, '/forward_position_controller/commands', self.position_callback,10)
        self.vel_subscriber = self.create_subscription(Float64MultiArray, '/forward_velocity_controller/commands', self.velocity_callback,10)
        
        self.broadcaster_timer = self.create_timer(0.01, self.odometry_broadcaster)
        
    def position_callback(self, msg):
        self.axle_positions = np.array(msg.data[0:4])
        self.camera_positions = np.array(msg.data[4:7])
    
    def velocity_callback(self, msg):
        self.wheel_velocities = np.array(msg.data)

    def odometry_callback(self, msg):
        current_time = msg.header.stamp  # Use a single timestamp for all transforms
        
        # Create a TransformStamped message
        odom_to_base_link = TransformStamped()
        odom_to_base_link.header.stamp = current_time
        odom_to_base_link.header.frame_id = 'odom'
        odom_to_base_link.child_frame_id = 'base_link'
        odom_to_base_link.transform.translation.x = msg.pose.pose.position.x
        odom_to_base_link.transform.translation.y = msg.pose.pose.position.y
        odom_to_base_link.transform.translation.z = msg.pose.pose.position.z
        odom_to_base_link.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(odom_to_base_link)

        transforms = []

        ## AXLE TRANSFORMS ##
        axles = [
            ('fl_axle_link', 0.4, 0.2125, 0.1016, self.axle_positions[1]),
            ('fr_axle_link', 0.4, -0.2125, 0.1016, self.axle_positions[0]),
            ('rl_axle_link', -0.4, 0.2125, 0.1016, self.axle_positions[3]),
            ('rr_axle_link', -0.4, -0.2125, 0.1016, self.axle_positions[2])
        ]

        for child_frame_id, x, y, z, yaw in axles:
            transform = TransformStamped()
            transform.header.stamp = current_time
            transform.header.frame_id = 'base_link'
            transform.child_frame_id = child_frame_id
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = z
            quat = quaternion_from_euler(0.0, 0.0, yaw)
            transform.transform.rotation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            transforms.append(transform)

        ## WHEEL TRANSFORMS ##
        wheels = [
            ('fl_wheel', 'fl_axle_link', -math.pi/2),
            ('fr_wheel', 'fr_axle_link', math.pi/2),
            ('rl_wheel', 'rl_axle_link', -math.pi/2),
            ('rr_wheel', 'rr_axle_link', math.pi/2)
        ]

        for child_frame_id, parent_frame_id, roll in wheels:
            transform = TransformStamped()
            transform.header.stamp = current_time
            transform.header.frame_id = parent_frame_id
            transform.child_frame_id = child_frame_id
            transform.transform.translation.x = 0.0
            transform.transform.translation.y = 0.0
            transform.transform.translation.z = 0.0
            quat = quaternion_from_euler(roll, 0.0, 0.0)
            transform.transform.rotation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            # transforms.append(transform)

        ## CAMERA TRANSFORMS ##
        cameras = [
            ('camera_base', 'chassis', 0.0, -0.2375, 0.43, 0.0, 0.0, -math.pi/2 - self.camera_positions[0]),
            ('left_camera_link', 'camera_base', 0.0, 0.0515, 0.1105, self.camera_positions[1], 0.0, math.pi/2),
            ('right_camera_link', 'camera_base', 0.0, -0.05, 0.1105, self.camera_positions[2], 0.0, math.pi/2),
            ('right_camera_depth_frame', 'right_camera_link', -0.09, 0.0, 0.065, 0.0, 0.0, -math.pi),
            ('left_camera_depth_frame', 'left_camera_link', 0.09, 0.01, 0.06, 0.0, 0.0, -math.pi)
        ]

        for child_frame_id, parent_frame_id, x, y, z, roll, pitch, yaw in cameras:
            transform = TransformStamped()
            transform.header.stamp = current_time
            transform.header.frame_id = parent_frame_id
            transform.child_frame_id = child_frame_id
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = z
            quat = quaternion_from_euler(roll, pitch, yaw)
            transform.transform.rotation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            transforms.append(transform)
            
            self.transforms = transforms

    def odometry_broadcaster(self):
        # Broadcast all transforms at once
        for transform in self.transforms:
            self.tf_broadcaster.sendTransform(transform)
        
def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()