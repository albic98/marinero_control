#!usr/bin/env python3

import math
import rclpy
import time
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler

class DockingStation(Node):
    def __init__(self):
        super().__init__('docking_station')
        self.goal_pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.docking_station_position = np.array([197.212, 280.830, 0, 0, 0, -3.0755])

    def go_to_docking_station(self, x, y, z, roll, pitch, yaw):
        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = z

        quaternion_orientation = quaternion_from_euler(roll, pitch, yaw)
        goal_pose.pose.orientation.x = quaternion_orientation[0]
        goal_pose.pose.orientation.y = quaternion_orientation[1]
        goal_pose.pose.orientation.z = quaternion_orientation[2]
        goal_pose.pose.orientation.w = quaternion_orientation[3]
        self.goal_pose_publisher.publish(goal_pose)
        self.get_logger().info(f'Published docking station goal: ({x}, {y}, {z})')

def main(args=None):
    rclpy.init(args=args)
    docking_station = DockingStation()

    time.sleep(1.0)

    docking_station.go_to_docking_station(*docking_station.docking_station_position)

    rclpy.spin(docking_station)
    docking_station.destroy_node()
    rclpy.shutdown()
