#!/usr/bin/env python3

import rclpy
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.node import Node
from sensor_msgs.msg import Joy

class Camera(Node):
    def __init__(self):
        super().__init__("camera")
        self.camera_trajectory_publisher = self.create_publisher(JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10)
        self.joy_subscriber = self.create_subscription(Joy, "/joy", self.joy_callback, 10)

        self.joint_names = ["camera_base_joint", "left_camera_joint", "right_camera_joint"]
        self.positions = [0.0, 1.5, 1.5]  # Initial positions
        self.axis_flag = Joy()
        self.axis_flag.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                          
    def joy_callback(self, msg):
        self.axis_flag = msg

        if msg.axes[6] == 1:   # LEFT
            self.positions[0] = max(min(self.positions[0] - 0.05, math.pi), -math.pi)
        elif msg.axes[6] == -1:  # RIGHT
            self.positions[0] = max(min(self.positions[0] + 0.05, math.pi), -math.pi)
        
        if msg.axes[7] == 1:   # UP
            self.positions[1] = max(min(self.positions[1] - 0.05, 2.0), 1.0)
            self.positions[2] = max(min(self.positions[2] - 0.05, 2.0), 1.0)
        elif msg.axes[7] == -1:  # DOWN
            self.positions[1] = max(min(self.positions[1] + 0.05, 2.0), 1.0)
            self.positions[2] = max(min(self.positions[2] + 0.05, 2.0), 1.0)
        self.send_trajectory()

    def send_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.positions
        msg.points = [point]

        condition_0 = self.axis_flag.axes[6] == 0
        condition_1 = self.axis_flag.axes[7] == 0

        if condition_0 and condition_1:
            return
        else:
            self.camera_trajectory_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    camera_node = Camera()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()