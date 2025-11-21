#!/usr/bin/env python3
import rclpy
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.node import Node
from std_msgs.msg import Header
from pynput import keyboard


class Camera(Node):
    def __init__(self):
        super().__init__("camera")
        self.camera_pose_publisher = self.create_publisher(JointTrajectory, "/set_joint_trajectory", 10)
        self.cam_base = 0.0   # Initial value for cam_base
        self.l_cam = 1.5      # Initial value for l_cam
        self.r_cam = 1.5      # Initial value for r_cam
        self.joint_names = ["camera_base_joint", "left_camera_joint", "right_camera_joint"]
        self.create_timer(0.1, self.joint_pose_publisher)  # Update every 0.1 seconds
        
        self.keyboard_listener = keyboard.Listener(
            on_press=self.on_press
        )
        self.keyboard_listener.start()

    def on_press(self, key):  # sourcery skip: use-contextlib-suppress
        try:
            if key.char == '4': # key == keyboard.Key.left:
                self.adjust_joint("cam_base", -0.15)
            elif key.char == '6': # key == keyboard.Key.right:
                self.adjust_joint("cam_base", 0.15)
            elif key.char == '8': # key == keyboard.Key.up:
                self.adjust_joint("l_cam", -0.15)
                self.adjust_joint("r_cam", -0.15)
            elif key.char == '2': # key == keyboard.Key.down:
                self.adjust_joint("l_cam", 0.15)
                self.adjust_joint("r_cam", 0.15)
        except AttributeError:
            # Special keys (e.g., arrow keys) don't have 'char' attribute
            pass

    def adjust_joint(self, joint, delta):
        if joint == "cam_base":
            self.cam_base = max(min(self.cam_base + delta, math.pi), -math.pi)
        elif joint == "l_cam":
            self.l_cam = max(min(self.l_cam + delta, 2.0), 1.0)
        elif joint == "r_cam":
            self.r_cam = max(min(self.r_cam + delta, 2.0), 1.0)
        # self.get_logger().info(f'Adjusted {joint} to {getattr(self, joint)}')

    def joint_pose_publisher(self):
        msg = JointTrajectory()
        msg.header = Header()
        msg.header.frame_id = "base_link"
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = [self.cam_base, self.l_cam, self.r_cam]
        
        msg.points = [point]
        
        self.camera_pose_publisher.publish(msg)
        # self.get_logger().info(f'Postavljene pozicije: {self.cam_base, self.l_cam, self.r_cam}')


def main(args=None):
    rclpy.init(args=args)
    
    camera_node = Camera()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()   

if __name__ == "__main__":
    main()