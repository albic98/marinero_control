#!/usr/bin/env python3

import sys
import rclpy
import math
from pynput import keyboard
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.node import Node
from std_msgs.msg import Header


class Camera(Node):
    def __init__(self,  cam_base, l_cam, r_cam):
        super().__init__("camera")
        self.camera_pose_publisher = self.create_publisher(JointTrajectory, "/set_joint_trajectory", 10)
        self.cam_base = max(min(cam_base, math.pi), -math.pi)   # Ensure cam_base is within the range [-pi, pi]
        self.l_cam = max(min(l_cam, 2.0), 1.0)                  # Ensure l_cam is within the range [1, 2]
        self.r_cam = max(min(r_cam, 2.0), 1.0)                  # Ensure r_cam is within the range [1, 2]
        self.joint_names = ["camera_base_joint", "left_camera_joint", "right_camera_joint"]
        self.initial_publish_flag = True
        self.create_timer(1.0, self.joint_pose_publisher)
        
        # self.current_positions = {name: 0.0 for name in self.joint_names}
        # self.callback_timer = self.create_timer(20.0, self.perform_joint_state_callback)
        # self.message = None

    # def joint_state_callback(self, msg):
    #     self.message = msg

    # def perform_joint_state_callback(self):
    #     if self.message:
    #         for name, position in zip(self.message.name, self.message.position):
    #             if name in self.current_positions:
    #                 self.current_positions[name] = position
    #         self.get_logger().info(f"Trenutne pozicije: {self.current_positions}") 

    def joint_pose_publisher(self):
        msg = JointTrajectory()
        msg.header = Header()
        msg.header.frame_id = "base_link"
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = [self.cam_base, self.l_cam, self.r_cam]
        
        msg.points = [point]
        
        self.camera_pose_publisher.publish(msg)
        # self.get_logger().info(f"Objava "JointTrajectory" poruke: {msg}")
        self.get_logger().info(f"Postavljene pozicije: {self.cam_base, self.l_cam, self.r_cam}")
        

def main(args=None):
    
    rclpy.init(args=args)
    
    if len(sys.argv) != 4:
        print("Uporaba: ros2 run marinero_control camera_rotation <base> <left_camera> <right_camera>")
        return
        
    try:
        cam_base = float(sys.argv[1])
        l_cam = float(sys.argv[2])
        r_cam = float(sys.argv[3])
    except ValueError:
        print("Greška: Pozicije moraju biti brojevi.")
        return

    camera_node = Camera(cam_base, l_cam, r_cam)
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()   
        
if __name__ == "__main__":
    main()
    
    