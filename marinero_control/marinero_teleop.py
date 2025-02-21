#!usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Int32

joy_vel_msg = Twist()  # robot velocity
nav_vel_msg = Twist() # autonomous velocity

class MarineroTeleop(Node):

    def __init__(self):
        super().__init__("marinero_teleop")

        self.scale_linear_x = self.declare_parameter("scale_linear_x", 10).value
        self.scale_linear_y = self.declare_parameter("scale_linear_y", 8).value
        self.scale_linear_turbo = self.declare_parameter("scale_linear_turbo", 20).value
        self.scale_angular_z = self.declare_parameter("scale_angular_z", 5).value
        self.scale_angular = self.declare_parameter("scale_angular", 0.01).value

        self.mode_selection = Int32()
        self.mode_selection.data = 0
        self.camera_positions = np.array([0.0, 0.0], float)    # initial camera positions
        self.nav_vel_x, self.nav_vel_y, self.nav_vel_z = 0.0, 0.0, 0.0

        self.joy_publisher = self.create_publisher(Twist, "/cmd_vel_joy", 10)
        self.camera_publisher = self.create_publisher(Float64MultiArray, "/camera_position", 10)
        self.mode_selection_publisher = self.create_publisher(Int32, "/mode_selection", 10)
        self.joy_subscription = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.nav_subscription = self.create_subscription(Twist, "/cmd_vel", self.navigation_callback, 10)
    
    def navigation_callback(self, msg):
        nav_vel_msg = msg
        self.nav_vel_x = nav_vel_msg.linear.x  
        self.nav_vel_y = nav_vel_msg.linear.y
        self.nav_vel_z = nav_vel_msg.angular.z

    def joy_callback(self, msg):

        # Driving modes
        if(msg.buttons[4] == 1 and msg.buttons[5] == 0):    # Opposite phase --> L1 button of PS4 controller
            self.mode_selection.data = 2
            joy_vel_msg.linear.y = 0.0
        elif(msg.buttons[5] == 1 and msg.buttons[4] == 0):  # In-phase --> R1 button of PS4 controller
            self.mode_selection.data = 1
            joy_vel_msg.linear.y = - msg.axes[0] * self.scale_linear_y
        elif(msg.buttons[4] == 1 and msg.buttons[5] == 1):  # Pivot turn --> L1 and R1 button of PS4 controller
            self.mode_selection.data = 3
            joy_vel_msg.linear.y = 0.0
        elif self.nav_vel_x != 0.0 or self.nav_vel_y != 0.0 or self.nav_vel_z != 0.0:
            self.mode_selection.data = 4
        else:
            self.mode_selection.data = 0
            joy_vel_msg.linear.x = 0.0
            joy_vel_msg.linear.y = 0.0
            joy_vel_msg.angular.z = 0.0

        # Publish mode selection
        self.mode_selection_publisher.publish(self.mode_selection)

        # Linear X speed (with turbo boost)
        if (msg.buttons[0] == 1 or msg.buttons[6] == 1) and msg.axes[1] > 0:
            joy_vel_msg.linear.x = msg.axes[1] * self.scale_linear_x + msg.buttons[0] * self.scale_linear_turbo + msg.buttons[6] * self.scale_linear_turbo
        else:
            joy_vel_msg.linear.x = msg.axes[1] * self.scale_linear_x

        # Angular Z speed (steering)
        joy_vel_msg.angular.z = msg.axes[3] * self.scale_angular_z if msg.axes[3] != 0 else 0.0

        # Publish joystick velocity
        if self.mode_selection.data == 0 or self.mode_selection.data == 1 or self.mode_selection.data == 2 or self.mode_selection.data == 3:
            self.joy_publisher.publish(joy_vel_msg)

        # Camera control
        self.camera_positions[0] = - msg.axes[6] * self.scale_angular
        self.camera_positions[1] = - msg.axes[7] * self.scale_angular

        # Publish camera position
        camera_pos_array = Float64MultiArray(data=self.camera_positions)
        self.camera_publisher.publish(camera_pos_array)

def main(args=None):

    rclpy.init(args=args)
    marinero_teleop = MarineroTeleop()
    rclpy.spin(marinero_teleop)
    marinero_teleop.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

