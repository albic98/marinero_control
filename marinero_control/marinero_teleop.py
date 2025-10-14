#!usr/bin/env python3

import time
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Int32, Bool

joy_vel_msg = Twist()   # robot velocity
nav_vel_msg = Twist()   # autonomous velocity

class MarineroTeleop(Node):

    def __init__(self):
        super().__init__("marinero_teleop")

        self.scale_linear_x = self.declare_parameter("scale_linear_x", 10).value
        self.scale_linear_y = self.declare_parameter("scale_linear_y", 8).value
        self.scale_linear_turbo = self.declare_parameter("scale_linear_turbo", 15).value
        self.scale_angular_z = self.declare_parameter("scale_angular_z", 5).value
        self.scale_angular = self.declare_parameter("scale_angular", 0.02).value

        self.mode_selection = Int32()
        self.mode_selection.data = 0
        self.camera_positions = np.array([0.0, 0.0], float)    # initial camera positions
        self.nav_vel_x, self.nav_vel_y, self.nav_vel_z = 0.0, 0.0, 0.0
        self.manual_control_active = True

        self.joy_publisher = self.create_publisher(Twist, "/cmd_vel_joy", 10)
        self.camera_publisher = self.create_publisher(Float64MultiArray, "/camera_position", 10)
        self.mode_selection_publisher = self.create_publisher(Int32, "/mode_selection", 10)
        self.joy_subscription = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.nav_subscription = self.create_subscription(Twist, "/cmd_vel", self.navigation_callback, 10)
        self.take_photo_subscription = self.create_subscription(Bool, "/take_photo", self.take_photo_callback_2, 10)
        self.photo_cmd_vel_subscription = self.create_subscription(Twist, "/cmd_vel_photo", self.photo_cmd_vel_callback, 10)
        self.take_photo = False 
        self.rotate_robot = 0.0

    def take_photo_callback_2(self, msg):
        self.take_photo = bool(msg.data)  

    def photo_cmd_vel_callback(self, msg):
        self.rotate_robot = msg.angular.z

    def navigation_callback(self, msg):
        nav_vel_msg = msg
        if not msg:
            self.nav_vel_x = 0.0
            self.nav_vel_y = 0.0
            self.nav_vel_z = 0.0
        else:
            self.nav_vel_x = nav_vel_msg.linear.x  
            self.nav_vel_y = nav_vel_msg.linear.y
            self.nav_vel_z = nav_vel_msg.angular.z

    def joy_callback(self, msg):
        global joy_vel_msg

        # Check manual control buttons
        if msg.buttons[4] == 1 or msg.buttons[5] == 1:
            self.manual_control_active = True
        else:
            if self.manual_control_active:
                # Ramp down joystick velocity to zero before switching to nav
                self.smooth_manual_to_nav()
                self.manual_control_active = False

        # Driving modes
        if(msg.buttons[4] == 1 and msg.buttons[5] == 0):    # Opposite phase --> L1 button of PS4 controller
            self.mode_selection.data = 2
            joy_vel_msg.linear.y = 0.0
        elif(msg.buttons[5] == 1 and msg.buttons[4] == 0):  # In-phase --> R1 button of PS4 controller
            self.mode_selection.data = 1
            joy_vel_msg.linear.y = msg.axes[0] * self.scale_linear_y
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
        if self.take_photo and msg.axes[3] == 0:
            joy_vel_msg.angular.z = self.rotate_robot
        else:
            joy_vel_msg.angular.z = msg.axes[3] * self.scale_angular_z if msg.axes[3] != 0 else 0.0

        # Publish joystick velocity if manual control
        if self.manual_control_active or self.mode_selection.data != 4:
            self.joy_publisher.publish(joy_vel_msg)

        # Camera control
        if not self.take_photo or (msg.axes[6] != 0.0 or msg.axes[7] != 0.0):
            self.camera_positions[0] = msg.axes[6] * self.scale_angular
            self.camera_positions[1] = - msg.axes[7] * self.scale_angular
            camera_pos_array = Float64MultiArray(data=self.camera_positions)
            self.camera_publisher.publish(camera_pos_array)


    def smooth_manual_to_nav(self, ramp_time=1.0, rate_hz=20):
        dt = 1.0 / rate_hz
        steps = int(ramp_time * rate_hz)
        vel_x = joy_vel_msg.linear.x
        vel_y = joy_vel_msg.linear.y
        vel_z = joy_vel_msg.angular.z

        for i in range(steps):
            factor = 1.0 - (i + 1) / steps
            joy_vel_msg.linear.x = vel_x * factor
            joy_vel_msg.linear.y = vel_y * factor
            joy_vel_msg.angular.z = vel_z * factor
            self.joy_publisher.publish(joy_vel_msg)
            time.sleep(dt)


        # After ramping down, take navigation velocity
        joy_vel_msg.linear.x = self.nav_vel_x
        joy_vel_msg.linear.y = self.nav_vel_y
        joy_vel_msg.angular.z = self.nav_vel_z
        self.joy_publisher.publish(joy_vel_msg)


def main(args=None):

    rclpy.init(args=args)
    marinero_teleop = MarineroTeleop()
    rclpy.spin(marinero_teleop)
    marinero_teleop.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

