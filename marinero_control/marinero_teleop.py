#!usr/bin/env python3

import time
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Int32, Bool, Float64

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
        self.camera_positions = np.array([0.0, 1.5], float)    # initial camera positions
        self.nav_vel_x, self.nav_vel_y, self.nav_vel_z = 0.0, 0.0, 0.0
        self.photo_vel_z = 0.0
        self.manual_control_active = True
        self.boat_searching = False 
        self.joy_vel_msg = Twist()

        self.joy_publisher = self.create_publisher(Twist, "/cmd_vel_joy", 10)
        self.camera_publisher = self.create_publisher(Float64MultiArray, "/camera_position", 10)
        self.mode_selection_publisher = self.create_publisher(Int32, "/mode_selection", 10)
        
        self.joy_subscription = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.nav_subscription = self.create_subscription(Twist, "/cmd_vel", self.navigation_callback, 10)
        self.boat_searching_subscription = self.create_subscription(Bool, "/boat_searching", self.boat_searching_callback_2, 10)
        self.photo_speed_subscription = self.create_subscription(Float64, "/photo_angular_speed", self.photo_speed_callback, 10)

    def boat_searching_callback_2(self, msg: Bool):
        self.boat_searching = msg.data

    def photo_speed_callback(self, msg: Float64):
        self.photo_vel_z = msg.data

    def navigation_callback(self, msg: Twist):
        nav_vel_msg = msg
        self.nav_vel_x = nav_vel_msg.linear.x  
        self.nav_vel_y = nav_vel_msg.linear.y
        self.nav_vel_z = nav_vel_msg.angular.z

    def joy_callback(self, msg):

        # Check manual control buttons
        if msg.buttons[4] == 1 or msg.buttons[5] == 1:
            self.manual_control_active = True
        elif self.manual_control_active:
            # Ramp down joystick velocity to zero before switching to nav
            self.smooth_manual_to_nav()
            self.manual_control_active = False

        # Driving modes
        if(msg.buttons[4] == 1 and msg.buttons[5] == 0):    # Opposite phase --> L1 button of PS4 controller
            self.mode_selection.data = 2
            self.joy_vel_msg.linear.y = 0.0
        elif(msg.buttons[5] == 1 and msg.buttons[4] == 0):  # In-phase --> R1 button of PS4 controller
            self.mode_selection.data = 1
            self.joy_vel_msg.linear.y = msg.axes[0] * self.scale_linear_y
        elif(msg.buttons[4] == 1 and msg.buttons[5] == 1):  # Pivot turn --> L1 and R1 button of PS4 controller
            self.mode_selection.data = 3
            self.joy_vel_msg.linear.y = 0.0
        elif self.nav_vel_x != 0.0 or self.nav_vel_y != 0.0 or self.nav_vel_z != 0.0:
            self.mode_selection.data = 4
        else:
            self.mode_selection.data = 0
            self.joy_vel_msg = Twist()

        # Publish mode selection
        self.mode_selection_publisher.publish(self.mode_selection)

        # Linear X speed (with turbo boost)
        if (msg.buttons[0] == 1 or msg.buttons[6] == 1) and msg.axes[1] > 0:
            self.joy_vel_msg.linear.x = msg.axes[1] * self.scale_linear_x + msg.buttons[0] * self.scale_linear_turbo + msg.buttons[6] * self.scale_linear_turbo
        else:
            self.joy_vel_msg.linear.x = msg.axes[1] * self.scale_linear_x

        # Angular Z speed (steering)
        if self.boat_searching and msg.axes[3] == 0:
            self.joy_vel_msg.angular.z = self.photo_vel_z
        else:
            self.joy_vel_msg.angular.z = msg.axes[3] * self.scale_angular_z if msg.axes[3] != 0 else 0.0

        # Publish joystick velocity if manual control
        if self.manual_control_active or self.mode_selection.data != 4:
            self.joy_publisher.publish(self.joy_vel_msg)

        # Camera control
        if not self.boat_searching or (msg.axes[6] != 0.0 or msg.axes[7] != 0.0):
            self.camera_positions[0] = msg.axes[6] * self.scale_angular
            self.camera_positions[1] = - msg.axes[7] * self.scale_angular
        camera_msg = Float64MultiArray(data=self.camera_positions)
        self.camera_publisher.publish(camera_msg)


    def smooth_manual_to_nav(self, ramp_time=1.0, rate_hz=20):
        dt = 1.0 / rate_hz
        steps = int(ramp_time * rate_hz)
        start_vel = Twist()
        start_vel.linear.x = self.joy_vel_msg.linear.x
        start_vel.linear.y = self.joy_vel_msg.linear.y
        start_vel.angular.z = self.joy_vel_msg.angular.z

        for i in range(steps):
            factor = 1.0 - (i + 1) / steps
            self.joy_vel_msg.linear.x = start_vel.linear.x * factor
            self.joy_vel_msg.linear.y = start_vel.linear.y * factor
            self.joy_vel_msg.angular.z = start_vel.angular.z * factor
            self.joy_publisher.publish(self.joy_vel_msg)
            time.sleep(dt)

        # Switch to navigation velocity
        self.joy_vel_msg.linear.x = self.nav_vel_x
        self.joy_vel_msg.linear.y = self.nav_vel_y
        self.joy_vel_msg.angular.z = self.nav_vel_z
        self.joy_publisher.publish(self.joy_vel_msg)


def main(args=None):

    rclpy.init(args=args)
    marinero_teleop = MarineroTeleop()
    rclpy.spin(marinero_teleop)
    marinero_teleop.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

