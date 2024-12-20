#!usr/bin/env python3

import math
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

vel_msg = Twist()  # robot velocity

class Commander(Node):

    def __init__(self):
        super().__init__("commander")
        timer_period = 0.0225
        self.wheel_seperation = 0.425
        self.wheel_base = 0.8
        self.wheel_radius = 0.1016
        self.wheel_steering_y_offset = 0.0
        self.steering_track = self.wheel_seperation - 2*self.wheel_steering_y_offset

        self.scale_linear_x = self.declare_parameter("scale_linear_x", 10).value
        self.scale_linear_y = self.declare_parameter("scale_linear_y", 8).value
        self.scale_linear_turbo = self.declare_parameter("scale_linear_turbo", 20).value
        self.scale_angular_z = self.declare_parameter("scale_angular_z", 5).value
        self.scale_angular = self.declare_parameter("scale_angular", 0.01).value

        self.mode_selection = 0             # 1: opposite phase, 2: in-phase, 3: pivot turn 4: none
        self.axle_adjustment_speed = 0.025  # rotation speed of axles when returning to initial positions

        self.pos = np.array([0,0,0,0,0,1.5,1.5], float)
        self.vel = np.array([0,0,0,0], float)                   # left_front, right_front, left_rear, right_rear
        self.target_pos = np.array([0,0,0,0], float)            # target positions for the wheels when the robot is standing still
        self.max_axle_position = math.pi/4                      # radians
        self.camera_base_turn, self.camera_turn = [0.0, 0.0]    # initial camera positions

        self.cmd_publisher = self.create_publisher(Twist, "/fcc/cmd_vel", 10)
        self.joy_subscription = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.pub_pos = self.create_publisher(Float64MultiArray, "/forward_position_controller/commands", 10)
        self.pub_vel = self.create_publisher(Float64MultiArray, "/forward_velocity_controller/commands", 10)

        self.command_timer = self.create_timer(timer_period, self.timer_callback)


    def joy_callback(self, msg):

        global vel_msg

        if(msg.buttons[4] == 1 and msg.buttons[5] == 0):    # opposite phase --> L1 button of PS4 controller
            self.mode_selection = 2
            vel_msg.linear.y = 0.0
        elif(msg.buttons[5] == 1 and msg.buttons[4] == 0):  # in-phase --> R1 button of PS4 controller
            self.mode_selection = 1
            vel_msg.linear.y = - msg.axes[0] * self.scale_linear_y
        elif(msg.buttons[4] == 1 and msg.buttons[5] == 1):  # pivot turn --> L1 and R1 button of PS4 controller
            self.mode_selection = 3
            vel_msg.linear.y = 0.0
        else:
            self.mode_selection = 0
            vel_msg.linear.x = 0.0
            vel_msg.linear.y = 0.0
            vel_msg.angular.z = 0.0

        if (msg.buttons[0] == 1 or msg.buttons[6] == 1) and msg.axes[1] > 0:
            vel_msg.linear.x = msg.axes[1] * self.scale_linear_x + msg.buttons[0] * self.scale_linear_turbo + msg.buttons[6] * self.scale_linear_turbo
        else:
            vel_msg.linear.x = msg.axes[1] * self.scale_linear_x

        if msg.axes[3] == 0:
            vel_msg.angular.z = 0.0
        else:
            vel_msg.angular.z = msg.axes[3] * self.scale_angular_z

        self.cmd_publisher.publish(vel_msg)

        self.camera_base_turn = - msg.axes[6] * self.scale_angular
        self.camera_turn = - msg.axes[7] * self.scale_angular


    def timer_callback(self):

        global vel_msg

        self.pos[4] += self.camera_base_turn
        self.pos[5] += self.camera_turn
        self.pos[6] += self.camera_turn

        if (self.mode_selection == 1):
            self.in_phase_steering(vel_msg)
            
        elif (self.mode_selection == 2):
            self.opposite_phase_steering(vel_msg)
            
        elif (self.mode_selection == 3):
            self.pivot_turn(vel_msg)
            
        else:
            self.previous_angle = 0.0
            self.initial_sign_x = 0.0
            self.entered_function_flag = False

            self.target_pos[:4] = 0.0
            if self.vel[0] > 5.0:
                # print(f"\nBraking: {[f'{v:.4f}' for v in self.vel[:4]]}")
                self.vel[:4] += - 0.05 * self.vel[0]
            else:
                self.vel[:4] = 0.0
        
        # Slowly adjust the current position to the target position
        for i in range(4):
            if abs(self.pos[i] - self.target_pos[i]) > self.axle_adjustment_speed:
                self.pos[i] += self.axle_adjustment_speed * np.sign(self.target_pos[i] - self.pos[i])
            else:
                self.pos[i] = self.target_pos[i]

        pos_array = Float64MultiArray(data=self.pos)
        vel_array = Float64MultiArray(data=self.vel)
        self.pub_pos.publish(pos_array)
        self.pub_vel.publish(vel_array)
        self.target_pos[:4] = 0


    def in_phase_steering(self, vel_msg):
        if self.initial_sign_x == 0.0 or self.entered_function_flag == False: 
            self.initial_sign_x = np.sign(vel_msg.linear.x)
            self.entered_function_flag = True

        sign_x = self.initial_sign_x
        sign_y = np.sign(vel_msg.linear.y)

        if np.sign(vel_msg.linear.x) != self.initial_sign_x:
            vel_msg.linear.x = 0.0
            vel_msg.linear.y = math.copysign(self.scale_linear_y, sign_y)
            axle_angle = self.previous_angle
        elif vel_msg.linear.x != 0:
            axle_angle = math.atan(vel_msg.linear.y / vel_msg.linear.x)
            self.previous_angle = axle_angle
        else:
            axle_angle = 0.0

        V = math.hypot(vel_msg.linear.x, vel_msg.linear.y)  # Combined linear velocity magnitude
        self.pos[:4] = axle_angle
        self.vel[:] = sign_x * V


    def opposite_phase_steering(self, vel_msg):
        vel_steerring_offset = vel_msg.angular.z * self.wheel_steering_y_offset
        sign = np.sign(vel_msg.linear.x)

        self.vel[0] = sign*math.hypot(vel_msg.linear.x - vel_msg.angular.z*self.steering_track/2, vel_msg.angular.z*self.wheel_base/2) - vel_steerring_offset
        self.vel[1] = sign*math.hypot(vel_msg.linear.x + vel_msg.angular.z*self.steering_track/2, vel_msg.angular.z*self.wheel_base/2) + vel_steerring_offset
        self.vel[2] = sign*math.hypot(vel_msg.linear.x - vel_msg.angular.z*self.steering_track/2, vel_msg.angular.z*self.wheel_base/2) - vel_steerring_offset
        self.vel[3] = sign*math.hypot(vel_msg.linear.x + vel_msg.angular.z*self.steering_track/2, vel_msg.angular.z*self.wheel_base/2) + vel_steerring_offset

        if vel_msg.linear.x == 0.0:
            self.target_pos[0] = 0.0
            self.target_pos[1] = 0.0
            self.target_pos[2] = 0.0
            self.target_pos[3] = 0.0
        elif vel_msg.angular.z < 0:
            self.pos[0] = -1.5*math.atan(vel_msg.angular.z*self.wheel_base/(2*abs(vel_msg.linear.x) + vel_msg.angular.z*self.steering_track))
            if abs(self.pos[0]) > self.max_axle_position:
                self.pos[0] = self.max_axle_position
            self.axle_positions()
        else:
            self.pos[0] = -1.5*math.atan(vel_msg.angular.z*self.wheel_base/(2*abs(vel_msg.linear.x) - vel_msg.angular.z*self.steering_track))
            if abs(self.pos[0]) > self.max_axle_position:
                self.pos[0] = -self.max_axle_position
            self.axle_positions()


    def pivot_turn(self, vel_msg):
        self.target_pos[0] = math.atan(self.wheel_base / self.steering_track)
        self.target_pos[1] = - math.atan(self.wheel_base / self.steering_track)
        self.target_pos[2] = - math.atan(self.wheel_base / self.steering_track)
        self.target_pos[3] = math.atan(self.wheel_base / self.steering_track)

        if self.target_pos[0] == self.pos[0]:
            self.vel[0] = - vel_msg.angular.z
            self.vel[1] = vel_msg.angular.z
            self.vel[2] = self.vel[0]
            self.vel[3] = self.vel[1]
        else:
            self.vel[:4] = 0

    def axle_positions(self):
        self.pos[1] = self.pos[0]
        self.pos[2] = - self.pos[0]
        self.pos[3] = - self.pos[0]


def main(args=None):

    rclpy.init(args=args)
    commander = Commander()
    rclpy.spin(commander)
    commander.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

