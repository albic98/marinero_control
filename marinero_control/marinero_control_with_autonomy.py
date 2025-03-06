#!usr/bin/env python3

import math
import rclpy
import time
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float64MultiArray, Int32

vel_msg = TwistStamped()       # robot velocity after twist mux node

class MarineroControl(Node):

    def __init__(self):
        super().__init__("marinero_control")
        self.wheel_seperation = 0.425
        self.wheel_base = 0.8
        self.wheel_radius = 0.1016
        self.wheel_steering_y_offset = 0.0
        self.steering_track = self.wheel_seperation - 2*self.wheel_steering_y_offset

        self.alpha = 0.025  # Smoothing factor (adjust as needed)
        self.dead_zone = 0.01   # Ignore changes smaller than this
        self.scale_linear_x = 10.0  # Linear velocity x scale
        self.scale_linear_y = 8.0   # Linear velocity y scale
        self.scale_angular_z = 5.0  # Angular velocity scale
        
        self.camera_base_turn, self.camera_turn = [0.0, 0.0]    # Initial camera positions
        self.nav_vel_x, self.nav_vel_y, self.nav_vel_z = 0.0, 0.0, 0.0 # Navigation velocities
        self.filtered_vel_x, self.filtered_vel_y, self.filtered_vel_z = 0.0, 0.0, 0.0 # Filtered velocities

        self.initial_sign_x = 0.0           # Initial sign of x velocity
        self.mode_selection = 0             # 1: opposite phase, 2: in-phase, 3: pivot turn 4: none
        self.axle_adjustment_speed = 0.025  # rotation speed of axles when returning to initial positions

        self.pos = np.array([0,0,0,0,0,1.5,1.5], float)
        self.vel = np.array([0,0,0,0], float)                   # left_front, right_front, left_rear, right_rear
        self.wheel_vel = np.array([0,0,0,0], float)             # wheel velocities for feedback control
        self.target_pos = np.array([0,0,0,0], float)            # target positions for the wheels when the robot is standing still
        self.max_axle_position = math.pi/4                      # radians

        self.final_vel_subscription = self.create_subscription(TwistStamped, "/cmd_vel_stamped", self.velocity_callback, 10)
        self.camera_subscription = self.create_subscription(Float64MultiArray, "/camera_position", self.camera_pose_callback, 10)  
        self.mode_selection_subscription = self.create_subscription(Int32, "/mode_selection", self.mode_selection_callback, 10)
        self.pub_pos = self.create_publisher(Float64MultiArray, "/forward_position_controller/commands", 10)
        self.pub_vel = self.create_publisher(Float64MultiArray, "/forward_velocity_controller/commands", 10)

    def mode_selection_callback(self, msg):
        self.mode_selection = msg.data

    def camera_pose_callback(self, msg):
        camera_pose = msg.data
        self.camera_base_turn = camera_pose[0]  # Base turn angle
        self.camera_turn = camera_pose[1]       # Camera turn angle  

    def velocity_callback(self, msg):
        vel_msg = msg
        
        self.pos[4] += self.camera_base_turn
        self.pos[5] += self.camera_turn
        self.pos[6] += self.camera_turn

        if self.mode_selection == 1:
            self.in_phase_steering(vel_msg)
        elif self.mode_selection == 2:
            self.opposite_phase_steering(vel_msg)
        elif self.mode_selection == 3:
            self.pivot_turn(vel_msg)
        elif self.mode_selection == 4:
            self.autonomy_control(vel_msg)
        else:
            self.previous_angle = 0.0
            self.initial_sign_x = 0.0
            self.entered_function_flag = False
            self.target_pos[:4] = 0.0
            if self.vel[0] > 15.0:
                self.vel[:4] += - 0.5
            elif self.vel[0] < 15.0 and self.vel[0] > 5.0:
                self.vel[:4] += - 0.1 * self.vel[0]
            else:
                self.vel[:4] = 0.0

        for i in range(4):
            target_delta = self.target_pos[i] - self.pos[i]
            if abs(target_delta) > self.axle_adjustment_speed:
                    self.pos[i] += self.axle_adjustment_speed * np.sign(target_delta)
            else:
                self.pos[i] = self.target_pos[i]

        # Publish wheel positions and velocities
        pos_array = Float64MultiArray(data=self.pos)
        vel_array = Float64MultiArray(data=self.vel)
        self.pub_pos.publish(pos_array)
        self.pub_vel.publish(vel_array)


    def apply_dead_zone(self, new_val, prev_val):
        if abs(new_val - prev_val) > self.dead_zone:
            return self.alpha * new_val + (1 - self.alpha) * prev_val
        return prev_val


    def autonomy_control(self, vel_msg):
        nav_vel_x = vel_msg.twist.linear.x
        nav_vel_z = vel_msg.twist.angular.z
        vel_msg.twist.linear.x = vel_msg.twist.linear.x * self.scale_linear_x
        vel_msg.twist.linear.y = vel_msg.twist.linear.y * self.scale_linear_y
        vel_msg.twist.angular.z = vel_msg.twist.angular.z * self.scale_angular_z
        if nav_vel_z != 0.0 and nav_vel_x == 0.0 or abs(nav_vel_z) > 0.75:
            self.pivot_turn(vel_msg)
        else:
            self.opposite_phase_steering(vel_msg)


    def in_phase_steering(self, vel_msg):
        if self.initial_sign_x == 0.0 or self.entered_function_flag == False: 
            self.initial_sign_x = np.sign(vel_msg.twist.linear.x)
            self.entered_function_flag = True

        sign_x = self.initial_sign_x
        sign_y = np.sign(vel_msg.twist.linear.y)

        if np.sign(vel_msg.twist.linear.x) != self.initial_sign_x:
            vel_msg.twist.linear.x = 0.0
            vel_msg.twist.linear.y = math.copysign(self.scale_linear_y, sign_y)
            axle_angle = self.previous_angle
        elif vel_msg.twist.linear.x != 0:
            axle_angle = math.atan(vel_msg.twist.linear.y / vel_msg.twist.linear.x)
            self.previous_angle = axle_angle
        else:
            axle_angle = 0.0

        V = math.hypot(vel_msg.twist.linear.x, vel_msg.twist.linear.y)  # Combined linear velocity magnitude
        self.pos[:4] = axle_angle
        self.vel[:] = sign_x * V


    def opposite_phase_steering(self, vel_msg):
        vel_steerring_offset = vel_msg.twist.angular.z * self.wheel_steering_y_offset
        sign = np.sign(vel_msg.twist.linear.x)

        self.vel[0] = sign*math.hypot(vel_msg.twist.linear.x - vel_msg.twist.angular.z*self.steering_track/2, vel_msg.twist.angular.z*self.wheel_base/2) - vel_steerring_offset
        self.vel[1] = sign*math.hypot(vel_msg.twist.linear.x + vel_msg.twist.angular.z*self.steering_track/2, vel_msg.twist.angular.z*self.wheel_base/2) + vel_steerring_offset
        self.vel[2] = sign*math.hypot(vel_msg.twist.linear.x - vel_msg.twist.angular.z*self.steering_track/2, vel_msg.twist.angular.z*self.wheel_base/2) - vel_steerring_offset
        self.vel[3] = sign*math.hypot(vel_msg.twist.linear.x + vel_msg.twist.angular.z*self.steering_track/2, vel_msg.twist.angular.z*self.wheel_base/2) + vel_steerring_offset

        if vel_msg.twist.linear.x == 0.0:
            self.target_pos[0] = 0.0
            self.target_pos[1] = 0.0
            self.target_pos[2] = 0.0
            self.target_pos[3] = 0.0
        elif vel_msg.twist.angular.z < 0:
            self.pos[0] = -1.5*math.atan(vel_msg.twist.angular.z*self.wheel_base/(2*abs(vel_msg.twist.linear.x) + vel_msg.twist.angular.z*self.steering_track))
            if abs(self.pos[0]) > self.max_axle_position:
                self.pos[0] = self.max_axle_position
            self.axle_positions()
        else:
            self.pos[0] = -1.5*math.atan(vel_msg.twist.angular.z*self.wheel_base/(2*abs(vel_msg.twist.linear.x) - vel_msg.twist.angular.z*self.steering_track))
            if abs(self.pos[0]) > self.max_axle_position:
                self.pos[0] = -self.max_axle_position
            self.axle_positions()

    def pivot_turn(self, vel_msg):
        self.target_pos[0] = math.atan(self.wheel_base / self.steering_track)
        self.target_pos[1] = - math.atan(self.wheel_base / self.steering_track)
        self.target_pos[2] = - math.atan(self.wheel_base / self.steering_track)
        self.target_pos[3] = math.atan(self.wheel_base / self.steering_track)

        if self.target_pos[0] == self.pos[0]:
            self.vel[0] = - vel_msg.twist.angular.z
            self.vel[1] = vel_msg.twist.angular.z
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
    marinero_control = MarineroControl()
    rclpy.spin(marinero_control)
    marinero_control.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

