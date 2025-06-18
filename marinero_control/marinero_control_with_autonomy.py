#!usr/bin/env python3

import math
import rclpy
import time
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float64MultiArray, Int32
from tf_transformations import euler_from_quaternion


vel_msg = Twist()       # robot velocity after twist mux node

class MarineroControl(Node):

    def __init__(self):
        super().__init__("marinero_control")
        self.wheel_seperation = 0.425
        self.wheel_base = 0.8
        self.wheel_radius = 0.1016
        self.wheel_steering_y_offset = 0.0
        self.steering_track = self.wheel_seperation - 2*self.wheel_steering_y_offset

        self.scale_linear_x = 10.0  # Linear velocity x scale
        self.scale_linear_y = 8.0   # Linear velocity y scale
        self.scale_angular_z = 5.0  # Angular velocity scale
        
        self.camera_base_turn, self.camera_turn = [0.0, 0.0]    # Initial camera positions
        self.nav_vel_x, self.nav_vel_y, self.nav_vel_z = 0.0, 0.0, 0.0 # Navigation velocities

        self.initial_sign_x = 0.0           # Initial sign of x velocity
        self.mode_selection = 0             # 1: opposite phase, 2: in-phase, 3: pivot turn 4: none
        self.axle_adjustment_speed = 0.025  # rotation speed of axles when returning to initial positions

        self.pos = np.array([0,0,0,0,0,1.5,1.5], float)
        self.vel = np.array([0,0,0,0], float)                   # left_front, right_front, left_rear, right_rear
        self.wheel_vel = np.array([0,0,0,0], float)             # wheel velocities for feedback control
        self.target_pos = np.array([0,0,0,0], float)            # target positions for the wheels when the robot is standing still
        self.max_axle_position = math.pi/4                      # radians

        self.goal_position = PoseStamped().pose.position
        self.goal_orientation = PoseStamped().pose.orientation
        self.position = Odometry().pose.pose.position
        self.orientation = Odometry().pose.pose.orientation
        
        self.final_vel_subscription = self.create_subscription(Twist, "/cmd_vel_out", self.velocity_callback, 10)
        self.stamped_velocity = self.create_publisher(TwistStamped, "/cmd_vel_stamped", 10)
        self.camera_subscription = self.create_subscription(Float64MultiArray, "/camera_position", self.camera_pose_callback, 10)  
        self.mode_selection_subscription = self.create_subscription(Int32, "/mode_selection", self.mode_selection_callback, 10)
        self.pub_pos = self.create_publisher(Float64MultiArray, "/forward_position_controller/commands", 10)
        self.pub_vel = self.create_publisher(Float64MultiArray, "/forward_velocity_controller/commands", 10)
        self.odom_subscriber = self.create_subscription(Odometry, "/marinero/odom", self.odom_callback, 10)
        self.goal_subscriber = self.create_subscription(PoseStamped, "/goal_pose", self.goal_callback, 10)

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation

    def goal_callback(self, msg):
        self.goal_position = msg.pose.position
        self.goal_orientation = msg.pose.orientation

    def mode_selection_callback(self, msg):
        self.mode_selection = msg.data

    def camera_pose_callback(self, msg):
        camera_pose = msg.data
        self.camera_base_turn = camera_pose[0]  # Base turn angle
        self.camera_turn = camera_pose[1]       # Camera turn angle  

    def velocity_callback(self, msg):
        vel_msg = msg
        stamped_velocity = TwistStamped()
        stamped_velocity.header.stamp = self.get_clock().now().to_msg()
        stamped_velocity.header.frame_id = "base_link"
        stamped_velocity.twist.linear.x = vel_msg.linear.x
        stamped_velocity.twist.linear.y = vel_msg.linear.y
        stamped_velocity.twist.angular.z = vel_msg.angular.z
        self.stamped_velocity.publish(stamped_velocity)

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
                self.vel[:4] += - 0.85
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


    ### For MPPI Navigation
    def autonomy_control(self, vel_msg):
        nav_vel_x = vel_msg.linear.x
        nav_vel_z = vel_msg.angular.z
        vel_msg.linear.x *= self.scale_linear_x
        vel_msg.angular.z *= self.scale_angular_z
        
        dx = self.goal_position.x - self.position.x
        dy = self.goal_position.y - self.position.y
        self.distance_from_goal = math.sqrt(dx**2 + dy**2)
        g_q = self.goal_orientation
        r_q = self.orientation
        self.goal_euler = euler_from_quaternion([g_q.x, g_q.y, g_q.z, g_q.w])
        self.robot_euler = euler_from_quaternion([r_q.x, r_q.y, r_q.z, r_q.w])
        self.angle_to_goal = self.goal_euler[2] - self.robot_euler[2] 

        if abs(self.angle_to_goal) < 0.25 and abs(self.distance_from_goal) < 0.5:
            vel_msg.linear.y = math.copysign(0.1, self.angle_to_goal) if self.angle_to_goal != 0 else 0.0
            vel_msg.linear.y *= self.scale_linear_y
            self.in_phase_steering(vel_msg)

        elif 0.0 < self.distance_from_goal < 0.25:
            self.opposite_phase_steering(vel_msg)

        elif nav_vel_x > 0.0 and abs(nav_vel_x) > abs(nav_vel_z):
            self.opposite_phase_steering(vel_msg)

        else:
            if nav_vel_z > 0.0:
                vel_msg.angular.z = max(math.copysign(0.2, nav_vel_z), nav_vel_z) * self.scale_angular_z
            else:
                vel_msg.angular.z = min(math.copysign(0.2, nav_vel_z), nav_vel_z) * self.scale_angular_z
            self.pivot_turn(vel_msg)

        # Debugging output
        # self.get_logger().info(f"Autonomy Control - Mode: {self.mode_selection}, Linear X: {vel_msg.linear.x}, Angular Z: {vel_msg.angular.z}")
        print(f"Distance from goal: {self.distance_from_goal:.2f}, Angle to goal: {self.angle_to_goal:.2f}")
        print(f"Linear X: {vel_msg.linear.x:.2f}, Linear Y: {vel_msg.linear.y:.2f}, Angular Z: {vel_msg.angular.z:.2f}")


    # ### For DWB Navigation
    # def autonomy_control(self, vel_msg):
    #     nav_vel_x = vel_msg.linear.x
    #     nav_vel_z = vel_msg.angular.z
    #     vel_msg.linear.x = vel_msg.linear.x * self.scale_linear_x
    #     vel_msg.linear.y = vel_msg.linear.y * self.scale_linear_y
    #     vel_msg.angular.z = vel_msg.angular.z * self.scale_angular_z
    #     if nav_vel_z != 0.0 and nav_vel_x == 0.0 or abs(nav_vel_z) > 0.75:
    #         self.pivot_turn(vel_msg)
    #     else:
    #         self.opposite_phase_steering(vel_msg)


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
    marinero_control = MarineroControl()
    rclpy.spin(marinero_control)
    marinero_control.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

