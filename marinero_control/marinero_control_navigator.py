#!usr/bin/env python3

import os
import cv2
import math
import rclpy
import time
import numpy as np
from datetime import datetime
from pathlib import Path
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Twist, TwistStamped
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Float64MultiArray, Int32, Bool, Float64, Empty

from rclpy.callback_groups import ReentrantCallbackGroup


vel_msg = Twist()       # robot velocity after twist mux node
photo_path = Path("/home/albert/marinero_ws/waypoint_images") # Hardcoded to my PC workstation --> modify to your needs

class MarineroControl(Node):

    def __init__(self):
        super().__init__("marinero_control")
        self.wheel_seperation = 0.45            # self.wheel_seperation = 0.425
        self.wheel_base = 0.7                   # self.wheel_base = 0.8
        self.wheel_radius = 0.1                 # self.wheel_radius = 0.1016
        self.wheel_steering_y_offset = 0.0
        self.steering_track = self.wheel_seperation - 2*self.wheel_steering_y_offset

        self.scale_linear_x = 10.0  # Linear velocity x scale
        self.scale_linear_y = 8.0   # Linear velocity y scale
        self.scale_angular_z = 5.0  # Angular velocity scale

        self.camera_base_turn, self.camera_turn = [0.0, 0.0]            # Initial camera positions
        self.nav_vel_x, self.nav_vel_y, self.nav_vel_z = 0.0, 0.0, 0.0  # Navigation velocities

        self.initial_sign_x = 0.0           # Initial sign of x velocity
        self.mode_selection = 0             # 1: opposite phase, 2: in-phase, 3: pivot turn 4: none
        self.axle_adjustment_speed = 0.025  # rotation speed of axles when returning to initial positions
        self.max_angle_step = 0.05          # maximum angle step for the axles
        self.stable_frames = 0.0            # saftey check for matching inital path and robot orientation

        self.pos = np.array([0,0,0,0,0,1.5,1.5], float)
        self.vel = np.array([0,0,0,0], float)                   # left_front, right_front, left_rear, right_rear
        self.wheel_vel = np.array([0,0,0,0], float)             # wheel velocities for feedback control
        self.target_pos = np.array([0,0,0,0], float)            # target positions for the wheels when the robot is standing still
        self.max_axle_position = math.pi/6                      # radians

        self.br = CvBridge()
        self.goal_position = PoseStamped().pose.position
        self.goal_orientation = PoseStamped().pose.orientation
        self.position = Odometry().pose.pose.position
        self.orientation = Odometry().pose.pose.orientation
        self.home_position = (197.212, 280.830)

        self.waypoints = []
        self.distance_from_goal = 0.0
        self.yaw_error = 0.0
        self.current_idx = 0
        self.position_reached_threshold = 0.20
        self.orientation_reached_threshold = 0.15
        self.goal_active = False
        self.new_goal = True

        self.last_frame = None
        self.boat_searching = False
        self.image_center = (320.0, 240.0)      # image center
        self.boat_box_center = (320.0, 240.0)   # boat box center
        self.photo_saved = 0    # 0 = travelling, 1 = photo saved, 2 = photo not sent, 3 = home
        
        self.callback_group = ReentrantCallbackGroup

        self.odom_subscription = self.create_subscription(Odometry, "/marinero/odom", self.odom_callback, 10)
        self.final_vel_subscription = self.create_subscription(Twist, "/cmd_vel_out", self.velocity_callback, 10)
        self.camera_subscription = self.create_subscription(Float64MultiArray, "/camera_position", self.camera_pose_callback, 10)  
        self.mode_selection_subscription = self.create_subscription(Int32, "/mode_selection", self.mode_selection_callback, 10)
        self.goal_subscription = self.create_subscription(PoseStamped, "/goal_pose", self.goal_callback, 10)
        self.waypoint_subscription = self.create_subscription(MarkerArray, "/waypoints", self.waypoint_callback, 10)

        self.stamped_velocity_publisher = self.create_publisher(TwistStamped, "/cmd_vel_stamped", 10)
        self.pub_pos = self.create_publisher(Float64MultiArray, "/forward_position_controller/commands", 10)
        self.pub_vel = self.create_publisher(Float64MultiArray, "/forward_velocity_controller/commands", 10)
        self.input_waypoint_publisher = self.create_publisher(Empty, "/input_at_waypoint/next_goal", 10)

        self.photo_take_publisher = self.create_publisher(Bool, "/boat_searching", 10)
        self.photo_saved_publisher = self.create_publisher(Int32, "/photo_saved", 10)
        self.photo_speed_publisher = self.create_publisher(Float64, "/photo_angular_speed", 10)
        self.box_center_subscriber = self.create_subscription(Float64MultiArray, "/yolo_box_center", self.box_center_callback, 10)
        self.image_subscriber = self.create_subscription(Image, "/right_depth_camera/image_raw", self.photo_callback, 10)

        # PID gains for camera base (pan)
        self.kp_x = 0.00015
        self.ki_x = 0.00002
        self.kd_x = 0.00002
        self.integral_x = 0.0
        self.prev_error_x = 0.0

        # PID gains for camera tilt (Y)
        self.kp_y = 0.00015
        self.ki_y = 0.000025
        self.kd_y = 0.000025
        self.integral_y = 0.0
        self.prev_error_y = 0.0

        # Time tracking for derivative calculation
        self.prev_time = self.get_clock().now()


    def odom_callback(self, msg):  # sourcery skip: extract-method
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation

        if self.goal_active:
            dx = self.goal_position.x - self.position.x
            dy = self.goal_position.y - self.position.y
            self.distance_from_goal = math.hypot(dx, dy)

            g_q = self.goal_orientation
            r_q = self.orientation
            goal_yaw = euler_from_quaternion([g_q.x, g_q.y, g_q.z, g_q.w])[2]
            robot_yaw = euler_from_quaternion([r_q.x, r_q.y, r_q.z, r_q.w])[2]
            self.yaw_error = math.atan2(math.sin(goal_yaw - robot_yaw), math.cos(goal_yaw - robot_yaw))
            self.check_goal_reached()

        if self.boat_searching:
            home_distance = float('inf')
            home_distance = math.sqrt((self.goal_position.x - self.home_position[0])**2 +
                                        (self.goal_position.y - self.home_position[1])**2)
            if home_distance > 0.5:
                self.face_boat()
            else:
                self.boat_searching = False
                self.photo_saved_publisher.publish(Int32(data=3))
                self.photo_take_publisher.publish(Bool(data=self.boat_searching))

    def goal_callback(self, msg):
        if not msg:
            return
        
        if not self.waypoints or self.current_idx >= len(self.waypoints):
            self.publish_next_goal_pose(msg)
        else:
            self.goal_active = False
            self.boat_searching = False
        self.photo_take_publisher.publish(Bool(data=(self.boat_searching)))
        self.photo_saved_publisher.publish(Int32(data=self.photo_saved))

    def waypoint_callback(self, msg: MarkerArray):
        if not msg.markers:
            return

        new_waypoints: list[PoseStamped] = []
        for marker in msg.markers:
            wp = PoseStamped()
            wp.header.frame_id = marker.header.frame_id
            wp.header.stamp = self.get_clock().now().to_msg()
            wp.pose = marker.pose  # marker.pose is a geometry_msgs/Pose

            # Avoid duplicates (compare position.x and position.y; add orientation if desired)
            is_duplicate = any(
                abs(wp.pose.position.x - existing.pose.position.x) < 1e-3 and
                abs(wp.pose.position.y - existing.pose.position.y) < 1e-3
                for existing in new_waypoints)
            if not is_duplicate:
                new_waypoints.append(wp)
            
        # If after filtering there are no waypoints, do nothing
        if not new_waypoints:
            return

        self.waypoints = new_waypoints
        self.current_idx = 0
        if len(self.waypoints) == 1:
            self.publish_next_waypoint()

    def box_center_callback(self, msg):
        self.boat_box_center = msg.data

    def photo_callback(self, msg):
        self.last_frame = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")


    def capture_photo(self):
        timestamp = datetime.now().strftime("%H%M%S")
        filename = os.path.join(photo_path, f"photo_{timestamp}.png")
        cv2.imwrite(filename, self.last_frame)                          # type: ignore
        # self.get_logger().info(f"Photo saved: {filename}")


    def check_goal_reached(self):
        if self.distance_from_goal <= self.position_reached_threshold and abs(self.yaw_error) <= self.orientation_reached_threshold:
            if self.waypoints and self.current_idx < len(self.waypoints):
                self.get_logger().info(f"Reached waypoint {self.current_idx}")
                self.current_idx += 1
            else:
                self.get_logger().info("Reached 2D Goal Pose")
                goal_reached_msg = Float64MultiArray()
                goal_reached_msg.data = [self.position.x, self.position.y]
            self.goal_active = False
            self.boat_searching = True
            self.boat_box_center = self.image_center
            self.photo_take_publisher.publish(Bool(data=(self.boat_searching)))


    def publish_next_goal_pose(self, goal: PoseStamped):
            self.goal_position = goal.pose.position
            self.goal_orientation = goal.pose.orientation
            goal_z_angle = self.goal_orientation.z * 180/math.pi
            self.goal_active = True
            self.new_goal = True
            self.boat_searching = False
            self.photo_saved = 0
            self.boat_box_center = self.image_center
            self.pos[4:7] = [0.0, 1.5, 1.5]
            self.get_logger().info(f"Received 2D Goal Pose -> ({self.goal_position.x:.2f}, {self.goal_position.y:.2f}, {goal_z_angle:.2f})")


    def publish_next_waypoint(self):
        if self.current_idx < len(self.waypoints):
            goal = self.waypoints[self.current_idx]
            goal.header.stamp = self.get_clock().now().to_msg()
            self.goal_position = goal.pose.position
            self.goal_orientation = goal.pose.orientation
            self.goal_active = True
            self.new_goal = True
            self.boat_searching = False
            self.photo_saved = 0
            self.boat_box_center = self.image_center
            self.pos[4:7] = [0.0, 1.5, 1.5]
            self.get_logger().info(f"Received waypoint {self.current_idx} -> "f"({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})")


    def face_boat(self):
        if not hasattr(self, "face_boat_start_time"):
            self.face_boat_start_time = self.get_clock().now()
            self.get_logger().info("Function face_boat() started.")
            
        total_elapsed = (self.get_clock().now() - self.face_boat_start_time).nanoseconds / 1e9
        
        if total_elapsed >= 35.0:
            self.get_logger().warn(f"Function face_boat() exceeded 35s ({total_elapsed:.2f}s) -> stopping.")
            self.boat_searching = False
            self.camera_turn = 0.0
            self.camera_base_turn = 0.0
            self.photo_saved = 2
            self.publish_next_waypoint()
            self.input_waypoint_publisher.publish(Empty())
            del self.face_boat_start_time
            return  # Exit early
        
        error_x = self.boat_box_center[0] - self.image_center[0]
        error_y = self.boat_box_center[1] - self.image_center[1]
        self.photo_saved = 0
        angular_z_velocity = 0.0
        
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        if dt <= 0.0:  # Avoid division by zero
            dt = 1e-6
        
        # Case 1: Boat detected, camera base needs correction
        if abs(error_x) > 10.0 and self.boat_box_center[0] != self.image_center[0] and abs(self.pos[4]) < math.pi:
            # PID for X (base)
            self.integral_x += error_x * dt
            derivative_x = (error_x - self.prev_error_x) / dt
            if error_x * self.prev_error_x < 0:     # if error direction changed, clear integral
                self.integral_x = 0.0
            correction_x = self.kp_x * error_x + self.ki_x * self.integral_x + self.kd_x * derivative_x
            
            # Limit correction
            max_step_x = 0.00125
            correction_x = max(-max_step_x, min(max_step_x, correction_x))
            self.pos[4] -= correction_x
            self.prev_error_x = error_x
            self.prev_time = current_time
            
        # Case 2: Boat roughly centered, adjust Y
        elif abs(error_x) <= 10.0 and self.boat_box_center[0] != self.image_center[0] and 1.05 < self.pos[5] < 1.95:
            self.camera_base_turn = 0.0
            
            if abs(error_y) <= 10.0 and self.boat_box_center[1] != self.image_center[1]:
                self.camera_turn = 0.0
                self.boat_searching = False
                self.capture_photo()
                self.get_logger().info("Boat centered -> Photo taken!")
                self.photo_saved = 1
                self.publish_next_waypoint()
                self.input_waypoint_publisher.publish(Empty())
                del self.face_boat_start_time 
            else:
                # PID for Y (tilt)
                self.integral_y += error_y * dt
                derivative_y = (error_y - self.prev_error_y) / dt
                if error_x * self.prev_error_x < 0:     # if error direction changed, clear integral
                    self.integral_x = 0.0
                correction_y = self.kp_y * error_y + self.ki_y * self.integral_y + self.kd_y * derivative_y
                
                # Limit correction
                max_step_y = 0.00085
                correction_y = max(-max_step_y, min(max_step_y, correction_y))
                self.pos[5] += correction_y
                self.pos[6] += correction_y
                self.prev_error_y = error_y
                self.prev_time = current_time
                
            if hasattr(self, "search_start_time"):
                del self.search_start_time
                
        # Case 3: Boat not detected
        else:
            if not hasattr(self, "search_start_time"):
                self.search_start_time = self.get_clock().now()
                
            elapsed = (self.get_clock().now() - self.search_start_time).nanoseconds / 1e9
            if elapsed < 20.0:
                if error_x > 0:
                    angular_z_velocity = 0.5 * self.scale_angular_z
                else:
                    angular_z_velocity = - 0.5 * self.scale_angular_z
            else:
                self.camera_turn = 0.0
                self.camera_base_turn = 0.0
                self.boat_searching = False
                angular_z_velocity = 0.0
                self.get_logger().warn("Boat not found after 20s -> Aborting search.")
                self.photo_saved = 2
                self.publish_next_waypoint()
                self.input_waypoint_publisher.publish(Empty())
                del self.search_start_time
                if hasattr(self, "face_boat_start_time"):
                    del self.face_boat_start_time
                
        self.photo_take_publisher.publish(Bool(data=self.boat_searching))
        self.photo_saved_publisher.publish(Int32(data=self.photo_saved))
        self.photo_speed_publisher.publish(Float64(data=angular_z_velocity))


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
        self.stamped_velocity_publisher.publish(stamped_velocity)
        
        self.pos[4] += self.camera_base_turn
        self.pos[5] += self.camera_turn
        self.pos[6] += self.camera_turn
        
        if self.mode_selection == 1:
            self.in_phase_steering(vel_msg)
        elif self.mode_selection == 2:
            self.opposite_phase_steering(vel_msg)
        elif self.mode_selection == 3 or self.boat_searching:
            self.pivot_turn(vel_msg)
        elif self.mode_selection == 4:
            self.autonomy_control(vel_msg)
        else:
            self.previous_angle = 0.0
            self.initial_sign_x = 0.0
            self.entered_function_flag = False
            self.target_pos[:4] = 0.0
            
            # Slowly reduce the wheel velocities to zero to avoid abrupt stops
            for i in range(4):
                if abs(self.vel[i]) > 15.0:
                    self.vel[i] -= 1.0 * np.sign(self.vel[i])
                elif 5.0 < abs(self.vel[i]) <= 15.0:
                    self.vel[i] -= 0.75 * self.vel[i]
                else:
                    self.vel[i] = 0.0
                    
        # Gradually move axles to target positions
        for i in range(4):
            target = self.target_pos[i] if self.target_pos[i] is not None else 0.0
            target_delta = target - self.pos[i]
            if abs(target_delta) > self.max_angle_step:
                self.pos[i] += self.axle_adjustment_speed * np.sign(target_delta)
            else:
                self.pos[i] = target
            time.sleep(0.005)
            
        # Publish wheel positions and velocities
        pos_array = Float64MultiArray(data=self.pos)
        vel_array = Float64MultiArray(data=self.vel)
        self.pub_pos.publish(pos_array)
        self.pub_vel.publish(vel_array)


    ### For MPPI Navigation
    def autonomy_control(self, vel_msg):
        distance_from_goal = self.distance_from_goal
        yaw_error = self.yaw_error
        
        nav_vel_x = vel_msg.linear.x
        nav_vel_z = vel_msg.angular.z
        vel_msg.linear.y = math.copysign(0.1, yaw_error) if yaw_error != 0 else 0.0
        vel_msg.linear.x *= self.scale_linear_x
        vel_msg.angular.z *= self.scale_angular_z
        vel_msg.linear.y *= - self.scale_linear_y
        
        if self.new_goal:
            self.pivot_turn(vel_msg)
            if abs(nav_vel_z) <= 0.075:
                self.stable_frames += 1
            else:
                self.stable_frames = 0
            # Exit pivot mode only after N stable frames
            if self.stable_frames > 15:   # e.g., 15 cycles at ~20Hz
                self.new_goal = False
                
        elif abs(yaw_error) < 0.15 and 0.0 < distance_from_goal <= 0.5:
            self.in_phase_steering(vel_msg)
            
        elif distance_from_goal > 1.0 and nav_vel_x > 0.0 and abs(nav_vel_z) <= 0.7:
            self.opposite_phase_steering(vel_msg)
            
        elif 0.25 < distance_from_goal < 1.0 and nav_vel_x > 0.0 and abs(nav_vel_z) < nav_vel_x * 1.5:
            self.opposite_phase_steering(vel_msg)
            
        else:
            if nav_vel_z > 0.0:
                vel_msg.angular.z = max(math.copysign(0.5*abs(yaw_error), nav_vel_z), nav_vel_z) * self.scale_angular_z
            else:
                vel_msg.angular.z = min(math.copysign(0.5*abs(yaw_error), nav_vel_z), nav_vel_z) * self.scale_angular_z
            self.pivot_turn(vel_msg)
            
            
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
            
        if self.mode_selection == 4:
            V = max(min(math.hypot(vel_msg.linear.x, vel_msg.linear.y), 1.0), -1.0)
        else:
            V = math.hypot(vel_msg.linear.x, vel_msg.linear.y)
            
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
            self.pos[0] = 1.5*math.atan(vel_msg.angular.z*self.wheel_base/(2*abs(vel_msg.linear.x) + vel_msg.angular.z*self.steering_track))
            if abs(self.pos[0]) > self.max_axle_position:
                self.pos[0] = - self.max_axle_position
            self.axle_positions()
        else:
            self.pos[0] = 1.5*math.atan(vel_msg.angular.z*self.wheel_base/(2*abs(vel_msg.linear.x) - vel_msg.angular.z*self.steering_track))
            if abs(self.pos[0]) > self.max_axle_position:
                self.pos[0] = self.max_axle_position
            self.axle_positions()


    def pivot_turn(self, vel_msg):
        self.target_pos[0] = - math.atan(self.wheel_base / self.steering_track)
        self.target_pos[1] = math.atan(self.wheel_base / self.steering_track)
        self.target_pos[2] = math.atan(self.wheel_base / self.steering_track)
        self.target_pos[3] = -math.atan(self.wheel_base / self.steering_track)
        
        if abs(self.target_pos[0] - self.pos[0]) < 0.02:
            self.vel[0] = - vel_msg.angular.z
            self.vel[1] = vel_msg.angular.z
            self.vel[2] = self.vel[0]
            self.vel[3] = self.vel[1]
        else:
            self.vel[:4] = 0.0


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
