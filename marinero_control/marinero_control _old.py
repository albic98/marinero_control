#!usr/bin/env python3

import math
import threading
import rclpy
import numpy as np
import rclpy.duration
import rclpy.executors
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

vel_msg = Twist()  # robot velocity
mode_selection = 0 # 1:opposite phase, 2:in-phase, 3:pivot turn 4: none
camera_base_turn, camera_turn = [0.0, 0.0]

class Commander(Node):

    def __init__(self):
        super().__init__('commander')
        timer_period = 0.02
        self.wheel_seperation = 0.425
        self.wheel_base = 0.8
        self.wheel_radius = 0.1016
        self.wheel_steering_y_offset = 0.0
        self.steering_track = self.wheel_seperation - 2*self.wheel_steering_y_offset
        
        self.pos = np.array([0,0,0,0,0,1.5,1.5], float)
        self.vel = np.array([0,0,0,0], float)               # left_front, right_front, left_rear, right_rear
        self.target_pos = np.array([0,0,0,0], float)        # target positions for the wheels when the robot is standing still

        self.pub_pos = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.pub_vel = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        
        self.joint_names = ['camera_base_joint', 'left_camera_joint', 'right_camera_joint']
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        global vel_msg, mode_selection, camera_base_turn, camera_turn
        adjustment_speed = 0.025 # Steering adjustment speed
    
        # self.joint_positions[0] += camera_base_turn
        # self.joint_positions[1] += camera_turn
        # self.joint_positions[2] += camera_turn
        
        # traj_msg = JointTrajectory()
        # traj_msg.joint_names = self.joint_names

        # point = JointTrajectoryPoint()
        # point.positions = self.joint_positions
        # point.time_from_start = rclpy.duration.Duration(seconds=1.0).to_msg()
        
        # traj_msg.points.append(point)
        # self.pub_cam.publish(traj_msg)

        self.pos[4] += camera_base_turn
        self.pos[5] += camera_turn
        self.pos[6] += camera_turn
        
        ## in-phase ##
        if(mode_selection == 1):
            
            # TODO Add logic for wheels which forbids wheels jumping from pi to -pi and vice versa
        
            V = math.hypot(vel_msg.linear.x, vel_msg.linear.y)
            sign = np.sign(vel_msg.linear.x)
            
            if(vel_msg.linear.x != 0):
                ang = vel_msg.linear.y / vel_msg.linear.x
            else:
                ang = 0
                
            self.pos[0] = math.atan(ang)
            self.pos[1] = math.atan(ang)
            self.pos[2] = self.pos[0]
            self.pos[3] = self.pos[1]
            self.vel[:] = sign*V
            
        ## opposite phase ##
        elif(mode_selection == 2):
            
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
            else:
                self.pos[2] = math.atan(vel_msg.angular.z*self.wheel_base/(2*vel_msg.linear.x + vel_msg.angular.z*self.steering_track))
                self.pos[3] = math.atan(vel_msg.angular.z*self.wheel_base/(2*vel_msg.linear.x - vel_msg.angular.z*self.steering_track))
                self.pos[0] = - self.pos[2]
                self.pos[1] = - self.pos[3]

    
        ## pivot turn ##
        elif(mode_selection == 3):

            self.target_pos[0] = math.atan(self.wheel_base / self.steering_track) 
            self.target_pos[1] = - math.atan(self.wheel_base / self.steering_track)      
            self.target_pos[2] = - math.atan(self.wheel_base / self.steering_track)    
            self.target_pos[3] = math.atan(self.wheel_base / self.steering_track)
            
            self.vel[0] = - vel_msg.angular.z
            self.vel[1] = vel_msg.angular.z
            self.vel[2] = self.vel[0]           # self.vel[0]*0.8
            self.vel[3] = self.vel[1]           # self.vel[1]*0.8
        else:
            self.target_pos[0:4] = 0
            self.vel[:] = 0

        # Slowly adjust the current position to the target position
        for i in range(4):
            if abs(self.pos[i] - self.target_pos[i]) > adjustment_speed:
                self.pos[i] += adjustment_speed * np.sign(self.target_pos[i] - self.pos[i])
            else:
                self.pos[i] = self.target_pos[i]
                
        pos_array = Float64MultiArray(data=self.pos) 
        vel_array = Float64MultiArray(data=self.vel) 
        self.pub_pos.publish(pos_array)
        self.pub_vel.publish(vel_array)
        self.target_pos[0:4] = 0
        self.vel[:] = 0

class JoySubscriber(Node):

    def __init__(self):
        super().__init__('joy_subscriber')
        self.scale_angular = self.declare_parameter('scale_angular', 0.025).value
        
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joy_subscription = self.create_subscription(Joy, '/joy', self.listener_callback, 10)
        self.joy_subscription

    def listener_callback(self, data):
        global vel_msg, mode_selection, camera_base_turn, camera_turn

        if(data.buttons[5] == 1 and data.buttons[4] == 0):   # in-phase # R1 button of PS4 controller
            mode_selection = 1
        elif(data.buttons[4] == 1 and data.buttons[5] == 0): # opposite phase # L1 button of PS4 controller
            mode_selection = 2
        elif(data.buttons[4] == 1 and data.buttons[5] == 1): # pivot turn # L1 and R1 button of PS4 controller
            mode_selection = 3
        else:
            mode_selection = 4

        if data.buttons[0] == 1 and data.axes[1] > 0:
            vel_msg.linear.x = data.axes[1] * 15.0 + data.buttons[0] * 15.0
        else:
            vel_msg.linear.x = data.axes[1] * 15.0
        
        vel_msg.linear.y = - data.axes[0] * 7.5
        if data.axes[3] == 0:
            vel_msg.angular.z = 0.000001
        else:
            vel_msg.angular.z = data.axes[3] * 10.0
        
        self.cmd_publisher.publish(vel_msg)
        
        camera_base_turn = - data.axes[6] * self.scale_angular
        camera_turn = - data.axes[7] * self.scale_angular

def main(args=None):
    rclpy.init(args=None)
    
    commander = Commander()
    joy_subscriber = JoySubscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(commander)
    executor.add_node(joy_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    rate = commander.create_rate(2.0)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()
    executor_thread.join()
    
if __name__ == '__main__':
    main()

