#!/usr/bin/env python3
# Author: mike4192 https://github.com/mike4192/spotMicro, lnotspotl, abutalipovvv

import numpy as np
import rclpy
from rclpy.node import Node
from RoboticsUtilities.Transformations import rotxyz, rotz
from .GaitController import GaitController
from .PIDController import PID_controller
from geometry_msgs.msg import Twist
from quadropted_msgs.msg import RobotFootContact

class TrotGaitController(GaitController):
    def __init__(self, node, default_stance, stance_time, swing_time, time_step, use_imu):
        self.node = node  
        self.use_imu = use_imu
        self.use_button = True
        self.autoRest = True
        self.trotNeeded = True
        
        
        contact_phases = np.array([[1, 1, 1, 0],  # 0: Leg swing
                                   [1, 0, 1, 1],  # 1: Moving stance forward
                                   [1, 0, 1, 1],  
                                   [1, 1, 1, 0]])

        z_error_constant = 0.02  # This constant determines how fast we move
                                 # toward the goal in the z direction
        z_leg_lift = 0.14  

        super().__init__(stance_time, swing_time, time_step, contact_phases, default_stance)
        
        self.velocity_pub = self.node.create_publisher(Twist, "controller_velocity", 10)  # Используем переданный node

        self.foot_contact_pub = self.node.create_publisher(RobotFootContact, "foot_contact", 10)

        self.max_x_velocity = 0.035  # [m/s]
        self.max_y_velocity = 0.012  # [m/s]
        self.max_yaw_rate = 0.5  # [rad/s]

        self.swingController = TrotSwingController(
            self.stance_ticks,
            self.swing_ticks,
            self.time_step,
            self.phase_length,
            z_leg_lift,
            self.default_stance
        )

        self.stanceController = TrotStanceController(
            self.phase_length,
            self.stance_ticks,
            self.swing_ticks,
            self.time_step,
            z_error_constant
        )

        # TODO: tune kp, ki and kd
        #                                     kp    ki    kd
        self.pid_controller = PID_controller(0.15, 0.02, 0.002)

    def updateStateCommand(self, msg, state, command):
        command.velocity[0] = msg.axes[4] * self.max_x_velocity
        command.velocity[1] = msg.axes[3] * self.max_y_velocity
        command.yaw_rate[2] = msg.axes[0] * self.max_yaw_rate

        
        velocity_msg = Twist()
        velocity_msg.linear.x = command.velocity[0]
        velocity_msg.linear.y = command.velocity[1]
        velocity_msg.angular.z = command.yaw_rate[2]
        self.velocity_pub.publish(velocity_msg)

        velocity_msg_raw = Twist()
        velocity_msg_raw.linear.x = msg.axes[4] * 0.5  
        velocity_msg_raw.linear.y = msg.axes[3] * 0.5  
        velocity_msg_raw.angular.z = msg.axes[0]
        self.velocity_pub.publish(velocity_msg_raw)

        if self.use_button:
            if msg.buttons[7]:
                self.use_imu = not self.use_imu
                self.use_button = False
                self.get_logger().info(f"Trot Gait Controller - Use roll/pitch compensation: {self.use_imu}")

            elif msg.buttons[6]:
                self.autoRest = not self.autoRest
                if not self.autoRest:
                    self.trotNeeded = True
                self.use_button = False
                self.get_logger().info(f"Trot Gait Controller - Use autorest: {self.autoRest}")
            
        if not self.use_button:
            if not (msg.buttons[6] or msg.buttons[7]):
                self.use_button = True

    def step(self, state, command):
        if self.autoRest:
            if command.velocity[0] == 0 and command.velocity[1] == 0 and np.all(command.yaw_rate == 0):
                if state.ticks % (2 * self.phase_length) == 0:
                    self.trotNeeded = False
            else:
                self.trotNeeded = True

        if self.trotNeeded:
            contact_modes = self.contacts(state.ticks)

            foot_contact_msg = RobotFootContact()
            foot_contact_msg.contacts = [bool(mode) for mode in contact_modes.tolist()]
            self.foot_contact_pub.publish(foot_contact_msg)

            new_foot_locations = np.zeros((3, 4))
            for leg_index in range(4):
                contact_mode = contact_modes[leg_index]
                if contact_mode == 1:
                    new_location = self.stanceController.next_foot_location(leg_index, state, command)
                else:
                    swing_proportion = float(self.subphase_ticks(state.ticks)) / float(self.swing_ticks)

                    new_location = self.swingController.next_foot_location(swing_proportion, leg_index, state, command)

                new_foot_locations[:, leg_index] = new_location

            # Компенсация крена и тангажа
            if self.use_imu:
                compensation = self.pid_controller.run(state.imu_roll, state.imu_pitch)
                roll_compensation = -compensation[0]
                pitch_compensation = -compensation[1]

                rot = rotxyz(roll_compensation, pitch_compensation, 0)
                new_foot_locations = np.matmul(rot, new_foot_locations)

            state.ticks += 1
            return new_foot_locations
        else:
            temp = self.default_stance.copy()
            temp[2] = [command.robot_height] * 4

            foot_contact_msg = RobotFootContact()
            foot_contact_msg.contacts = [True, True, True, True]  # [FR, FL, RR, RL]
            self.foot_contact_pub.publish(foot_contact_msg)
            
            return temp

    def run(self, state, command):
        state.foot_locations = self.step(state, command)
        state.robot_height = command.robot_height

        return state.foot_locations

class TrotSwingController:
    def __init__(self, stance_ticks, swing_ticks, time_step, phase_length, z_leg_lift, default_stance):
        self.stance_ticks = stance_ticks
        self.swing_ticks = swing_ticks
        self.time_step = time_step
        self.phase_length = phase_length
        self.z_leg_lift = z_leg_lift
        self.default_stance = default_stance

    def raibert_touchdown_location(self, leg_index, command):
        scale_factor = 1.0  
        delta_pos_2d = command.velocity * self.phase_length * self.time_step * scale_factor
        delta_pos = np.array([delta_pos_2d[0], delta_pos_2d[1], 0])

        theta = self.stance_ticks * self.time_step * command.yaw_rate[2]
        rotation = rotz(theta)

        return np.matmul(rotation, self.default_stance[:, leg_index]) + delta_pos

    def swing_height(self, swing_phase):
        scale_factor = 1.0  
        if swing_phase < 0.5:
            swing_height_ = (swing_phase / 0.5) * self.z_leg_lift * scale_factor
        else:
            swing_height_ = self.z_leg_lift * (1 - (swing_phase - 0.5) / 0.5) * scale_factor
        return swing_height_

    def next_foot_location(self, swing_prop, leg_index, state, command):
        assert 0 <= swing_prop <= 1
        foot_location = state.foot_locations[:, leg_index]
        swing_height_ = self.swing_height(swing_prop)
        touchdown_location = self.raibert_touchdown_location(leg_index, command)

        time_left = self.time_step * self.swing_ticks * (1.0 - swing_prop)
        velocity = (touchdown_location - foot_location) / float(time_left) * np.array([1, 1, 0])

        delta_foot_location = velocity * self.time_step
        z_vector = np.array([0, 0, swing_height_ + command.robot_height])
        return foot_location * np.array([1, 1, 0]) + z_vector + delta_foot_location

class TrotStanceController:
    def __init__(self, phase_length, stance_ticks, swing_ticks, time_step, z_error_constant):
        self.phase_length = phase_length
        self.stance_ticks = stance_ticks
        self.swing_ticks = swing_ticks
        self.time_step = time_step
        self.z_error_constant = z_error_constant

    def position_delta(self, leg_index, state, command):
        z = state.foot_locations[2, leg_index]

        step_dist_x = command.velocity[0] * (float(self.phase_length) / self.swing_ticks)
        step_dist_y = command.velocity[1] * (float(self.phase_length) / self.swing_ticks)

        
        velocity = np.array([
            -(step_dist_x / 4) / (float(self.time_step) * self.stance_ticks),
            -(step_dist_y / 4) / (float(self.time_step) * self.stance_ticks),
            1.0 / self.z_error_constant * (state.robot_height - z)
        ])

        delta_pos = velocity * self.time_step
        delta_ori = rotxyz(
            -command.yaw_rate[0] * self.time_step,  # Roll
            -command.yaw_rate[1] * self.time_step,  # Pitch
            -command.yaw_rate[2] * self.time_step   # Yaw
        )
        return (delta_pos, delta_ori)

    def next_foot_location(self, leg_index, state, command):
        foot_location = state.foot_locations[:, leg_index]
        (delta_pos, delta_ori) = self.position_delta(leg_index, state, command)
        next_foot_location = np.matmul(delta_ori, foot_location) + delta_pos
        return next_foot_location