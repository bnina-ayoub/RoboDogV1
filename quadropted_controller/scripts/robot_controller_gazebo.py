#!/usr/bin/env python3
# Author: lnotspotl, abutalipovvv
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String  
from quadropted_msgs.msg import RobotModeCommand, RobotVelocity  
from quadropted_msgs.srv import RobotBehaviorCommand
from InverseKinematics import robot_IK
from RobotController import RobotController
import numpy as np  

USE_IMU = True
RATE = 60

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__("Robot_Controller")

        # Объявляем параметры
        self.declare_parameter('verbose', True)  # Set verbose to True for debugging
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value
        if self.verbose:
            self.get_logger().info(f"Verbose mode: {self.verbose}")

        # Геометрия робота
        body = [0.3762, 0.0935]
        legs = [0.0, 0.0955, 0.213, 0.213]
        
        self.declare_parameter('robot_id', 1)
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().integer_value

        
        self.robot = RobotController.Robot(self, body, legs, USE_IMU, self.robot_id)
        self.inverseKinematics = robot_IK.InverseKinematics(body, legs)

        
        self.joint_command_publisher = self.create_publisher(Float64MultiArray, "joint_group_controller/commands", 10)

        # Create topics and subscribers with more verbose logging
        if self.verbose:
            self.get_logger().info("Creating robot_mode subscriber")
        self.create_subscription(RobotModeCommand, "robot_mode", self.mode_callback, 10)
        
        # Also subscribe to String messages for alternative control
        self.create_subscription(String, "mode", self.string_mode_callback, 10)
        
        self.create_subscription(RobotVelocity, "robot_velocity", self.robot.velocity_callback, 10)

        # Create the robot behavior service
        self.srv = self.create_service(
            RobotBehaviorCommand, 
            'robot_behavior_command', 
            self.behavior_command_callback
        )
        
        if self.verbose:
            self.get_logger().info(f"Service created: robot_behavior_command")
            self.get_logger().info(f"Robot controller initialized with ID: {self.robot_id}")

        # Timer for control loop
        self.timer = self.create_timer(1.0 / RATE, self.control_loop)
    
    def mode_callback(self, msg):
        """Explicit mode callback that logs the received message"""
        if self.verbose:
            self.get_logger().info(f"Received robot mode: {msg.mode}")
        
        # Pass the message to the robot's mode handler
        self.robot.mode_callback(msg)
    
    def string_mode_callback(self, msg):
        """Handle string mode messages for compatibility"""
        if self.verbose:
            self.get_logger().info(f"Received string mode: {msg.data}")
        
        # Convert string to RobotModeCommand
        mode_msg = RobotModeCommand()
        mode_msg.mode = msg.data
        mode_msg.robot_id = self.robot_id
        
        # Pass to the robot's mode handler
        self.robot.mode_callback(mode_msg)
    
    def behavior_command_callback(self, request, response):
        """Service callback for robot behavior commands"""
        if self.verbose:
            self.get_logger().info(f"Received behavior command: {request.command}")
        
        # Implement behavior command handling here
        command = request.command.lower()
        
        if command == "sit":
            # Handle sit command
            self.get_logger().info("Robot sitting down")
            mode_msg = RobotModeCommand()
            mode_msg.mode = "STAND"
            mode_msg.robot_id = self.robot_id
            self.robot.mode_callback(mode_msg)
            response.success = True
            
        elif command == "walk" or command == "trot":
            # Handle walk command
            self.get_logger().info("Setting robot to walk/trot mode")
            mode_msg = RobotModeCommand()
            mode_msg.mode = "TROT"
            mode_msg.robot_id = self.robot_id
            self.robot.mode_callback(mode_msg)
            response.success = True
            
        elif command == "stand" or command == "up":
            # Handle stand command
            self.get_logger().info("Robot standing up")
            mode_msg = RobotModeCommand()
            mode_msg.mode = "REST"
            mode_msg.robot_id = self.robot_id
            self.robot.mode_callback(mode_msg)
            response.success = True
            
        else:
            self.get_logger().warning(f"Unknown command: {command}")
            response.success = False
        
        return response

    def control_loop(self):
        if self.verbose:
            self.get_logger().debug("Control loop triggered")
        
        leg_positions = self.robot.run()
        
        # Change controller
        self.robot.change_controller()

        dx = self.robot.state.body_local_position[0]
        dy = self.robot.state.body_local_position[1]
        dz = self.robot.state.body_local_position[2]

        roll = self.robot.state.body_local_orientation[0]
        pitch = self.robot.state.body_local_orientation[1]
        yaw = self.robot.state.body_local_orientation[2]

        try:
            # Calculate joint angles
            joint_angles = self.inverseKinematics.inverse_kinematics(leg_positions, dx, dy, dz, roll, pitch, yaw)

            # Publish joint angles
            joint_command_msg = Float64MultiArray()
            joint_command_msg.data = joint_angles  
            self.joint_command_publisher.publish(joint_command_msg)
            if self.verbose:
                self.get_logger().debug(f"Published joint angles: {joint_angles}")

        except Exception as e:
            self.get_logger().error(f"Error in control loop: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
