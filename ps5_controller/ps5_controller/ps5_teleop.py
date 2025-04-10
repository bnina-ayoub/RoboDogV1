#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from quadropted_msgs.msg import RobotModeCommand
import math

class PS5Controller(Node):
    def __init__(self):
        super().__init__('ps5_controller')
        
        # Parameters
        self.declare_parameter('namespace', 'robot1')
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value
        
        # Publishers
        self.vel_pub = self.create_publisher(
            Twist, f'/{self.namespace}/cmd_vel', 10)
        
        self.mode_pub = self.create_publisher(
            RobotModeCommand, f'/{self.namespace}/robot_mode', 10)
        
        # Subscribers
        self.joy_sub = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)
        
        # Control variables - match with keyboard teleop defaults
        self.max_linear_speed = 0.5
        self.max_angular_speed = 1.0
        
        # Speed modifiers
        self.speed_linear_x = 0.5  
        self.speed_linear_y = 0.5
        self.speed_linear_z = 0.5
        self.speed_angular = 1.0
        
        # Deadzone for joystick inputs
        self.deadzone = 0.05
        
        # For button debouncing (to prevent multiple speed changes)
        self.last_speed_change = {}
        
        self.get_logger().info(f'PS5 controller initialized for {self.namespace} - matching keyboard teleop mapping')
    
    def apply_deadzone(self, value):
        """Apply deadzone to joystick values to prevent drift"""
        if abs(value) < self.deadzone:
            return 0.0
        return value
    
    def joy_callback(self, msg):

        twist = Twist()
  

        left_x = self.apply_deadzone(msg.axes[0])  # Left/right on left stick
        left_y = self.apply_deadzone(msg.axes[1])  # Up/down on left stick
        
        # Get the right stick values - for rotation
        right_x = self.apply_deadzone(msg.axes[3])  # Left/right on right stick
        
        twist.linear.x = left_y * self.speed_linear_x  # Forward/backward
        twist.linear.y = left_x * self.speed_linear_y  # Left/right strafing
        

        twist.angular.z = -right_x * self.speed_angular  # Rotation - negative to make right turn clockwise
        
       
        if len(msg.buttons) > 11 and msg.buttons[11]:  # L3 button pressed - turn left
            twist.angular.z = self.speed_angular  # Positive is counterclockwise
        elif len(msg.buttons) > 12 and msg.buttons[12]:  # R3 button pressed - turn right
            twist.angular.z = -self.speed_angular  # Negative is clockwise

            
        # Use shoulder buttons (L1/R1) for speed adjustments, not rotation
        

        # Use L2/R2 triggers for up/down motion
        l2_trigger = (1.0 - msg.axes[2]) / 2.0  # Convert from 1..-1 to 0..1
        r2_trigger = (1.0 - msg.axes[5]) / 2.0  # Convert from 1..-1 to 0..1
        
        # L2 is up (+z), R2 is down (-z)
        twist.linear.z = (l2_trigger - r2_trigger) * self.speed_linear_z
        
        # Publish velocity command
        self.vel_pub.publish(twist)
        
        # Handle speed adjustments with D-pad (equivalent to q/z, w/x, e/c)
        # D-pad on PS5 is typically mapped to hat axis, but sometimes as buttons
        # We'll support both button-style (buttons) and axis-style (dpad_x, dpad_y)
        
        # First check for d-pad style inputs (may vary by controller/driver)
        dpad_y = 0
        dpad_x = 0
        
        # Try to find D-pad in axes (common mapping)
        if len(msg.axes) >= 7:  # If we have enough axes
            dpad_x = msg.axes[6] if abs(msg.axes[6]) > 0.5 else 0  # D-pad left/right
            dpad_y = msg.axes[7] if abs(msg.axes[7]) > 0.5 else 0  # D-pad up/down
        
        # D-pad up/down: change all speeds (q/z)
        if dpad_y > 0.5 and self.debounce("dpad_up"):
            self.speed_linear_x *= 1.1
            self.speed_linear_y *= 1.1
            self.speed_linear_z *= 1.1
            self.speed_angular *= 1.1
            self.get_logger().info(f"Speed increased by 10% - linear: {self.speed_linear_x:.2f}, angular: {self.speed_angular:.2f}")
        elif dpad_y < -0.5 and self.debounce("dpad_down"):
            self.speed_linear_x *= 0.9
            self.speed_linear_y *= 0.9
            self.speed_linear_z *= 0.9
            self.speed_angular *= 0.9
            self.get_logger().info(f"Speed decreased by 10% - linear: {self.speed_linear_x:.2f}, angular: {self.speed_angular:.2f}")
        
        # D-pad left/right: change angular speed (e/c)
        if dpad_x > 0.5 and self.debounce("dpad_right"):  # D-pad right
            self.speed_angular *= 1.1
            self.get_logger().info(f"Angular speed increased by 10% - angular: {self.speed_angular:.2f}")
        elif dpad_x < -0.5 and self.debounce("dpad_left"):  # D-pad left
            self.speed_angular *= 0.9
            self.get_logger().info(f"Angular speed decreased by 10% - angular: {self.speed_angular:.2f}")
        
        # L1/R1 buttons now only change linear speed (w/x)
        if msg.buttons[4] and self.debounce("L1"):  # L1 button (increase linear speed)
            self.speed_linear_x *= 1.1
            self.speed_linear_y *= 1.1
            self.speed_linear_z *= 1.1
            self.get_logger().info(f"Linear speed increased by 10% - linear: {self.speed_linear_x:.2f}")
        elif msg.buttons[5] and self.debounce("R1"):  # R1 button (decrease linear speed)
            self.speed_linear_x *= 0.9
            self.speed_linear_y *= 0.9
            self.speed_linear_z *= 0.9
            self.get_logger().info(f"Linear speed decreased by 10% - linear: {self.speed_linear_x:.2f}")
        
        # Handle robot mode changes with face buttons
        if msg.buttons[0] and self.debounce("X"):  # X button for TROT
            self.change_mode('TROT')
        elif msg.buttons[1] and self.debounce("Circle"):  # Circle button for STAND
            self.change_mode('STAND')
        elif msg.buttons[2] and self.debounce("Triangle"):  # Triangle button for REST
            self.change_mode('REST')
        elif msg.buttons[3] and self.debounce("Square"):  # Square button for CRAWL
            self.change_mode('CRAWL')
    
    def debounce(self, button_name, debounce_time=0.3):
        """Simple debounce function to prevent multiple rapid button presses"""
        current_time = self.get_clock().now().to_msg().sec
        if button_name not in self.last_speed_change or \
           (current_time - self.last_speed_change[button_name]) > debounce_time:
            self.last_speed_change[button_name] = current_time
            return True
        return False
    
    def change_mode(self, mode):
        cmd = RobotModeCommand()
        cmd.mode = mode
        cmd.robot_id = int(self.namespace.replace('robot', ''))
        self.mode_pub.publish(cmd)
        self.get_logger().info(f'Changed mode to {mode}')

def main(args=None):
    rclpy.init(args=args)
    node = PS5Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
