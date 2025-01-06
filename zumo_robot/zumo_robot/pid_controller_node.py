#!/usr/bin/env python3
"""
PID Controller Node for ROS 2
Author: Mutasem Bader
Description:
    - Receives line coordinates from 'line_coordinates' topic.
    - Controls motors using PID logic and sends speed commands to Arduino.
    - Activates or deactivates the PID controller based on 'robot_command' topic.

Requirements:
    - ROS 2 installation
    - numpy, serial, Zumo_Library for PIDController
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool, Int16MultiArray
import serial
from Zumo_Library.PIDController import Zumo328PPID
import struct
import numpy as np


class PIDControllerNode(Node):
    """ROS 2 Node for controlling a robot's motors using a PID controller."""

    def __init__(self):
        super().__init__('pid_controller_node')

        # Subscriptions
        self.create_subscription(Float32MultiArray, 'line_coordinates', self.line_coordinates_callback, 10)
        self.create_subscription(Bool, 'robot_command', self.command_callback, 10)

        # Controller activation state (use the Qt application and then change hier to False)
        self.is_active = True

        # Serial connection setup
        self.serial_port = self.setup_serial_connection('/dev/ttyACM0', 115200)

        # PID Controller initialization
        self.pid_controller = Zumo328PPID(max_speed=150.0)

        # Publisher for motor speed commands
        self.publisher_motor_speeds = self.create_publisher(Int16MultiArray, 'motor_speeds', 10)

    def setup_serial_connection(self, port, baudrate):
        """Initializes the serial connection to the Arduino."""

        try:
            serial_port = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(f"Serial port {port} opened successfully.")
            return serial_port
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {str(e)}")
            return None

    def line_coordinates_callback(self, msg):
        """Callback to handle line coordinates and calculate motor speeds."""
        if not self.is_active:
            self.get_logger().info("PID Controller is inactive. Ignoring line coordinates.")
            return

        try:
            cX, cY = msg.data  # Extract coordinates
            #self.get_logger().info(f"Received line coordinates: cX={cX}, cY={cY}")

            # PID control calculation
            self.pid_controller.control_speed(cX, target_position=334, kp=0.6, kd=0.3, aktiv=False)

            # Get calculated motor speeds
            left_speed = np.int16(self.pid_controller.get_left_speed())
            right_speed = np.int16(self.pid_controller.get_right_speed())

            # Publish motor speeds
            motor_msg = Int16MultiArray()
            motor_msg.data = [int(left_speed), int(right_speed)]
            self.publisher_motor_speeds.publish(motor_msg)

            # Send commands to Arduino
            self.send_to_arduino(left_speed, right_speed)
        except ValueError as e:
            self.get_logger().error(f"Invalid line coordinates received: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in line_coordinates_callback: {str(e)}")

    def send_to_arduino(self, left_speed, right_speed):
        """Sends motor speed commands to the Arduino over the serial port."""
        if not self.serial_port or not self.serial_port.is_open:
            self.get_logger().error("Serial port is not available. Skipping send.")
            return

        try:
            # Pack speeds as two 16-bit integers
            data_to_send = struct.pack('<hh', left_speed, right_speed)
            start_byte = b'\x02'  # Start byte
            end_byte = b'\x03'  # End byte
            full_data = start_byte + data_to_send + end_byte

            # Send the data
            self.serial_port.write(full_data)
            self.get_logger().info(f"Sent to Arduino: Left={left_speed}, Right={right_speed}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to send data: {str(e)}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in send_to_arduino: {str(e)}")

    def command_callback(self, msg: Bool):
        """Activates or deactivates the PID controller based on the command message."""
        self.is_active = msg.data
        state = "active" if self.is_active else "inactive"
        self.get_logger().info(f"PID Controller is now {state}.")

        if not self.is_active:
            # Stop motors when deactivated
            self.send_to_arduino(0, 0)

    def close(self):
        """Closes the serial connection when the node shuts down."""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info("Closed serial port.")


def main(args=None):
    """Main function to initialize and spin the ROS 2 node."""
    rclpy.init(args=args)
    node = PIDControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user.")
    finally:
        node.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


