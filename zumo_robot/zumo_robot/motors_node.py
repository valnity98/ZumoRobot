#!/usr/bin/env python3

"""
Motors Node for ROS 2
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
from std_msgs.msg import Float32MultiArray, Int16MultiArray, Int8
import serial
from Zumo_Library.PIDController import Zumo328PPID
from Zumo_Library.log_node import LogPublisher
import struct
import numpy as np


class Motors(Node):
    """ROS 2 Node for controlling a robot's motors using a PID controller."""

    def __init__(self, log_publisher):
        super().__init__('motors_node')

        self.log_publisher = log_publisher
        # Subscriptions
        self.create_subscription(Float32MultiArray, 'line_coordinates', self.line_coordinates_callback, 10)
        self.create_subscription(Int8, 'robot_command', self.command_callback, 10)

        # Controller activation state (use the Qt application and then change hier to 0)
        self.is_command = 0

        # Serial connection setup
        self.serial_port = self.setup_serial_connection('/dev/ttyACM0', 115200)

        # PID Controller initialization
        self.pid_controller = Zumo328PPID(kp=0.35, kd=0.1, ki = 0.1, max_speed=125.0, aktiv=True)

        # Publisher for motor speed commands
        self.publisher_motor_speeds = self.create_publisher(Int16MultiArray, 'motor_speeds', 10)

        self.log_publisher.log("Motors node is on.")

    def setup_serial_connection(self, port, baudrate):
        """Initializes the serial connection to the Arduino."""

        try:
            serial_port = serial.Serial(port, baudrate, timeout=1)
            self.log_publisher.log(f"Serial port {port} opened successfully.")
            return serial_port
        except serial.SerialException as e:
            self.log_publisher.log(f"Failed to open serial port: {str(e)}", level= "error")
            return None

    def line_coordinates_callback(self, msg):
        """Callback to handle line coordinates and calculate motor speeds."""
        if self.is_command == 0:
            self.get_logger().info("PID Controller is inactive. Ignoring line coordinates.")
            return

        try:
            cX, cY = msg.data  # Extract coordinates
            self.get_logger().info(f"Received line coordinates: cX={cX}, cY={cY}")

            # PID control calculation
            self.pid_controller.control_speed(cX, target_position=334)

            # Get calculated motor speed, 
            left_speed = np.int16(self.pid_controller.get_left_speed())
            right_speed = np.int16(self.pid_controller.get_right_speed())

            # Publish motor speeds
            motor_msg = Int16MultiArray()
            motor_msg.data = [int(left_speed), int(right_speed)]
            self.publisher_motor_speeds.publish(motor_msg)

            # Send commands to Arduino
            self.send_to_arduino(left_speed, right_speed, b'\x11')
        except ValueError as e:
            self.log_publisher.log(f"Invalid line coordinates received: {e}", level="error")
        except Exception as e:
            self.log_publisher.log(f"Unexpected error in line_coordinates_callback: {str(e)}", level="error")

    def send_to_arduino(self, left_speed, right_speed, is_reset_byte):
        """Sends motor speed commands to the Arduino over the serial port."""
        if not self.serial_port or not self.serial_port.is_open:
            self.log_publisher.log("Serial port is not available. Skipping send.",level="error")
            return

        try:
            # Pack speeds as two 16-bit integers
            data_to_send = struct.pack('<hh', left_speed, right_speed)
            start_byte = b'\x02'  # Start byte
            end_byte = b'\x03'  # End byte
            is_reset_byte = is_reset_byte # Reset byte
            full_data = start_byte + data_to_send + is_reset_byte + end_byte

            # Send the data
            self.serial_port.write(full_data)
            self.get_logger().info(f"Sent to Arduino: Left={left_speed}, Right={right_speed}")
        except serial.SerialException as e:
            self.log_publisher.log(f"Failed to send data: {str(e)}", level="error")
        except Exception as e:
            self.log_publisher.log(f"Unexpected error in send_to_arduino: {str(e)}", level="error")

    def command_callback(self, msg: Int8):
        """Activates or deactivates  or RESET the PID controller based on the command message."""
        self.is_command = msg.data
      
        if self.is_command == 0:
            # Stop motors when deactivated
            self.pid_controller.set_left_speed(0)
            self.pid_controller.set_right_speed(0)
            self.send_to_arduino(0, 0, b'\x11')
        elif self.is_command == 2:
            # only Reset encoder 
            left_speed = np.int16(self.pid_controller.get_left_speed())
            right_speed = np.int16(self.pid_controller.get_right_speed())
            self.send_to_arduino(left_speed, right_speed, b'\x12')

    def close(self):
        """Closes the serial connection when the node shuts down."""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.log_publisher.log("Closed serial port.")


def main(args=None):
    """Main function to initialize and spin the ROS 2 node."""
    rclpy.init(args=args)
    log_publisher = LogPublisher()
    node = Motors(log_publisher)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
       node.send_to_arduino(0, 0, b'\x11')
       log_publisher.log("Node interrupted by user.")
    finally:
        node.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
