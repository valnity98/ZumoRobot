#!/usr/bin/env python3

"""
Arduino Node for ROS 2
Author: Mutasem Bader
Description:
    - Reads encoder data from Arduino over a serial connection.
    - Publishes the encoder data on the 'encoder_data' topic.
Requirements:
    - ROS 2 installation
    - serial libraries
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial
from zumo_robot.zumo_robot.log_node import LogPublisher


class Encoder(Node):
    """ROS 2 Node to read encoder data from Arduino and publish it."""

    def __init__(self, log_publisher):
        super().__init__('encoder_node')

        self.log_publisher = log_publisher

        # Publisher for encoder data
        self.publisher_ = self.create_publisher(Int32MultiArray, 'encoder_data', 10)

        # Initialize serial port
        self.serial_port = self.setup_serial_connection('/dev/ttyACM0', 115200) # Port "ttyACM0" for leonardo Board

        self.log_publisher.log("Encoder node is on")

        # Timer to periodically fetch encoder data
        self.create_timer(0.1, self.read_encoder_data)

    def setup_serial_connection(self, port, baudrate):
        """Initializes the serial connection to the Arduino."""
        try:
            serial_port = serial.Serial(port, baudrate, timeout=1)
            self.log_publisher.log(f"Serial port {port} opened successfully.")
            return serial_port
        except serial.SerialException as e:
            self.log_publisher.log(f"Failed to open serial port: {str(e)}", level= "error")
            return None

    def read_encoder_data(self):
        """Reads encoder data from the Arduino and publishes it."""
        if self.serial_port and self.serial_port.in_waiting >= 10:
            try:
                # Read the 10-byte packet from Arduino
                Encoder_Data = self.serial_port.read(10)
                
                if len(data) == 10 and data[0] == 0x02 and data[9] == 0x03:  # Check for STX and ETX
                    # Extract encoder data
                    right_encoder = int.from_bytes(Encoder_Data[1:5], byteorder='little', signed=True)
                    left_encoder = int.from_bytes(Encoder_Data[5:9], byteorder='little', signed=True)

                    # Create and publish the message
                    msg = Int32MultiArray()
                    msg.data = [left_encoder, right_encoder]
                    self.publisher_.publish(msg)

                    self.get_logger().info(f"Left: {left_encoder}, Right: {right_encoder}")
                else:
                    self.log_publisher.log("Received invalid data or incomplete packet.", level= "warn")
            except serial.SerialException as e:
                self.log_publisher.log(f"Serial read error: {str(e)}",level="error")
            except Exception as e:
                self.log_publisher.log(f"Unexpected error while reading encoder data: {str(e)}", level= "error")

    def close(self):
        """Closes the serial connection when the node shuts down."""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.log_publisher.log("Closed serial port.")


def main(args=None):
    """Main function to initialize and spin the ROS 2 node."""
    rclpy.init(args=args)
    log_publisher = LogPublisher()
    node = Encoder(log_publisher)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        log_publisher.log("Node interrupted by user.")
    finally:
        node.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



