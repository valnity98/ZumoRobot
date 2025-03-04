#!/usr/bin/env python3

'''
 ------------------------------------------------------------------------------
 File: Log_node.py
 Author: Mutasem Bader
 Description:
    In order to avoid an excessive flood o recurrring log messages in the
    user interface, a special log class has been implemented. it only stores
    important messages that occur once and displays them in a special log field.
    Repeated messages, such as thoes about camera images or engine speeds, are 
    deliberately excluded in order to keep the user interface clear.
 ------------------------------------------------------------------------------ 
'''

import rclpy  # ROS2 Python client library
from rclpy.node import Node  # For creating a ROS2 node
from std_msgs.msg import String  # ROS2 message
from rclpy.exceptions import InvalidHandle

class LogPublisher(Node):
    """
    ROS2 Node for publishing log messages.
    """
    def __init__(self):
        super().__init__('log_node')
        self.publisher = self.create_publisher(String, 'log_messages', 10)

    def log(self, message, level="info"):
        """
        Publishes a log message with the specified level.
        """

        if not rclpy.ok():
            print(f"ROS 2 context is not valid. Skipping log: {message}")
            return
        try:
            formatted_message = f"{level.upper()}: {message}"
            self.publisher.publish(String(data=formatted_message))

            if level.upper() == "ERROR":
                self.get_logger().error(f"Published log: {formatted_message}")
            
            elif level.upper() == "DEBUG":
                self.get_logger().debug(f"Published log: {formatted_message}")

            elif level.upper() == "WARN":
                self.get_logger().warn(f"Published log: {formatted_message}")

            else:
                self.get_logger().info(f"Published log: {formatted_message}")
        except InvalidHandle:
        # Context or publisher is invalid, handle gracefully
            print(f"Failed to publish log: {formatted_message} (Invalid context)")
        except Exception as e:
            print(f"Unexpected error while logging: {e}")


def main():
    try:
        rclpy.init()  # Initialize rclpy before creating any nodes

        # Now we can create the LogPublisher node
        log_publisher = LogPublisher()
        log_publisher.log("ROS2 Node initialized and log publishing started.")

        # Spin the node to keep it running
        rclpy.spin(log_publisher)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()  # Ensure that ROS2 is properly shut down when done

if __name__ == "__main__":
    main()
