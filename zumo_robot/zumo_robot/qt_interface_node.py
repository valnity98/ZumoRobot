#!/usr/bin/env python3
'''
Qt App Node for ROS 2
Author: Mutasem Bader
Description:
    - This Qt application communicates with a robot using ROS 2.
    - The app allows sending start/stop commands to control the robot and displays the camera feed from the robot.
    - The ROS 2 node handles video stream reception, robot control, and communication via serial.
    - OpenCV and cv_bridge are used to display the video feed in the GUI.
    - ROS2 communication runs in a separate thread to avoid blocking the GUI event loop.
Requirements:
    - ROS 2 installation
    - serial libraries (`pyserial`)
    - OpenCV (`cv2`)
    - PyQt5
    - cv_bridge (for ROS2 <-> OpenCV image conversion)
'''

import sys  # System functions such as command line arguments
import os  # Operating system functions like path management
import rclpy  # ROS2 Python client library
from rclpy.node import Node  # For creating a ROS2 node
from std_msgs.msg import Bool  # ROS2 Bool message
from sensor_msgs.msg import Image  # ROS2 Image message for camera data
from cv_bridge import CvBridge ,CvBridgeError  # ROS2 image message to OpenCV conversion
import cv2  # OpenCV library for image processing
from PyQt5.QtWidgets import QApplication, QMainWindow  # GUI components
from PyQt5.QtGui import QImage, QPixmap  # Image display classes
from PyQt5 import uic, QtCore  # UI file loader and QtCore for threading

class ROS2Thread(QtCore.QThread):
    """
    ROS2 Thread class to run the ROS2 node in a background thread
    to prevent blocking the GUI.
    """
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        """Starts the ROS2 node to process messages in the background."""
        try:
            rclpy.spin(self.node)
        except rclpy.shutdown_shutdown_exception:
            self.node.get_logger().info("ROS2 Node stopped successfully.")
        except Exception as e:
            self.node.get_logger().error(f"Unexpected error occurred in ROS2 thread: {e}")

class MainWindow(QMainWindow):
    """
    Main class for the Qt application interface.
    Handles communication with ROS2, robot control, and video display.
    """
    def __init__(self):
        super(MainWindow, self).__init__()

        # Load the UI file
        ui_path = os.path.join(os.path.dirname(__file__), os.getcwd() + '/src/ZumoRobot/zumo_robot/Qt/zumorobot.ui')
        uic.loadUi(ui_path, self)

        # Initialize the ROS2 node
        rclpy.init()
        self.node = Node("qt_ros_interface")
        self.publisher = self.node.create_publisher(Bool, 'robot_command', 10)
        self.video_subscriber = self.node.create_subscription(Image, 'camera_topic', self.display_video, 10)

        self.bridge = CvBridge()  # Initialize CvBridge for converting ROS2 image messages to OpenCV images

        # Connect GUI buttons to their corresponding functions
        self.Start_PushButton.clicked.connect(self.send_start_command)
        self.Stop_PushButton.clicked.connect(self.send_stop_command)

        # Start the ROS2 communication in a background thread to prevent blocking the GUI
        self.ros_thread = ROS2Thread(self.node)
        self.ros_thread.start()

    def send_start_command(self):
        """Sends the start command to the robot system."""
        self._send_robot_command(True)
        self.Status_LineEdit.setText("Robot started")

    def send_stop_command(self):
        """Sends the stop command to the robot system."""
        self._send_robot_command(False)
        self.Status_LineEdit.setText("Robot stopped")

    def _send_robot_command(self, command: bool):
        """Helper function to send commands (start/stop) to the robot via ROS2."""
        try:
            msg = Bool(data=command)
            self.publisher.publish(msg)
            self.get_logger().info(f"Sent {'start' if command else 'stop'} command to the robot.")
        except rclpy.exceptions.RCLException as e:
            self.get_logger().error(f"Failed to send robot command due to ROS2 exception: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in sending robot command: {e}")

    def display_video(self, msg):
        """Handles video stream data from the robot and updates the GUI display."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")  # Convert the ROS image to OpenCV format
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # Convert to RGB format for displaying in the GUI

            # Get image dimensions for creating QImage object
            height, width, channel = frame.shape
            bytes_per_line = 3 * width  # 3 bytes per pixel (RGB)
            q_img = QImage(frame.data, width, height, bytes_per_line, QImage.Format_RGB888)

            # Set the QImage as the image source in the GUI label
            self.Video_Label.setPixmap(QPixmap.fromImage(q_img))
        except CvBridgeError as e:
            self.get_logger().error(f"Error converting ROS image message to OpenCV: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error displaying video: {e}")

    def closeEvent(self, event):
        """Handles cleanup and ROS2 node shutdown when the window is closed."""
        try:
            self.node.destroy_node()  # Ensure the node is destroyed properly
            rclpy.shutdown()  # Shutdown ROS2 gracefully
            event.accept()
        except Exception as e:
            self.get_logger().error(f"Error during shutdown: {e}")
            event.accept()

def main():
    """Main function to run the Qt application."""
    try:
        app = QApplication(sys.argv)
        window = MainWindow()
        window.show()
        sys.exit(app.exec_())
    except Exception as e:
        print(f"Error running the application: {e}")

if __name__ == "__main__":
    main()
