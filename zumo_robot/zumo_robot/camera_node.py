#!/usr/bin/env python3
"""
Camera Node for ROS 2
Author: Mutasem Bader, Felix Biermann
Description:
    - Captures frames from a camera using OpenCV.
    - Publishes detected line coordinates (x, y) as Float32MultiArray on the 'line_coordinates' topic.
    - Publishes the camera frames as ROS 2 Image messages on the 'camera_topic' topic.

Requirements:
    - ROS 2 installation
    - OpenCV and cv_bridge libraries
"""

import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Base class for ROS 2 nodes
from std_msgs.msg import Float32MultiArray  # Message type for publishing coordinates
import cv2  # OpenCV library for image processing
from sensor_msgs.msg import Image  # ROS 2 Image message type
from cv_bridge import CvBridge  # Bridge between OpenCV and ROS

class CameraNode(Node):
    """ROS 2 Node for camera operations and publishing line coordinates and frames."""

    def __init__(self):
        super().__init__('camera_node')

        # Publishers
        self.publisher_coordinates = self.create_publisher(Float32MultiArray, 'line_coordinates', 10)
        self.publisher_image = self.create_publisher(Image, 'camera_topic', 10)

        # CvBridge for OpenCV-ROS conversions
        self.bridge = CvBridge()

        # Initialize camera
        self.cap = cv2.VideoCapture(2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # Error handling for camera initialization
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera. Please check the camera connection.")
            rclpy.shutdown()
            return

        # Timers for periodic publishing
        self.timer_coordinates = self.create_timer(0.1, self.publish_coordinates)
        self.timer_frame = self.create_timer(0.1, self.publish_frame)

    def publish_frame(self):
        """Reads a frame from the camera and publishes it as a ROS Image message."""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Failed to read frame from camera.")
            return

        try:
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_image.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f"Error converting frame to ROS Image message: {e}")

    def publish_coordinates(self):
        """Processes the camera frame to find line coordinates and publishes them."""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Failed to read frame for line coordinates.")
            return

        # Bestimmen der Höhe und Breite des Frames
        # height, width = frame.shape[:2]
        y_start, y_end = 400, 450
        strip = frame[y_start:y_end, :]

        try:
            gray = cv2.cvtColor(strip, cv2.COLOR_BGR2GRAY)
            _, thresholded = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
            contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"]) + y_start

                    # Publish coordinates
                    msg = Float32MultiArray()
                    msg.data = [float(cX), float(cY)]
                    self.publisher_coordinates.publish(msg)
                    self.get_logger().info(f"Published coordinates: cX={cX}, cY={cY}")
                    # Zeichnen des Schwerpunkts als roten Punkt auf dem Streifen (nur zur Visualisierung)
                    cv2.circle(strip, (cX, cY - y_start), 5, (0, 0, 255), -1) 
                    # Zeichne die größte Kontur auf dem Streifen zur Visualisierung
                    cv2.drawContours(strip, [largest_contour], -1, (0, 255, 0), 2)  # Zeichnet die Kontur in Grün
        except Exception as e:
            self.get_logger().error(f"Error processing coordinates: {e}")

        # Setze den bearbeiteten Streifen wieder in das Originalbild ein
        frame[y_start:y_end, :] = strip

        # Debugging visualization (optional)
        cv2.imshow("Line Detection", strip)
        cv2.imshow("Threshold", thresholded)
        cv2.imshow("Original", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    """Main function to initialize and spin the ROS 2 node."""
    rclpy.init(args=args)
    node = CameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user.")
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


