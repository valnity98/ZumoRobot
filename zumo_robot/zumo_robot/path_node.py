#!/usr/bin/env python3

"""
Pathmapping Node for ROS 2
Author: Mutasem Bader

Description:
    This node implements a path mapping algorithm for a mobile robot. 
    It processes encoder data to track the robot's position, updates an occupancy grid map, 
    and visualizes the robot's path using ROS 2 visualization markers.

Features:
    - Real-time robot position tracking using encoder data.
    - Occupancy grid map updates for path visualization.
    - Marker-based visualization for RViz.
    - Modular handling of map and path data.

Requirements:
    - ROS 2 installation
    - Python libraries: numpy, math
    - ROS 2 messages: std_msgs, nav_msgs, geometry_msgs, visualization_msgs

How to Run:
    1. Launch your ROS 2 environment.
    2. Ensure the topic `encoder_data` is publishing valid Int32MultiArray messages.
    3. Run the node using `ros2 run <package_name> <node_script.py>`.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import Marker
import math
import numpy as np

class PathMappingNode(Node):
    def __init__(self):
        super().__init__('path_mapping_node')

        # Subscribe to the encoder data (left_motor, right_motor)
        self.create_subscription(Int32MultiArray, 'encoder_data', self.encoder_callback, 10)
        # Publisher for the robot position, map, and visualization marker
        self.position_pub = self.create_publisher(Pose, 'robot_position', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, 'path_map', 10)
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)

        # Internal variables for map and robot's current position
        self.map_size = 400  # Map size 400x400 cells
        self.resolution = 0.01  # 1 cm per cell
        self.map_data = np.full((self.map_size, self.map_size), -1, dtype=np.int8)  # Unknown cells
        self.current_position = [self.map_size / 2, self.map_size / 2, 0.0]  # Start in the center of the map

        # Wheel and gear parameters
        self.wheel_radius = 0.019  # Wheel radius (meters)
        self.gear_ratio = 75.81  # Gear ratio
        self.cpr = (self.gear_ratio * 12) / 2  # Counts per Revolution (~909.7 for 75:1 gear)
        self.wheeltrack = 0.09  # Wheeltrack (meters)

        # Initialize previous encoder values
        self.prev_left_motor = 0
        self.prev_right_motor = 0

    def encoder_callback(self, msg):
        """
        Callback function for receiving encoder data (left_encoder and right_encoder).
        The encoder data provides the robot's movement from both motors.
        This updates the robot's position on the map.
        """
        if len(msg.data) == 2:
            left_encoder, right_encoder = msg.data

            # Calculate tick differences
            ticks_left = left_encoder - self.prev_left_motor
            ticks_right = right_encoder - self.prev_right_motor

            # Convert ticks to meters
            delta_left = (ticks_left / self.cpr) * (2 * math.pi * self.wheel_radius)
            delta_right = (ticks_right / self.cpr) * (2 * math.pi * self.wheel_radius)

            # Calculate average movement (forward)
            delta_movement = (delta_left + delta_right) / 2

            # Calculate rotation (rotation around the Z-axis)
            delta_theta = (delta_right - delta_left) / self.wheeltrack

            # Update the robot position
            self.current_position[0] += delta_movement * math.cos(self.current_position[2]) / self.resolution  # X movement
            self.current_position[1] += delta_movement * math.sin(self.current_position[2]) / self.resolution  # Y movement
            self.current_position[2] += delta_theta  # Orientation

            # Normalize orientation to [-pi, pi]
            self.current_position[2] = (self.current_position[2] + math.pi) % (2 * math.pi) - math.pi

            # Limit the position to the map's boundaries
            self.current_position[0] = max(0, min(self.map_size - 1, self.current_position[0]))
            self.current_position[1] = max(0, min(self.map_size - 1, self.current_position[1]))

            # Log the updated position for debugging
            self.get_logger().info(f"Position updated: X={self.current_position[0]:.2f}, Y={self.current_position[1]:.2f}, Theta={math.degrees(self.current_position[2]):.2f}")

            # Update map and visualization
            self.update_map(self.current_position[0], self.current_position[1])
            self.publish_position()
            self.visualize_path()

            # Store current encoder values for the next callback
            self.prev_left_motor = left_encoder
            self.prev_right_motor = right_encoder

    def publish_position(self):
        """
        Publish the robot's current position as a Pose message.
        """
        pose_msg = Pose()
        pose_msg.position.x = (-self.map_size * self.resolution / 2) + self.current_position[0] * self.resolution
        pose_msg.position.y = (-self.map_size * self.resolution / 2) + self.current_position[1] * self.resolution
        pose_msg.orientation.z = math.sin(self.current_position[2] / 2)
        pose_msg.orientation.w = math.cos(self.current_position[2] / 2)
        self.position_pub.publish(pose_msg)

    def update_map(self, x_index, y_index):
        """
        Update the map with the current robot position.
        The position is marked as 100 (path) on the grid.
        """
        # Limit within bounds
        x_index = max(0, min(self.map_size - 1, int(x_index)))
        y_index = max(0, min(self.map_size - 1, int(y_index)))

        if self.map_data[y_index, x_index] != 1:  # Avoid redundant marking
            self.map_data[y_index, x_index] = 1

        # Publish map
        self.publish_map()

    def publish_map(self):
        """
        Convert the map to an OccupancyGrid message and publish it.
        """
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = "map"
        map_msg.info.resolution = self.resolution
        map_msg.info.width = self.map_size
        map_msg.info.height = self.map_size
        map_msg.info.origin.position.x = -self.map_size * self.resolution / 2
        map_msg.info.origin.position.y = -self.map_size * self.resolution / 2
        map_msg.info.origin.orientation.w = 1.0
        map_msg.data = self.map_data.flatten().tolist()
        self.map_pub.publish(map_msg)

    def visualize_path(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "path"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0

        path_points = np.argwhere(self.map_data == 100)
        for p in path_points:
            point = Point()
            point.x = p[1] * self.resolution - (self.map_size * self.resolution / 2)
            point.y = p[0] * self.resolution - (self.map_size * self.resolution / 2)
            point.z = 0.0
            marker.points.append(point)

        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = PathMappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
