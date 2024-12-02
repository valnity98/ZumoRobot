#!/usr/bin/env python3

"""
Path Mapping Node for a Zumo Robot with Differential Drive (Left and Right Motor Encoders).
This node uses data from the camera (cX - lateral deviation from the line) 
and encoder data (left_motor, right_motor) to create a map of the robot's path.

The robot follows a line, and each movement is recorded on a grid map.
This map is published as an OccupancyGrid message in ROS 2.

Dependencies:
- ROS 2
- rclpy (ROS 2 Python library)
- sensor_msgs (for image data, if needed)
- OpenCV (for line detection via camera)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose




class PathMappingNode(Node):
    def __init__(self):
        super().__init__('path_mapping_node')

        # Subscribe to the coordinates from the camera (line detection) and encoder data
        self.create_subscription(Float32MultiArray, 'line_coordinates', self.coordinates_callback, 10)
        self.create_subscription(Float32MultiArray, 'encoder_data', self.encoder_callback, 10)

        # Publisher for the map (OccupancyGrid)
        self.map_pub = self.create_publisher(OccupancyGrid, 'path_map', 10)

        # Internal variables for map and robot's current position
        self.map_size = 200  # Size of the map (200x200 cells)
        self.resolution = 0.01  # Resolution: 1 cm per cell
        self.map_data = [[-1 for _ in range(self.map_size)] for _ in range(self.map_size)]  # Initialize map with unknown cells
        self.current_position = [self.map_size // 2, self.map_size // 2]  # Start in the center of the map

        # Robot's parameters (wheel radius and wheelbase)
        self.wheel_radius = 0.03  # Radius of the wheels (in meters)
        self.wheelbase = 0.1  # Distance between the two wheels (in meters)

    def coordinates_callback(self, msg):
        """
        Callback function for receiving coordinates from the camera.
        The camera provides lateral deviation 'cX' (side shift from the line).
        This is used to update the robot's position in the X direction.
        """
        if len(msg.data) == 2:
            cX, _ = msg.data  # cX is the lateral deviation from the line

            # Correct the lateral deviation and update the X position
            corrected_x = int(cX / self.resolution)  # Convert to map coordinates
            self.current_position[0] += corrected_x  # Update X position based on deviation

            # Ensure the position is within the map boundaries
            self.current_position[0] = max(0, min(self.map_size - 1, self.current_position[0]))

            # Log the current position for debugging
            self.get_logger().info(f"Current Position: {self.current_position[0]}, {self.current_position[1]}")

            # Update the map with the new position
            self.update_map(self.current_position)

    def encoder_callback(self, msg):
        """
        Callback function for receiving encoder data (left_motor and right_motor).
        The encoder data provides the robot's movement from both motors.
        This updates the robot's position on the map.
        """
        if len(msg.data) == 2:
            left_motor, right_motor = msg.data  # Encoder data for left and right motor

            # Calculate the change in position using the encoder data
            delta_left = left_motor * self.wheel_radius  # Distance moved by the left wheel
            delta_right = right_motor * self.wheel_radius  # Distance moved by the right wheel

            # Calculate the average distance moved (delta_x)
            delta_x = (delta_left + delta_right) / 2  # Forward movement is the average of the two wheels

            # Calculate the change in orientation (delta_y)
            delta_theta = (delta_right - delta_left) / self.wheelbase  # Difference determines turning

            # Update the robot's position
            self.current_position[0] += int(delta_x / self.resolution)  # Update X position
            self.current_position[1] += int(delta_theta / self.resolution)  # Update Y (orientation) position

            # Ensure the position is within the map boundaries
            self.current_position[0] = max(0, min(self.map_size - 1, self.current_position[0]))
            self.current_position[1] = max(0, min(self.map_size - 1, self.current_position[1]))

            # Log the updated position for debugging
            self.get_logger().info(f"Updated Position: {self.current_position[0]}, {self.current_position[1]}")

            # Update the map with the new position
            self.update_map(self.current_position)

    def update_map(self, position):
        """
        Update the map with the current robot position.
        The position is marked as 100 (path) on the grid.
        """
        x, y = position
        if 0 <= x < self.map_size and 0 <= y < self.map_size:
            self.map_data[x][y] = 100  # Mark the cell as part of the path

        # Publish the updated map
        self.publish_map()

    def publish_map(self):
        """
        Convert the map to an OccupancyGrid message and publish it.
        The map is published periodically to visualize the robot's path.
        """
        # Create OccupancyGrid message
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = "map"
        map_msg.info.resolution = self.resolution
        map_msg.info.width = self.map_size
        map_msg.info.height = self.map_size
        map_msg.info.origin.position.x = -self.map_size * self.resolution / 2
        map_msg.info.origin.position.y = -self.map_size * self.resolution / 2
        map_msg.info.origin.orientation.w = 1.0

        # Flatten the map data to a 1D array (100 for path, -1 for unknown)
        map_msg.data = [cell for row in self.map_data for cell in row]
        self.map_pub.publish(map_msg)

def main(args=None):
    """
    Main function to initialize and run the ROS 2 node.
    """
    rclpy.init(args=args)  # Initialize ROS 2
    node = PathMappingNode()  # Create the PathMappingNode instance
    rclpy.spin(node)  # Keep the node running
    rclpy.shutdown()  # Shut down ROS 2

if __name__ == '__main__':
    main()