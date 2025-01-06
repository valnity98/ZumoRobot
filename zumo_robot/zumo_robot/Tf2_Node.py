import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped, Pose

class TransformPublisherNode(Node):
    def __init__(self):
        super().__init__('transform_publisher_node')

        # TF2 Transform Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscribe to robot position
        self.create_subscription(Pose, 'robot_position', self.pose_callback, 10)

    def pose_callback(self, msg):
        x = msg.position.x
        y = msg.position.y
        z = msg.orientation.z
        w = msg.orientation.w
        self.publish_transform(x, y, z, w)

    def publish_transform(self, x, y, z,w):
        """
        Broadcast the transformation from 'base_link' to 'map' using tf2.
        This transformation tells where the robot's frame (base_link) is in the map frame.
        """
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_link"

        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0

        # Convert orientation to quaternion
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = z
        transform.transform.rotation.w = w
        
        self.get_logger().info(f"Transform: x={x}, y={y}, z={z}, w={w}")

        # Send the transformation
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    transform_publisher = TransformPublisherNode()    
    rclpy.spin(transform_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
