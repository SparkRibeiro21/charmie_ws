import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__("marker_publisher")
        self.publisher = self.create_publisher(Marker, "visualization_marker", 10)
        self.timer = self.create_timer(1.0, self.publish_marker)  # Publish every 1 second

    def publish_marker(self):
        marker = Marker()
        
        # Header - Defines frame and timestamp
        marker.header.frame_id = "map"  # Change to "odom" or "base_link" if needed
        marker.header.stamp = self.get_clock().now().to_msg()

        # Namespace and ID (useful when publishing multiple markers)
        marker.ns = "basic_shapes"
        marker.id = 0  # Each marker must have a unique ID

        # Marker Type (Choose shape)
        marker.type = Marker.CUBE  # Other options: SPHERE, CYLINDER, ARROW, etc.
        
        # Marker Action
        marker.action = Marker.ADD  # Can be ADD, MODIFY, or DELETE

        # Position (x, y, z) and Orientation (quaternion w, x, y, z)
        marker.pose.position.x = 1.0  # Set the X coordinate
        marker.pose.position.y = 2.0  # Set the Y coordinate
        marker.pose.position.z = 0.0  # Set the Z coordinate

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0  # No rotation

        # Scale (size of marker)
        marker.scale.x = 0.5  # Width
        marker.scale.y = 0.5  # Depth
        marker.scale.z = 0.5  # Height

        # Color (RGBA format, values from 0 to 1)
        marker.color.r = 1.0  # Red
        marker.color.g = 0.0  # Green
        marker.color.b = 0.0  # Blue
        marker.color.a = 1.0  # Alpha (1.0 = fully visible, 0.0 = invisible)

        # Lifetime (0 = forever, otherwise, disappears after X seconds)
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        # Frame behavior (Keeps marker always facing the camera if enabled)
        marker.frame_locked = False

        # Publish the marker
        self.publisher.publish(marker)
        self.get_logger().info("Marker published!")

def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
