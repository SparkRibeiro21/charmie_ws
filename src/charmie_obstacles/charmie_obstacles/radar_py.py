import rclpy
from rclpy.node import Node
import yaml
from ament_index_python.packages import get_package_share_directory
import os
import math
import time

from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Point
from charmie_interfaces.msg import ListOfPoints


class RadarNode(Node):
    def __init__(self):
        super().__init__('radar_node')

        default_yaml = os.path.join(get_package_share_directory('charmie_description'), 'config', 'radar_params.yaml')
        # Declare and read parameter
        self.declare_parameter('radar_config', default_yaml)
        yaml_path = self.get_parameter('radar_config').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info(f"Loading config from: {yaml_path}")

        # Load YAML file
        if not os.path.exists(yaml_path):
            self.get_logger().error(f"YAML file not found at path: {yaml_path}")
            return

        try:
            with open(yaml_path, 'r') as f:
                config = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to parse YAML: {e}")
            return

        radar = config.get('radar_node')
        if not radar:
            self.get_logger().error("Missing 'radar_node' key in YAML file.")
            return

        self.robot_base_frame = radar.get('robot_base_frame', 'N/A')
        self.update_frequency = radar.get('update_frequency', 10.0)
        sources_str = radar.get('observation_sources', '')
        self.sources = sources_str.split()

        self.latest_scans = {}  # Dictionary to store latest messages
        self.latest_scans_new_msg = {}
        self.subscribers = []

        self.get_logger().info(f"Robot Base Frame: {self.robot_base_frame}")
        self.get_logger().info(f"Update Frequency: {self.update_frequency}")
        self.get_logger().info(f"Observation sources: {sources_str}")

        for sensor_name in self.sources:
            sensor = radar.get(sensor_name)
            if not sensor:
                self.get_logger().warn(f"Sensor config block '{sensor_name}' not found.")
                continue

            topic = sensor.get('topic', 'N/A')
            data_type = sensor.get('data_type', 'N/A')
            max_obstacle_height = sensor.get('max_obstacle_height', -1.0)
            min_obstacle_height = sensor.get('min_obstacle_height', -1.0)
            obstacle_max_range = sensor.get('obstacle_max_range', -1.0)
            obstacle_min_range = sensor.get('obstacle_min_range', -1.0)

            self.get_logger().info(f"--- Sensor: {sensor_name} ---")
            self.get_logger().info(f"Topic: {topic}")
            self.get_logger().info(f"Data type: {data_type}")
            self.get_logger().info(f"Obstacle height: {min_obstacle_height:.2f} to {max_obstacle_height:.2f}")
            self.get_logger().info(f"Obstacle range: {obstacle_min_range:.2f} to {obstacle_max_range:.2f}")

            if data_type == "LaserScan":
                sub = self.create_subscription(LaserScan, topic, lambda msg, s=sensor_name:self.laserscan_callback(msg, s), 10)
                self.subscribers.append(sub)
                self.get_logger().info(f"Subscribed to sensor '{sensor_name}' of message type '{data_type}'")
            elif data_type == "PointCloud2":
                sub = self.create_subscription(PointCloud2, topic, lambda msg, s=sensor_name:self.pointcloud_callback(msg, s), 10)
                self.subscribers.append(sub)
                self.get_logger().info(f"Subscribed to sensor '{sensor_name}' of message type '{data_type}'")
            else:
                self.get_logger().info(f"Unsupported data_type '{data_type}' for sensor '{sensor_name}'")

        for sensor_name in self.sources:
            self.latest_scans_new_msg[sensor_name] = False

        self.points_publisher = self.create_publisher(ListOfPoints, 'radar_points', 10)

        # Create timer to print stored scans
        timer_period = 1.0 / self.update_frequency
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("RadarNode setup complete.")

    def laserscan_callback(self, msg, sensor_name):
        self.latest_scans[sensor_name] = msg
        self.latest_scans_new_msg[sensor_name] = True
        self.get_logger().info(f"[{sensor_name}] Received LaserScan with {len(msg.ranges)} ranges")

    def pointcloud_callback(self, msg, sensor_name):
        self.latest_scans[sensor_name] = msg
        self.latest_scans_new_msg[sensor_name] = True
        self.get_logger().info(f"[{sensor_name}] Received PointCloud2 with height {msg.height}, width {msg.width}")

    def timer_callback(self):
        # self.get_logger().info("Timer triggered. Current scans:")
        if not self.latest_scans:
            self.get_logger().info("  No scan data received yet.")
            return

        all_points = []
        start_time = time.time()

        for sensor_name, msg in self.latest_scans.items():
            
            if self.latest_scans_new_msg[sensor_name]:

                msg_type = type(msg)
                if msg_type == LaserScan:

                    angle = msg.angle_min
                    angle_increment = msg.angle_increment
                    frame_id = msg.header.frame_id  # original frame of the LaserScan

                    for r in msg.ranges:
                        if math.isfinite(r):
                            x = r * math.cos(angle)
                            y = r * math.sin(angle)
                            point_stamped = PointStamped()
                            point_stamped.header.stamp = msg.header.stamp
                            point_stamped.header.frame_id = frame_id
                            point_stamped.point.x = x
                            point_stamped.point.y = y
                            point_stamped.point.z = 0.0

                            try:
                                transformed_point = self.tf_buffer.transform(
                                    point_stamped,
                                    self.robot_base_frame,
                                    timeout=rclpy.duration.Duration(seconds=0.1)
                                )
                                all_points.append(transformed_point.point)
                            except Exception as e:
                                self.get_logger().warn(
                                    f"TF transform failed for {sensor_name}: {e}"
                                )
                        angle += angle_increment
                
                elif msg_type == PointCloud2:
                    for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                        ps = PointStamped()
                        ps.header = msg.header
                        ps.point.x = p[0]
                        ps.point.y = p[1]
                        ps.point.z = p[2]
                        
                        try:
                            transformed_point = self.tf_buffer.transform(
                                ps, 
                                self.robot_base_frame, 
                                timeout=rclpy.duration.Duration(seconds=0.1)
                            )
                            all_points.append(transformed_point.point)
                        except Exception as e:
                            self.get_logger().warn(
                                f"TF transform failed for {sensor_name}: {e}"
                            )

                self.latest_scans_new_msg[sensor_name] = False
    
        if all_points:
            elapsed_ms = (time.time() - start_time) * 1000.0  # milliseconds
            self.get_logger().info(f"Converted {len(all_points)} points in {elapsed_ms:.2f} ms")
            msg = ListOfPoints()
            msg.coords = all_points
            self.points_publisher.publish(msg)

            
def main(args=None):
    rclpy.init(args=args)
    node = RadarNode()
    rclpy.spin(node)
    rclpy.shutdown()
