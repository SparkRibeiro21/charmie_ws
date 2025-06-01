import rclpy
from rclpy.node import Node
import yaml
from ament_index_python.packages import get_package_share_directory
import os
import math
import time

from sensor_msgs.msg import LaserScan

class RadarNode(Node):
    def __init__(self):
        super().__init__('radar_node')

        default_yaml = os.path.join(get_package_share_directory('charmie_description'), 'config', 'radar_params.yaml')
        # Declare and read parameter
        self.declare_parameter('radar_config', default_yaml)
        yaml_path = self.get_parameter('radar_config').get_parameter_value().string_value

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
            else:
                self.get_logger().info(f"Unsupported data_type '{data_type}' for sensor '{sensor_name}'")

        for sensor_name in self.sources:
            self.latest_scans_new_msg[sensor_name] = False

        # Create timer to print stored scans
        timer_period = 1.0 / self.update_frequency
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("RadarNode setup complete.")

   
    def laserscan_callback(self, msg, sensor_name):
        self.latest_scans[sensor_name] = msg
        self.latest_scans_new_msg[sensor_name] = True
        self.get_logger().info(f"[{sensor_name}] Received LaserScan with {len(msg.ranges)} ranges")


    def timer_callback(self):
        # self.get_logger().info("Timer triggered. Current scans:")
        if not self.latest_scans:
            self.get_logger().info("  No scan data received yet.")
        
        for sensor_name, msg in self.latest_scans.items():
            
            if self.latest_scans_new_msg[sensor_name]:
                start_time = time.time()
                valid_ranges = [r for r in msg.ranges if math.isfinite(r)]
                total = sum(valid_ranges)
                count = len(valid_ranges)

                if count > 0:
                    avg = total / count
            
                end_time = time.time()
                elapsed_ms = (end_time - start_time) * 1000.0  # milliseconds


                self.get_logger().info(f"  {sensor_name} - valid ranges: {count}, sum: {total}, avg: {avg}, time: {elapsed_ms}")

                self.latest_scans_new_msg[sensor_name] = False

            
def main(args=None):
    rclpy.init(args=args)
    node = RadarNode()
    rclpy.spin(node)
    rclpy.shutdown()
