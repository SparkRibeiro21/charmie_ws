#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "charmie_interfaces/msg/list_of_points.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <yaml-cpp/yaml.h>
#include <chrono>

class RadarNode : public rclcpp::Node {
public:
    RadarNode() : Node("radar_node") {
        
        std::string full_path = ament_index_cpp::get_package_share_directory("charmie_description") + "/config/radar_params.yaml";
        std::string yaml_path = this->declare_parameter<std::string>("radar_config", full_path);
        
        try {
            YAML::Node config = YAML::LoadFile(yaml_path);
            YAML::Node radar = config["radar_node"];
            if (!radar) {
                RCLCPP_ERROR(this->get_logger(), "Missing 'radar_node' key in YAML file.");
                return;
            }
            
            RCLCPP_INFO(this->get_logger(), "Loading config from: %s", yaml_path.c_str());

            robot_base_frame_ = radar["robot_base_frame"] ? radar["robot_base_frame"].as<std::string>() : "N/A";
            double update_frequency = radar["update_frequency"] ? radar["update_frequency"].as<double>() : 10.0;
            robot_radius_ = radar["robot_radius"] ? radar["robot_radius"].as<double>() : 0.0;

            // Get observation_sources string and split it
            std::string sources_str = radar["observation_sources"].as<std::string>();
            std::vector<std::string> sources = split_string(sources_str, ' ');

            RCLCPP_INFO(this->get_logger(), "Robot Base Frame: %s", robot_base_frame_.c_str());
            RCLCPP_INFO(this->get_logger(), "Update Frequency: %.1f ", update_frequency);
            RCLCPP_INFO(this->get_logger(), "Robot Radius: %.2f", robot_radius_);
            RCLCPP_INFO(this->get_logger(), "Observation sources: %s", sources_str.c_str());

            for (const auto& sensor_name : sources) {
                if (!radar[sensor_name]) {
                    RCLCPP_WARN(this->get_logger(), "Sensor config block '%s' not found.", sensor_name.c_str());
                    continue;
                }

                YAML::Node sensor = radar[sensor_name];
                std::string topic = sensor["topic"] ? sensor["topic"].as<std::string>() : "N/A";
                std::string data_type = sensor["data_type"] ? sensor["data_type"].as<std::string>() : "N/A";
                
                double max_obstacle_height = sensor["max_obstacle_height"] ? sensor["max_obstacle_height"].as<double>() : -1.0;
                double min_obstacle_height = sensor["min_obstacle_height"] ? sensor["min_obstacle_height"].as<double>() : -1.0;
                double obstacle_max_range = sensor["obstacle_max_range"] ? sensor["obstacle_max_range"].as<double>() : -1.0;
                double obstacle_min_range = sensor["obstacle_min_range"] ? sensor["obstacle_min_range"].as<double>() : -1.0;

                SensorLimits limits;
                limits.min_height = min_obstacle_height;
                limits.max_height = max_obstacle_height;
                limits.min_range = obstacle_min_range;
                limits.max_range = obstacle_max_range;
                sensor_limits_[sensor_name] = limits;

                RCLCPP_INFO(this->get_logger(), "--- Sensor: %s ---", sensor_name.c_str());
                RCLCPP_INFO(this->get_logger(), "Topic: %s", topic.c_str());
                RCLCPP_INFO(this->get_logger(), "Data type: %s", data_type.c_str());
                RCLCPP_INFO(this->get_logger(), "Obstacle height: %.2f to %.2f", min_obstacle_height, max_obstacle_height);
                RCLCPP_INFO(this->get_logger(), "Obstacle range: %.2f to %.2f", obstacle_min_range, obstacle_max_range);

                if (data_type == "PointCloud2") {
                    bool publish_filtered = sensor["publish_filtered"] ? sensor["publish_filtered"].as<bool>() : true;
                    sensor_publish_filtered_[sensor_name] = publish_filtered;

                    RCLCPP_INFO(this->get_logger(), "Publish Filtered: %s", publish_filtered ? "True" : "False");

                    if (publish_filtered) { // dynamic creates publisher for all point cloud with publish filtered set to true 
                        std::string filtered_topic = topic + "/filtered";
                        auto pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(filtered_topic, 10);
                        filtered_cloud_publishers_[sensor_name] = pub;
                        RCLCPP_INFO(this->get_logger(), "Created publisher for filtered data on topic: %s", filtered_topic.c_str());
                    }
                }
                latest_scans_new_msg_[sensor_name] = false;

                if (data_type == "LaserScan") {
                    auto sub = this->create_subscription<sensor_msgs::msg::LaserScan>(topic, 10, 
                        [this, sensor_name](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                            this->laser_callback(msg, sensor_name);
                        });
                    laser_subscribers_[sensor_name] = sub;
                    RCLCPP_INFO(this->get_logger(), "Subscribed to sensor '%s' of message type '%s'", sensor_name.c_str(), data_type.c_str());
                } else if (data_type == "PointCloud2") {
                    auto sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(topic, 10,
                        [this, sensor_name](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                            this->cloud_callback(msg, sensor_name);
                        });
                    cloud_subscribers_[sensor_name] = sub;
                    RCLCPP_INFO(this->get_logger(), "Subscribed to sensor '%s' of message type 'PointCloud2'", sensor_name.c_str());

                } else {
                    RCLCPP_WARN(this->get_logger(), "Unsupported data type '%s' for sensor '%s'", data_type.c_str(), sensor_name.c_str());
                }

            }

            RCLCPP_INFO(this->get_logger(), "--- Finisehd Sensors Setup ---");
            
            // TF2 setup
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            // Publisher setup
            points_publisher_ = this->create_publisher<charmie_interfaces::msg::ListOfPoints>("radar_points", 10);
            // filtered_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/camera/depth/points_filtered", 10);
            //discarded_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/camera/depth/points_discarded", 10);


            timer_ = this->create_wall_timer(
                std::chrono::duration<double>(1.0 / update_frequency),
                std::bind(&RadarNode::timer_callback, this)
            );

        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load or parse radar_params.yaml: %s", e.what());
        }

        RCLCPP_INFO(this->get_logger(), "RadarNode Setup Complete.");
    }

private:

    struct SensorLimits {
        double min_height;
        double max_height;
        double min_range;
        double max_range;
    };

    std::unordered_map<std::string, rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> laser_subscribers_;
    std::unordered_map<std::string, rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> cloud_subscribers_;
    std::unordered_map<std::string, sensor_msgs::msg::LaserScan::SharedPtr> latest_scans_;
    std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr> latest_clouds_;
    std::unordered_map<std::string, bool> latest_scans_new_msg_;
    std::unordered_map<std::string, bool> latest_clouds_new_msg_;
    std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> filtered_cloud_publishers_;
    rclcpp::Publisher<charmie_interfaces::msg::ListOfPoints>::SharedPtr points_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;
    //rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr discarded_cloud_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string robot_base_frame_;
    double robot_radius_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unordered_map<std::string, SensorLimits> sensor_limits_;
    std::unordered_map<std::string, bool> sensor_publish_filtered_;

    std::vector<std::string> split_string(const std::string &input, char delimiter) {
        std::stringstream ss(input);
        std::string item;
        std::vector<std::string> tokens;
        while (std::getline(ss, item, delimiter)) {
            if (!item.empty()) {
                tokens.push_back(item);
            }
        }
        return tokens;
    }

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg, const std::string &source_name) {
        latest_scans_[source_name] = msg;
        latest_scans_new_msg_[source_name] = true;
        RCLCPP_INFO(this->get_logger(), "[%s] Received LaserScan with %lu ranges", source_name.c_str(), msg->ranges.size());
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string &source_name) {
        latest_clouds_[source_name] = msg;
        latest_clouds_new_msg_[source_name] = true;
        RCLCPP_INFO(this->get_logger(), "[%s] Received PointCloud2 with width %u points.", source_name.c_str(), msg->width);
    
        // If publish_filtered is true for this sensor, publish raw PointCloud2
        if (sensor_publish_filtered_[source_name] && filtered_cloud_publishers_.count(source_name)) {
            auto pub = filtered_cloud_publishers_[source_name];
            sensor_msgs::msg::PointCloud2 output_msg = *msg;  // clone input
            pub->publish(output_msg);
            //RCLCPP_INFO(this->get_logger(), "[%s] Published filtered cloud to: %s",
            //            source_name.c_str(), pub->get_topic_name());
        }
    
    }

    void timer_callback() {
        if (latest_scans_.empty() && latest_clouds_.empty()) {
            RCLCPP_INFO(this->get_logger(), "No scan data received yet.");
            return;
        }
        
        std::vector<geometry_msgs::msg::Point> all_points;
        auto start = std::chrono::high_resolution_clock::now();

        for (const auto& [sensor_name, msg] : latest_scans_) {
            if (latest_scans_new_msg_[sensor_name]) {
            

                float angle = msg->angle_min;
                float increment = msg->angle_increment;
                std::string frame_id = msg->header.frame_id;

                geometry_msgs::msg::PointStamped pt_in, pt_out;
                pt_in.header = msg->header;

                for (float r : msg->ranges) {
                    if (!std::isfinite(r)) {
                        angle += increment;
                        continue;
                    }

                    pt_in.point.x = r * std::cos(angle);
                    pt_in.point.y = r * std::sin(angle);
                    pt_in.point.z = 0.0;
                    angle += increment;

                    try {
                        auto tf = tf_buffer_->lookupTransform(robot_base_frame_, frame_id, tf2::TimePointZero);
                        tf2::doTransform(pt_in, pt_out, tf);
                        all_points.push_back(pt_out.point);
                    } catch (const tf2::TransformException &ex) {
                        RCLCPP_WARN(this->get_logger(), "TF error for %s -> %s: %s",
                                    frame_id.c_str(), robot_base_frame_.c_str(), ex.what());
                        continue;
                    }
                }

                latest_scans_new_msg_[sensor_name] = false;
            }
        }


        if (!all_points.empty()) {
            auto end = std::chrono::high_resolution_clock::now();
            double elapsed_ms = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;

            charmie_interfaces::msg::ListOfPoints msg;
            msg.coords = all_points;
            points_publisher_->publish(msg);

            RCLCPP_INFO(this->get_logger(), "Published %lu points in %.2f ms", all_points.size(), elapsed_ms);
        }
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RadarNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}