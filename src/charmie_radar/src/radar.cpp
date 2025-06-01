#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
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

            std::string robot_base_frame = radar["robot_base_frame"] ? radar["robot_base_frame"].as<std::string>() : "N/A";
            double update_frequency = radar["update_frequency"] ? radar["update_frequency"].as<double>() : 10.0;

            // Get observation_sources string and split it
            std::string sources_str = radar["observation_sources"].as<std::string>();
            std::vector<std::string> sources = split_string(sources_str, ' ');

            RCLCPP_INFO(this->get_logger(), "Robot Base Frame: %s", robot_base_frame.c_str());
            RCLCPP_INFO(this->get_logger(), "Update Frequency: %.1f ", update_frequency);
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

                RCLCPP_INFO(this->get_logger(), "--- Sensor: %s ---", sensor_name.c_str());
                RCLCPP_INFO(this->get_logger(), "Topic: %s", topic.c_str());
                RCLCPP_INFO(this->get_logger(), "Data type: %s", data_type.c_str());
                RCLCPP_INFO(this->get_logger(), "Obstacle height: %.2f to %.2f", min_obstacle_height, max_obstacle_height);
                RCLCPP_INFO(this->get_logger(), "Obstacle range: %.2f to %.2f", obstacle_min_range, obstacle_max_range);

                latest_scans_new_msg_[sensor_name] = false;

                if (data_type == "LaserScan") {
                    auto sub = this->create_subscription<sensor_msgs::msg::LaserScan>(topic, 10, 
                        [this, sensor_name](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                            this->laser_callback(msg, sensor_name);
                        });
                    laser_subscribers_[sensor_name] = sub;
                    RCLCPP_INFO(this->get_logger(), "Subscribed to sensor '%s' of message type '%s'", sensor_name.c_str(), data_type.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(), "Unsupported data type '%s' for sensor '%s'", data_type.c_str(), sensor_name.c_str());
                }

            }

            timer_ = this->create_wall_timer(
                std::chrono::duration<double>(1.0 / update_frequency),
                std::bind(&RadarNode::timer_callback, this)
            );

        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load or parse radar_params.yaml: %s", e.what());
        }

        RCLCPP_INFO(this->get_logger(), "RadarNode setup complete.");
    }

private:

    std::unordered_map<std::string, rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> laser_subscribers_;
    std::unordered_map<std::string, sensor_msgs::msg::LaserScan::SharedPtr> latest_scans_;
    std::unordered_map<std::string, bool> latest_scans_new_msg_;
    rclcpp::TimerBase::SharedPtr timer_;

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

    void timer_callback() {
        if (latest_scans_.empty()) {
            RCLCPP_INFO(this->get_logger(), "No scan data received yet.");
            return;
        }

        for (const auto& [sensor_name, msg] : latest_scans_) {
            if (latest_scans_new_msg_[sensor_name]) {
            
                auto start = std::chrono::high_resolution_clock::now();

                //RCLCPP_INFO(this->get_logger(), "  %s - timestamp: %u.%u, range count: %lu",
                //            sensor_name.c_str(),
                //            msg->header.stamp.sec,
                //            msg->header.stamp.nanosec,
                //            msg->ranges.size());

                float sum = 0.0f;
                int count = 0;
                float avg = 0.0f;

                for (float r : msg->ranges) {
                    if (std::isfinite(r)) {
                        sum += r;
                        count++;
                    }
                }

                if (count > 0) {
                    avg = sum / count;
                }

                auto end = std::chrono::high_resolution_clock::now();
                auto duration_ms = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;            

                RCLCPP_INFO(this->get_logger(), "  %s - valid ranges: %d, sum: %.2f, avg: %.2f, time: %.4f",
                    sensor_name.c_str(), count, sum, avg, duration_ms);

                latest_scans_new_msg_[sensor_name] = false;
            }
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