#include "rclcpp/rclcpp.hpp"
#include <yaml-cpp/yaml.h>

class RadarNode : public rclcpp::Node {
public:
    RadarNode() : Node("radar_node") {
        std::string yaml_path = this->declare_parameter<std::string>("radar_config", "radar_params.yaml");

        try {
            YAML::Node config = YAML::LoadFile(yaml_path);
            YAML::Node radar = config["radar_node"];
            if (!radar) {
                RCLCPP_ERROR(this->get_logger(), "Missing 'radar_node' key in YAML file.");
                return;
            }

            // Get observation_sources string and split it
            std::string sources_str = radar["observation_sources"].as<std::string>();
            std::vector<std::string> sources = split_string(sources_str, ' ');

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
            }

        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load or parse radar_params.yaml: %s", e.what());
        }

        RCLCPP_INFO(this->get_logger(), "RadarNode setup complete.");
    }
    
private:
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

    //void timerCallback()
    //{
    //    RCLCPP_INFO(this->get_logger(), "Hello %d", counter_);
    //    counter_++;
    //}
    //rclcpp::TimerBase::SharedPtr timer_;
    //int counter_;
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