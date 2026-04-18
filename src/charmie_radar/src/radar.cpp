#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "charmie_interfaces/msg/radar_sector.hpp"
#include "charmie_interfaces/msg/radar_data.hpp"
#include "charmie_interfaces/srv/get_min_radar_distance.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <unordered_map>
#include <optional>
#include <sstream>
#include <iomanip>
#include <limits>
#include <chrono>
#include <cmath>
#include <mutex>


class RadarNode : public rclcpp::Node {
public:
    RadarNode() : Node("radar_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {

        // --- Debug params ---
        debug_enabled_ = declare_if_missing_and_get<bool>("debug", false);

        // --- General params ---
        robot_base_frame_ = declare_if_missing_and_get<std::string>("robot_base_frame", "base_footprint");
        robot_radius_     = declare_if_missing_and_get<double>("robot_radius", 0.28);
        double update_frequency = declare_if_missing_and_get<double>("update_frequency", 10.0);

        // --- Radar configuration (nested) ---
        number_of_sectors_ = declare_if_missing_and_get<int>("radar_configuration.number_of_sectors", 0);
        min_radar_angle_   = declare_if_missing_and_get<double>("radar_configuration.min_radar_angle", -M_PI);
        max_radar_angle_   = declare_if_missing_and_get<double>("radar_configuration.max_radar_angle",  M_PI);
        max_radar_range_   = declare_if_missing_and_get<double>("radar_configuration.max_radar_dist", 10.0);

        const double total_angle_range = max_radar_angle_ - min_radar_angle_;
        sector_size_ = (number_of_sectors_ > 0) ? total_angle_range / static_cast<double>(number_of_sectors_) : 0.0;

        // --- Observation sources: list of sensor names ---
        std::vector<std::string> sources = declare_if_missing_and_get<std::vector<std::string>>("observation_sources", std::vector<std::string>{});
        
        if (sources.empty()) {
            RCLCPP_WARN(this->get_logger(), "Parameter 'observation_sources' is empty. No sensors will be used.");
        }

        // Pretty-print the sources list
        std::ostringstream sources_oss;
        for (size_t i = 0; i < sources.size(); ++i) {
            sources_oss << sources[i];
            if (i + 1 < sources.size()) sources_oss << " ";
        }

        std::string successful_sensors_str;
        int number_sensor_skipped = 0;

        RCLCPP_INFO(this->get_logger(), "--- General Configurations: ---");
        RCLCPP_INFO(this->get_logger(), "Robot Base Frame: %s", robot_base_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Update Frequency: %.1f ", update_frequency);
        RCLCPP_INFO(this->get_logger(), "Robot Radius: %.2f m", robot_radius_);
        RCLCPP_INFO(this->get_logger(), "Observation sources: %s", sources_oss.str().c_str());
        
        RCLCPP_INFO(this->get_logger(), "--- Sensor Configurations: ---");

        auto key = [](const std::string& sensor_name, const std::string& suffix) {
            return sensor_name + "." + suffix;
        };
        
        for (const auto& sensor_name : sources) {

            RCLCPP_INFO(this->get_logger(), "\t--- Sensor: %s ---", sensor_name.c_str());
            
            const std::string topic = declare_if_missing_and_get<std::string>(key(sensor_name, "topic"), "");
            const std::string data_type = declare_if_missing_and_get<std::string>(key(sensor_name, "data_type"), "");

            if (topic.empty() || data_type.empty()) {
                RCLCPP_WARN(this->get_logger(),
                            "\tSensor '%s' missing required parameters (topic/data_type). Skipping.",
                            sensor_name.c_str());
                number_sensor_skipped++;
                continue;
            }

            // Early Data Type validation
            if (data_type != "LaserScan" && data_type != "PointCloud2") {
                RCLCPP_WARN(this->get_logger(),
                            "\tUnsupported data type '%s' for sensor '%s'. Skipping.",
                            data_type.c_str(), sensor_name.c_str());
                number_sensor_skipped++;
                continue;
            }

            const double min_obstacle_height = declare_if_missing_and_get<double>(key(sensor_name, "min_obstacle_height"), 0.0);
            const double max_obstacle_height = declare_if_missing_and_get<double>(key(sensor_name, "max_obstacle_height"), 2.0);
            const double min_obstacle_range  = declare_if_missing_and_get<double>(key(sensor_name, "min_obstacle_range"), 0.0);
            const double max_obstacle_range  = declare_if_missing_and_get<double>(key(sensor_name, "max_obstacle_range"), 30.0);
            const double min_obstacle_angle  = declare_if_missing_and_get<double>(key(sensor_name, "min_obstacle_angle"), -M_PI);
            const double max_obstacle_angle  = declare_if_missing_and_get<double>(key(sensor_name, "max_obstacle_angle"),  M_PI);

            SensorLimits limits;
            limits.min_height = min_obstacle_height;
            limits.max_height = max_obstacle_height;
            limits.min_range  = min_obstacle_range;
            limits.max_range  = max_obstacle_range;
            limits.min_angle  = min_obstacle_angle;
            limits.max_angle  = max_obstacle_angle;
            sensor_limits_[sensor_name] = limits;

            RCLCPP_INFO(this->get_logger(), "\tTopic: %s", topic.c_str());
            RCLCPP_INFO(this->get_logger(), "\tData type: %s", data_type.c_str());
            RCLCPP_INFO(this->get_logger(), "\tObstacle height: %.2f to %.2f (m)", min_obstacle_height, max_obstacle_height);
            RCLCPP_INFO(this->get_logger(), "\tObstacle range: %.2f to %.2f (m)", min_obstacle_range, max_obstacle_range);
            RCLCPP_INFO(this->get_logger(), "\tObstacle angle: %.4f to %.4f (rad), %.2f to %.2f (deg)",
                        min_obstacle_angle, max_obstacle_angle,
                        min_obstacle_angle * 180.0 / M_PI, max_obstacle_angle * 180.0 / M_PI);

            // PointCLoud2 specific parameters and publishers (publish filtered cloud, and if debug enabled, discarded cloud)
            if (data_type == "PointCloud2") {
                bool publish_filtered = declare_if_missing_and_get<bool>(key(sensor_name, "publish_filtered"), true);
                sensor_publish_filtered_[sensor_name] = publish_filtered;

                RCLCPP_INFO(this->get_logger(), "\tPublish Filtered: %s", publish_filtered ? "True" : "False");

                if (publish_filtered) {
                    std::string filtered_topic = topic + "/filtered";
                    auto pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(filtered_topic, 10);
                    filtered_cloud_publishers_[sensor_name] = pub;
                    RCLCPP_INFO(this->get_logger(), "\tCreated publisher for filtered data on topic: %s", filtered_topic.c_str());

                    if (debug_enabled_) {
                        std::string discarded_topic = topic + "/discarded";
                        auto discarded_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(discarded_topic, 10);
                        discarded_cloud_publishers_[sensor_name] = discarded_pub;
                        RCLCPP_INFO(this->get_logger(), "\tCreated publisher for discarded points on topic: %s", discarded_topic.c_str());
                    }
                }
            }
                

            // Create subscribers based on data type (after all validations and parameter declarations)
            if (data_type == "LaserScan") {
                auto sub = this->create_subscription<sensor_msgs::msg::LaserScan>(topic, 10,
                    [this, sensor_name](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                        this->laser_callback(msg, sensor_name);
                    });
                laser_subscribers_[sensor_name] = sub;
                RCLCPP_INFO(this->get_logger(), "\tSubscribed to sensor '%s' of type '%s'", sensor_name.c_str(), data_type.c_str());

            } else if (data_type == "PointCloud2") {
                auto sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(topic, 10,
                    [this, sensor_name](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                        this->cloud_callback(msg, sensor_name);
                    });
                cloud_subscribers_[sensor_name] = sub;
                RCLCPP_INFO(this->get_logger(), "\tSubscribed to sensor '%s' of type 'PointCloud2'", sensor_name.c_str());

            } else {
                // Unsupported data type code ... 
                // Ready for diferent data types that can be added in the future
            }
    
            successful_sensors_str += sensor_name + " ";
        }

        RCLCPP_INFO(this->get_logger(), "--- Radar Configurations: ---");
        RCLCPP_INFO(this->get_logger(), "Sensors Used: %s", successful_sensors_str.c_str());
        if (number_sensor_skipped > 0) {
            RCLCPP_WARN(this->get_logger(), "Skipped %d sensors due to missing, wrong or unsupported configurations.", number_sensor_skipped);
        }
        RCLCPP_INFO(this->get_logger(), "Sectors: %d", number_of_sectors_); 
        RCLCPP_INFO(this->get_logger(), "Min angle: %.4f rad, %.2f deg", min_radar_angle_, min_radar_angle_ * 180.0 / M_PI); 
        RCLCPP_INFO(this->get_logger(), "Max angle: %.4f rad, %.2f deg", max_radar_angle_, max_radar_angle_ * 180.0 / M_PI); 
        RCLCPP_INFO(this->get_logger(), "Sector size: %.4f rad, %.2f deg", sector_size_, sector_size_ * 180.0 / M_PI); 
        RCLCPP_INFO(this->get_logger(), "Max range: %.2f m", max_radar_range_); 

        if (max_radar_range_ <= robot_radius_) {
            RCLCPP_FATAL(this->get_logger(),
                "Invalid radar config: max_radar_range_ (%.2f m) <= robot_radius_ (%.2f m).",
                max_radar_range_, robot_radius_);
            throw std::runtime_error("Radar config error: max range must be greater than robot radius.");
        }

        RCLCPP_INFO(this->get_logger(), "--- Finished Sensors Setup ---");
        
        // TF2 setup
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Publisher setup
        radar_pointcloud_baseframe_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("radar/pointcloud_baseframe", 10);
        radar_distances_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("radar/distances", 10);
        radar_distances_normalized_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("radar/distances/normalized", 10);
        radar_custom_pub_ = this->create_publisher<charmie_interfaces::msg::RadarData>("radar/data", 10);

        get_min_srv_ = this->create_service<charmie_interfaces::srv::GetMinRadarDistance>("get_min_radar_distance", std::bind(&RadarNode::getMinRadarDistanceSrv, this, std::placeholders::_1, std::placeholders::_2));

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / update_frequency),
            std::bind(&RadarNode::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "RadarNode Setup Complete.");
    }

private:

    struct SensorLimits {
        double min_height;
        double max_height;
        double min_range;
        double max_range;
        double min_angle;
        double max_angle;
    };

    struct SectorInfo {
        double min_distance = std::numeric_limits<double>::infinity();
        pcl::PointXYZ closest_point;
        bool has_point = false;
    };

    // Helper: declares a parameter with a default value if it does not exist,
    // then returns its value as type T.
    // Useful for dynamic parameters (sensor_name.*) where configs may be missing.
    // Prevents double-declaration errors when parameters are auto-declared from YAML.
    // Not needed for fixed parameters, but still use it for consistency and safety.
    template <typename T>
    T declare_if_missing_and_get(const std::string& name, const T& default_value)
    {
        if (!this->has_parameter(name)) {
            this->declare_parameter<T>(name, default_value);
        }
        return this->get_parameter(name).get_value<T>();
    }

    bool debug_enabled_ = false;
    std::unordered_map<std::string, rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> laser_subscribers_;
    std::unordered_map<std::string, rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> cloud_subscribers_;
    std::unordered_map<std::string, sensor_msgs::msg::LaserScan::SharedPtr> latest_scans_;
    std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr> latest_clouds_;
    std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> filtered_cloud_publishers_;
    std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> discarded_cloud_publishers_; // for debug ...
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr radar_pointcloud_baseframe_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr radar_distances_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr radar_distances_normalized_pub_;
    rclcpp::Publisher<charmie_interfaces::msg::RadarData>::SharedPtr radar_custom_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string robot_base_frame_;
    double robot_radius_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unordered_map<std::string, SensorLimits> sensor_limits_;
    std::unordered_map<std::string, bool> sensor_publish_filtered_;
    std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr> latest_filtered_clouds_baseframe_;
    
    // For Get Minimum Radar Distance Service
    rclcpp::Service<charmie_interfaces::srv::GetMinRadarDistance>::SharedPtr get_min_srv_;
    std::mutex radar_mutex_;
    charmie_interfaces::msg::RadarData latest_radar_msg_;
    bool radar_initialized_ = false;

    // radar_configuration params
    int number_of_sectors_;
    double min_radar_angle_;
    double max_radar_angle_;
    double sector_size_;
    double max_radar_range_;

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg, const std::string &source_name) {
        latest_scans_[source_name] = msg;

        if (debug_enabled_) {
            RCLCPP_INFO(this->get_logger(), "[%s] Received LaserScan with %lu ranges", source_name.c_str(), msg->ranges.size());
        }

        const auto &limits = sensor_limits_[source_name];

        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_filtered_baseframe(new pcl::PointCloud<pcl::PointXYZ>());

        // Discard reason counters
        int discarded_due_to_range = 0;
        int discarded_due_to_height = 0;
        int discarded_due_to_robot_radius = 0;
        int discarded_due_to_angle = 0;

        // Measure filtering time
        auto t_start = std::chrono::high_resolution_clock::now();

        try {
            auto tf = tf_buffer_->lookupTransform(robot_base_frame_, msg->header.frame_id, tf2::TimePointZero);
            tf2::Quaternion quat;
            tf2::fromMsg(tf.transform.rotation, quat);
            tf2::Matrix3x3 rot_matrix(quat);

            float angle = msg->angle_min;
            float increment = msg->angle_increment;

            geometry_msgs::msg::PointStamped pt_in, pt_out;
            pt_in.header = msg->header;

            for (float r : msg->ranges) {
                if (!std::isfinite(r)) {
                    angle += increment;
                    continue;
                }

                // Check range
                if (r < limits.min_range || r > limits.max_range) {
                    discarded_due_to_range++;
                    angle += increment;
                    continue;
                }

                // Generate 3D point (in sensor frame)
                pt_in.point.x = r * std::cos(angle);
                pt_in.point.y = r * std::sin(angle);
                pt_in.point.z = 0.0;
                angle += increment;

                // Rotate original point (for angle filtering)
                tf2::Vector3 original(pt_in.point.x, pt_in.point.y, pt_in.point.z);
                tf2::Vector3 rotated = rot_matrix * original;
                float rot_angle = std::atan2(rotated.y(), rotated.x());

                if (rot_angle < limits.min_angle || rot_angle > limits.max_angle) {
                    discarded_due_to_angle++;
                    continue;
                }

                try {
                    tf2::doTransform(pt_in, pt_out, tf);
                } catch (const tf2::TransformException &ex) {
                    RCLCPP_WARN(this->get_logger(), "[%s] TF error: %s", source_name.c_str(), ex.what());
                    continue;
                }

                double z = pt_out.point.z;
                double xy_dist = std::hypot(pt_out.point.x, pt_out.point.y);

                if (z < limits.min_height || z > limits.max_height) {
                    discarded_due_to_height++;
                    continue;
                }

                if (xy_dist < robot_radius_) {
                    discarded_due_to_robot_radius++;
                    continue;
                }

                // Valid point → Add to cloud (in base frame)
                pcl::PointXYZ pt_base;
                pt_base.x = pt_out.point.x;
                pt_base.y = pt_out.point.y;
                pt_base.z = pt_out.point.z;
                pcl_filtered_baseframe->points.push_back(pt_base);
            }

        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "[%s] TF error during processing: %s", source_name.c_str(), ex.what());
            return;
        }

        // Convert to PointCloud2 and store
        pcl_filtered_baseframe->width = pcl_filtered_baseframe->points.size();
        pcl_filtered_baseframe->height = 1;
        pcl_filtered_baseframe->is_dense = true;

        auto msg_baseframe = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*pcl_filtered_baseframe, *msg_baseframe);
        msg_baseframe->header.stamp = msg->header.stamp;
        msg_baseframe->header.frame_id = robot_base_frame_;
        msg_baseframe->is_dense = true;

        latest_filtered_clouds_baseframe_[source_name] = msg_baseframe;
        
        if (debug_enabled_) {
            auto t_end = std::chrono::high_resolution_clock::now();
            double elapsed_ms = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count() / 1000.0;
            RCLCPP_INFO(this->get_logger(), "[%s] Filtering took %.2f ms.", source_name.c_str(), elapsed_ms);
        }

        // Summary logging
        const size_t total_points = msg->ranges.size();
        const size_t total_discarded = discarded_due_to_range + discarded_due_to_height +
                                    discarded_due_to_robot_radius + discarded_due_to_angle;
        const size_t valid = pcl_filtered_baseframe->points.size();


        if (debug_enabled_ ) {
            
            RCLCPP_INFO(this->get_logger(),
                "[%s] LaserScan total=%lu valid=%lu, discarded=%lu (range=%d, height=%d, radius=%d, angle=%d)",
                source_name.c_str(),
                total_points,
                valid,
                total_discarded,
                discarded_due_to_range,
                discarded_due_to_height,
                discarded_due_to_robot_radius,
                discarded_due_to_angle
            );
        }
        else { // Simplified

            RCLCPP_DEBUG(this->get_logger(),
                "[%s] Points: %lu/%lu (range=%d, height=%d, robot=%d, back=%d)",
                source_name.c_str(),
                valid,
                total_points,
                discarded_due_to_range,
                discarded_due_to_height,
                discarded_due_to_robot_radius,
                discarded_due_to_angle
            );
        }

    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string &source_name) {
        latest_clouds_[source_name] = msg;

        if (debug_enabled_) {
            RCLCPP_INFO(this->get_logger(), "[%s] Received PointCloud2 with width %u points.", source_name.c_str(), msg->width);
        }

        const auto& limits = sensor_limits_[source_name];

        // Convert to PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_filtered_baseframe(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_filtered(new pcl::PointCloud<pcl::PointXYZ>());    
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_discarded(new pcl::PointCloud<pcl::PointXYZ>()); // for debug ...
        pcl::fromROSMsg(*msg, *pcl_input);
        
        int discarded_due_to_range = 0;
        int discarded_due_to_height = 0;
        int discarded_due_to_robot_radius = 0;
        int discarded_due_to_angle = 0;

        // Measure filtering time
        auto t_start = std::chrono::high_resolution_clock::now();

        try {
            // Lookup the transform only once (cloud_frame → robot_base_frame)
            geometry_msgs::msg::TransformStamped transformStamped = tf_buffer_->lookupTransform(
                robot_base_frame_, msg->header.frame_id, tf2::TimePointZero);

            // Convert just the rotation to a rotation matrix for min and max angle checks
            // This way the points referencial comes from the lidar tf, but the min and max angles are from the robot base frame
            tf2::Quaternion quat;
            tf2::fromMsg(transformStamped.transform.rotation, quat);
            tf2::Matrix3x3 rot_matrix(quat);

            geometry_msgs::msg::PointStamped pt_in, pt_out;
            pt_in.header = msg->header;

            // Checks whether the point is valid
            for (const auto& pt : pcl_input->points) {
                if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
                    continue;

                // Checks min and max range
                float range_xy = std::sqrt(pt.x * pt.x + pt.y * pt.y);
                if (range_xy < limits.min_range || range_xy > limits.max_range) {
                    discarded_due_to_range++;
                    pcl_discarded->points.push_back(pt);
                    continue;
                }

                // Transform this point to robot base frame
                pt_in.point.x = pt.x;
                pt_in.point.y = pt.y;
                pt_in.point.z = pt.z;
                tf2::doTransform(pt_in, pt_out, transformStamped);

                double z_robot = pt_out.point.z;
                double distance_xy = std::hypot(pt_out.point.x, pt_out.point.y);  // Equivalent to sqrt(x² + y²)

                // Check height limits
                if (z_robot >= limits.min_height && z_robot <= limits.max_height) {
                    if (distance_xy >= robot_radius_) {
                        
                        // Check angle limits
                        tf2::Vector3 original_point(pt.x, pt.y, pt.z);
                        tf2::Vector3 rotated = rot_matrix * original_point;

                        float angle = std::atan2(rotated.y(), rotated.x());
                        if (angle < limits.min_angle || angle > limits.max_angle) {
                            discarded_due_to_angle++;
                            pcl_discarded->points.push_back(pt);
                        }
                        else {
                            // Point is valid, add to filtered cloud
                            pcl_filtered->points.push_back(pt);

                            pcl::PointXYZ transformed_pt;
                            transformed_pt.x = pt_out.point.x;
                            transformed_pt.y = pt_out.point.y;
                            transformed_pt.z = pt_out.point.z;
                            pcl_filtered_baseframe->points.push_back(transformed_pt);
                        }

                    } else {
                        discarded_due_to_robot_radius++;
                        pcl_discarded->points.push_back(pt);  // Inside robot, discard
                    }
                } else {
                    discarded_due_to_height++;
                    pcl_discarded->points.push_back(pt);  // Height outside range
                }
            }
        }
        catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "[%s] TF error during procesing: %s", source_name.c_str(), ex.what());
            return;  // Skip publishing this frame if TF fails
        }

        // Creates msg headers 
        pcl_filtered->width = pcl_filtered->points.size();
        pcl_filtered->height = 1;
        pcl_filtered->is_dense = true;

        if (debug_enabled_) {
            pcl_discarded->width = pcl_discarded->points.size();
            pcl_discarded->height = 1;
            pcl_discarded->is_dense = true;
        }

        // If publish_filtered is true for this sensor, publish filtered PointCloud2
        if (sensor_publish_filtered_[source_name] && filtered_cloud_publishers_.count(source_name)) {
            
            auto filtered_pub = filtered_cloud_publishers_[source_name];
            sensor_msgs::msg::PointCloud2 ros_filtered;
            pcl::toROSMsg(*pcl_filtered, ros_filtered);
            ros_filtered.header = msg->header;
            filtered_pub->publish(ros_filtered);

            if (debug_enabled_) {
                auto discarded_pub = discarded_cloud_publishers_[source_name];
                sensor_msgs::msg::PointCloud2 ros_discarded;
                pcl::toROSMsg(*pcl_discarded, ros_discarded);
                ros_discarded.header = msg->header;
                discarded_pub->publish(ros_discarded);
            }
        }

        auto msg_baseframe = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*pcl_filtered_baseframe, *msg_baseframe);
        msg_baseframe->header.stamp = msg->header.stamp;
        msg_baseframe->header.frame_id = robot_base_frame_;
        msg_baseframe->is_dense = true;

        latest_filtered_clouds_baseframe_[source_name] = msg_baseframe;
        
        if (debug_enabled_) {
            auto t_end = std::chrono::high_resolution_clock::now();
            double elapsed_ms = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count() / 1000.0;
            RCLCPP_INFO(this->get_logger(), "[%s] Filtering took %.2f ms", source_name.c_str(), elapsed_ms);
        }


        if (debug_enabled_) {
            RCLCPP_INFO(this->get_logger(), "[%s] Published filtered cloud with %lu points (from %lu)", 
                        source_name.c_str(), pcl_filtered->points.size(), pcl_input->points.size());
            
            RCLCPP_INFO(this->get_logger(),
                "[%s] Published discarded cloud with %lu points (from %lu). Breakdown: range=%d, height=%d, radius=%d, angle=%d",
                source_name.c_str(),
                pcl_discarded->points.size(),
                pcl_input->points.size(),
                discarded_due_to_range,
                discarded_due_to_height,
                discarded_due_to_robot_radius,
                discarded_due_to_angle
            );
        }
        else { // Simplified

            RCLCPP_DEBUG(this->get_logger(),
                "[%s] Points: %lu/%lu (range=%d, height=%d, robot=%d, back=%d)",
                source_name.c_str(),
                pcl_filtered->points.size(),
                pcl_input->points.size(),
                discarded_due_to_range,
                discarded_due_to_height,
                discarded_due_to_robot_radius,
                discarded_due_to_angle
            );
        }

    }

    void timer_callback() {

        auto t_start = std::chrono::high_resolution_clock::now();

        if (latest_scans_.empty() && latest_clouds_.empty()) {
            RCLCPP_INFO(this->get_logger(), "No scan data received yet.");
            return;
        }
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        rclcpp::Time latest_stamp = this->get_clock()->now();


        for (const auto& [sensor_name, cloud_ptr] : latest_filtered_clouds_baseframe_) {
            if (cloud_ptr && cloud_ptr->width > 0) {

                pcl::PointCloud<pcl::PointXYZ> pcl_temp;
                pcl::fromROSMsg(*cloud_ptr, pcl_temp);
                *merged_cloud += pcl_temp;  // Concatenate

                rclcpp::Time stamp(cloud_ptr->header.stamp);
                if (stamp > latest_stamp) {
                    latest_stamp = stamp;
                }

                if (debug_enabled_) {
                    RCLCPP_INFO(this->get_logger(), "[radar][%s] Included cloud with %u points.", sensor_name.c_str(), cloud_ptr->width);
                }

            }
        }

        auto t_start2 = std::chrono::high_resolution_clock::now();
        if (merged_cloud->empty()) {
            if (debug_enabled_) {
                RCLCPP_INFO(this->get_logger(), "[radar] No points to publish after fusing clouds.");
            }
            return;
        }
        else {
            if (debug_enabled_) {
                RCLCPP_INFO(this->get_logger(), "[radar] Merged cloud has %lu points.", merged_cloud->points.size());
            }
            
            std::vector<SectorInfo> sectors(number_of_sectors_);

            // Iterate over merged cloud points
            for (const auto& pt : merged_cloud->points) {
                if (!std::isfinite(pt.x) || !std::isfinite(pt.y))
                    continue;

                double angle = std::atan2(pt.y, pt.x);
                double distance_xy = std::hypot(pt.x, pt.y);

                // Only consider points within angular range
                if (angle < min_radar_angle_ || angle > max_radar_angle_)
                    continue;

                // Determine sector index
                int sector_idx = static_cast<int>((angle - min_radar_angle_) / sector_size_);
                if (sector_idx < 0 || sector_idx >= number_of_sectors_)
                    continue;

                // Check if this point is closer than the current closest
                if (distance_xy < sectors[sector_idx].min_distance) {
                    sectors[sector_idx].min_distance = distance_xy;
                    sectors[sector_idx].closest_point = pt;
                    sectors[sector_idx].has_point = true;
                }
            }


            // Output results
            for (int i = 0; i < number_of_sectors_; ++i) {
                double start_angle_deg = (min_radar_angle_ + sector_size_ *       i) * 180.0 / M_PI;
                double end_angle_deg   = (min_radar_angle_ + sector_size_ * (i + 1)) * 180.0 / M_PI;

                if (sectors[i].has_point) {
                    RCLCPP_DEBUG(this->get_logger(),
                        "[radar] Sector %d [%.1f°, %.1f°]: Point (%.2f, %.2f, %.2f) d=%.2f m",
                        i, start_angle_deg, end_angle_deg,
                        sectors[i].closest_point.x,
                        sectors[i].closest_point.y,
                        sectors[i].closest_point.z,
                        sectors[i].min_distance
                    );
                } else {
                    RCLCPP_DEBUG(this->get_logger(),
                        "[radar] Sector %d [%.1f°, %.1f°]: No Points", 
                        i, start_angle_deg, end_angle_deg
                    );
                }
            }


            // RADAR DISTANCES
            std_msgs::msg::Float32MultiArray distances_msg;
            std_msgs::msg::Float32MultiArray distances_normalized_msg;

            distances_msg.data.resize(number_of_sectors_);
            distances_normalized_msg.data.resize(number_of_sectors_);
            
            // Define max adjusted range once
            const double max_adjusted_range = max_radar_range_ - robot_radius_;

            for (int i = 0; i < number_of_sectors_; ++i) {
                double raw_distance;

                if (sectors[i].has_point) {
                    raw_distance = sectors[i].min_distance;
                } else {
                    raw_distance = max_radar_range_;
                }

                // Adjust for robot radius (i.e., from robot *edge*, not center)
                double adjusted = std::max(0.0, raw_distance - robot_radius_);
                adjusted = std::min(adjusted, max_adjusted_range);  // Clamp

                // Normalized value ∈ [0, 1]
                double normalized = adjusted / max_adjusted_range;

                // Fill messages
                distances_msg.data[i] = static_cast<float>(adjusted);         // meters from robot edge
                distances_normalized_msg.data[i] = static_cast<float>(normalized);
            }

            radar_distances_pub_->publish(distances_msg);
            radar_distances_normalized_pub_->publish(distances_normalized_msg);


            if (debug_enabled_) {
                // PRINTS RADAR DISTANCES, IN METERS AND NORMALIZED
                // Build string for raw distances
                std::ostringstream raw_stream;
                raw_stream << "Radar raw: ";
                for (size_t i = 0; i < distances_msg.data.size(); ++i) {
                    raw_stream << std::fixed << std::setprecision(2) << distances_msg.data[i];
                    if (i < distances_msg.data.size() - 1) {
                        raw_stream << " ";
                    }
                }

                // Build string for normalized distances
                std::ostringstream norm_stream;
                norm_stream << "Radar norm: ";
                for (size_t i = 0; i < distances_normalized_msg.data.size(); ++i) {
                    norm_stream << std::fixed << std::setprecision(2) << distances_normalized_msg.data[i];
                    if (i < distances_normalized_msg.data.size() - 1) {
                        norm_stream << " ";
                    }
                }

                // Print both in INFO logs
                RCLCPP_DEBUG(this->get_logger(), "%s", raw_stream.str().c_str());
                RCLCPP_DEBUG(this->get_logger(), "%s", norm_stream.str().c_str());
            }

            // RADAR DATA
            charmie_interfaces::msg::RadarData radar_msg;
            radar_msg.header.stamp = latest_stamp;
            radar_msg.header.frame_id = robot_base_frame_;
            radar_msg.number_of_sectors = number_of_sectors_;
            radar_msg.sector_ang_range = sector_size_;

            for (int i = 0; i < number_of_sectors_; ++i) {
                charmie_interfaces::msg::RadarSector sector;
                
                sector.start_angle = min_radar_angle_ + i * sector_size_;
                sector.end_angle   = sector.start_angle + sector_size_;
                sector.has_point   = sectors[i].has_point;

                if (sectors[i].has_point) {
                    sector.min_distance = sectors[i].min_distance;
                    sector.point.x = sectors[i].closest_point.x;
                    sector.point.y = sectors[i].closest_point.y;
                    sector.point.z = sectors[i].closest_point.z;
                } else {
                    // If no point: use NaN for distance, zero point
                    sector.min_distance = std::numeric_limits<double>::quiet_NaN();
                    sector.point.x = 0.0;
                    sector.point.y = 0.0;
                    sector.point.z = 0.0;
                }

                radar_msg.sectors.push_back(sector);
            }
            
            radar_custom_pub_->publish(radar_msg);

            std::lock_guard<std::mutex> lock(radar_mutex_);
            latest_radar_msg_ = radar_msg;
            radar_initialized_ = true;

            if (debug_enabled_) {
                // Debug print of full RadarData message
                RCLCPP_INFO(this->get_logger(), "[RadarData] frame_id: %s, sectors: %d",
                            radar_msg.header.frame_id.c_str(),
                            radar_msg.number_of_sectors);

                for (size_t i = 0; i < radar_msg.sectors.size(); ++i) {
                    const auto& sector = radar_msg.sectors[i];
                    
                    if (sector.has_point) {
                        RCLCPP_INFO(this->get_logger(),
                            "  Sector %02zu: [%.4f, %.4f]rad [%.2f, %.2f]deg dist=%.2f m → (%.2f, %.2f, %.2f)",
                            i,
                            sector.start_angle,
                            sector.end_angle,
                            sector.start_angle * 180.0 / M_PI,
                            sector.end_angle   * 180.0 / M_PI,
                            sector.min_distance,
                            sector.point.x,
                            sector.point.y,
                            sector.point.z
                        );
                    } else {
                        RCLCPP_INFO(this->get_logger(),
                            "  Sector %02zu: [%.2f°, %.2f°] → No points",
                            i,
                            sector.start_angle * 180.0 / M_PI,
                            sector.end_angle   * 180.0 / M_PI
                        );
                    }
                }
            }
                        
        }

        sensor_msgs::msg::PointCloud2 ros_merged;
        pcl::toROSMsg(*merged_cloud, ros_merged);
        ros_merged.header.frame_id = robot_base_frame_;
        ros_merged.header.stamp = latest_stamp;
        ros_merged.is_dense = true;

        radar_pointcloud_baseframe_publisher_->publish(ros_merged);
        RCLCPP_DEBUG(this->get_logger(), "[radar] Published merged cloud with %lu points.", merged_cloud->points.size());

        auto t_end = std::chrono::high_resolution_clock::now();
        double total_elapsed_ms    = std::chrono::duration_cast<std::chrono::microseconds>(t_end    - t_start ).count() / 1000.0;
        double merging_elapsed_ms  = std::chrono::duration_cast<std::chrono::microseconds>(t_start2 - t_start ).count() / 1000.0;
        double checking_elapsed_ms = std::chrono::duration_cast<std::chrono::microseconds>(t_end    - t_start2).count() / 1000.0;
        RCLCPP_DEBUG(this->get_logger(), "[radar] Total time: %.2f ms, Merging time %.2f ms, Checking time %.2f ms", total_elapsed_ms, merging_elapsed_ms, checking_elapsed_ms);
        
    }

    void getMinRadarDistanceSrv( 
    const std::shared_ptr<charmie_interfaces::srv::GetMinRadarDistance::Request> req,
    std::shared_ptr<charmie_interfaces::srv::GetMinRadarDistance::Response> res)
    {
        // Defaults
        res->success = false;
        res->message = "";
        res->min_radar_distance_to_robot_edge = 0.0f;

        // Snapshot current radar data
        charmie_interfaces::msg::RadarData radar;
        {
            std::lock_guard<std::mutex> lock(radar_mutex_);
            if (!radar_initialized_) {
                res->message = "Radar not initialized";
                RCLCPP_WARN(this->get_logger(), "%s", res->message.c_str());
                return;
            }
            radar = latest_radar_msg_;
        }

        // Validate inputs (match your Python constraints)
        const double direction_deg = static_cast<double>(req->direction);
        const double window_deg    = static_cast<double>(req->ang_obstacle_check);

        if (direction_deg < -100.0 || direction_deg > 100.0 || !(window_deg > 0.0 && window_deg <= 360.0)) {
            res->message = "Wrong parameter definition";
            RCLCPP_WARN(this->get_logger(), "%s (direction=%.2f deg, window=%.2f deg)",
                        res->message.c_str(), direction_deg, window_deg);
            return;
        }

        // Convert to radians
        const double dir_rad = direction_deg * M_PI / 180.0;
        const double halfwin = (window_deg * M_PI / 180.0) / 2.0;

        // Scan sectors where BOTH start and end lie inside the requested window
        std::optional<double> min_distance_center;
        int used_count = 0;

        for (const auto& s : radar.sectors) {
            // shift by -dir and wrap using atan2(sin,cos) (no helper needed)
            const double dStart = std::atan2(std::sin(static_cast<double>(s.start_angle) - dir_rad),
                                            std::cos(static_cast<double>(s.start_angle) - dir_rad));
            const double dEnd   = std::atan2(std::sin(static_cast<double>(s.end_angle)   - dir_rad),
                                            std::cos(static_cast<double>(s.end_angle)   - dir_rad));

            if (dStart >= -halfwin && dStart <= halfwin &&
                dEnd   >= -halfwin && dEnd   <= halfwin)
            {
                ++used_count;
                if (s.has_point && std::isfinite(s.min_distance)) {
                    const double d = static_cast<double>(s.min_distance);
                    if (!min_distance_center.has_value() || d < *min_distance_center) {
                        min_distance_center = d;
                    }
                }
            }
        }

        if (!min_distance_center.has_value()) {
            res->success = false;
            res->message = "No obstacles detected in the selected direction";
            RCLCPP_WARN(this->get_logger(), "%s (used_sectors=%d)", res->message.c_str(), used_count);
            return;
        }

        // Return distance from robot EDGE (>= 0)
        double edge = *min_distance_center - robot_radius_;
        if (edge < 0.0) edge = 0.0;

        res->success = true;
        res->message = "";
        res->min_radar_distance_to_robot_edge = static_cast<float>(edge);

        if (debug_enabled_) {
            RCLCPP_INFO(this->get_logger(),
                "get_min_radar_distance: dir=%.1f deg, win=%.1f deg → center=%.2f m, edge=%.2f m (used=%d)",
                direction_deg, window_deg, *min_distance_center, edge, used_count);
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