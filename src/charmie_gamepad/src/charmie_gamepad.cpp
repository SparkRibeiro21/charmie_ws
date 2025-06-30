#include <cinttypes>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <cmath>

#include "rcutils/logging_macros.h"

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "charmie_interfaces/msg/gamepad_controller.hpp"

#include "charmie_gamepad/charmie_gamepad.hpp"
#include "charmie_gamepad/sdl_utils.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>

#define ROS_INFO_NAMED RCUTILS_LOG_INFO_NAMED
#define ROS_INFO_COND_NAMED RCUTILS_LOG_INFO_EXPRESSION_NAMED

namespace charmie_gamepad
{

template<typename T> T get_param_from_yaml(const YAML::Node & config, const std::string & key, const T & default_value);

struct CHARMIEGamepad::Impl
{
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);
  void sendCmdVelMsg(const std::string & which_map);
  void fillCmdVelMsg(const std::string & which_map, geometry_msgs::msg::Twist * cmd_vel_msg);
  std::string get_yaml_path_for_controller(const std::string & controller_name);

  bool get_button(const sensor_msgs::msg::Joy::SharedPtr & msg, int index)
  {
    return index >= 0 && 
           static_cast<int>(msg->buttons.size()) > index &&
           msg->buttons[index];
  }

  bool is_axis_pressed(const sensor_msgs::msg::Joy::SharedPtr & msg, int index, float threshold)
  {
    return index >= 0 &&
          static_cast<int>(msg->axes.size()) > index &&
          std::abs(msg->axes[index] - threshold) < 0.5;
  }

  float get_axis_position(const sensor_msgs::msg::Joy::SharedPtr & msg, int index,
    float raw_min = -1.0f,
    float raw_max = 1.0f,
    float scaled_min = -1.0f,
    float scaled_max = 1.0f)
  {
    if (index < 0 || static_cast<int>(msg->axes.size()) <= index) {
      return 0.0f;
    }

    float raw = msg->axes[index];
    raw = std::clamp(raw, std::min(raw_min, raw_max), std::max(raw_min, raw_max));

    bool is_inverted = raw_min > raw_max;
    if (is_inverted) {
      std::swap(raw_min, raw_max);
      std::swap(scaled_min, scaled_max);
    }

    float scaled = (raw - raw_min) / (raw_max - raw_min);
    return scaled_min + scaled * (scaled_max - scaled_min);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::Publisher<charmie_interfaces::msg::GamepadController>::SharedPtr gamepad_controller_publisher;

  rclcpp::Clock::SharedPtr clock;
  rclcpp::TimerBase::SharedPtr safety_timer;
  rclcpp::Time last_joy_msg_time;

  // In Impl struct
  // The follwing variables are used to map joystick buttons and axes to their respective indices.
  int a_button;
  int b_button;
  int x_button;
  int y_button;
  int left_bumper_button;
  int right_bumper_button;
  int view_button;
  int menu_button;
  int logo_button;
  int share_button;
  int left_thumbstick_button;
  int right_thumbstick_button;
  int left_trigger_button;
  int right_trigger_button;
  int dpad_up_button;
  int dpad_down_button;
  int dpad_left_button;
  int dpad_right_button;

  int left_thumbstick_x_axis;
  int left_thumbstick_y_axis;
  int right_thumbstick_x_axis;
  int right_thumbstick_y_axis;
  int left_trigger_axis;
  int right_trigger_axis;
  int dpad_x_axis;
  int dpad_y_axis;

  bool dpad_axis;
  bool triggers_axis;
  bool triggers_reversed;

  // Button pressed readings
  bool a_button_state = false;
  bool b_button_state = false;
  bool x_button_state = false;
  bool y_button_state = false;
  bool left_bumper_button_state = false;
  bool right_bumper_button_state = false;
  bool view_button_state = false;
  bool menu_button_state = false;
  bool logo_button_state = false;
  bool share_button_state = false;
  bool left_thumbstick_button_state = false;
  bool right_thumbstick_button_state = false;
  bool dpad_up_button_state = false;
  bool dpad_down_button_state = false;
  bool dpad_left_button_state = false;
  bool dpad_right_button_state = false;
  bool left_trigger_button_state = false;
  bool right_trigger_button_state = false;

  float left_thumbstick_x_axis_state = 0.0;
  float left_thumbstick_y_axis_state = 0.0;
  float right_thumbstick_x_axis_state = 0.0;
  float right_thumbstick_y_axis_state = 0.0;
  float left_trigger_axis_state = 0.0;
  float right_trigger_axis_state = 0.0;

  bool timeout = false;  // true if the controller is not responding
};


CHARMIEGamepad::CHARMIEGamepad(const rclcpp::NodeOptions & options)
: rclcpp::Node("charmie_gamepad_node", options)
{
  pimpl_ = new Impl;

  pimpl_->clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

  pimpl_->joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", rclcpp::QoS(10),
    std::bind(&CHARMIEGamepad::Impl::joyCallback, this->pimpl_, std::placeholders::_1));
  
  pimpl_->gamepad_controller_publisher = this->create_publisher<charmie_interfaces::msg::GamepadController>(
    "gamepad_controller", 10);

  std::string controller_name = sdl_utils::get_controller_name();
  RCLCPP_INFO(this->get_logger(), "Detected controller: %s", controller_name.c_str());

  std::string yaml_path = pimpl_->get_yaml_path_for_controller(controller_name);
  YAML::Node config = YAML::LoadFile(yaml_path);

  using namespace std::chrono_literals;
  pimpl_->safety_timer = this->create_wall_timer(
    100ms,
    [this]() {
      rclcpp::Time now = pimpl_->clock->now();
      if ((now - pimpl_->last_joy_msg_time).seconds() > 0.5) {
        pimpl_->timeout = true;
        // RCLCPP_WARN(this->get_logger(), "No joystick msg for 0.5s, TIMEOUT!");
        charmie_interfaces::msg::GamepadController controller_msg;
        controller_msg.timeout = pimpl_->timeout;
        pimpl_->gamepad_controller_publisher->publish(controller_msg);
        
      }
      else {
        pimpl_->timeout = false;
      }
    });
 
  // Declare button parameters
  pimpl_->a_button =                  get_param_from_yaml(config, "a_button", 0);
  pimpl_->b_button =                  get_param_from_yaml(config, "b_button", 1);
  pimpl_->x_button =                  get_param_from_yaml(config, "x_button", 2);
  pimpl_->y_button =                  get_param_from_yaml(config, "y_button", 3);
  pimpl_->left_bumper_button =        get_param_from_yaml(config, "left_bumper_button", 4);
  pimpl_->right_bumper_button =       get_param_from_yaml(config, "right_bumper_button", 5);
  pimpl_->view_button =               get_param_from_yaml(config, "view_button", 6);
  pimpl_->menu_button =               get_param_from_yaml(config, "menu_button", 7);
  pimpl_->logo_button =               get_param_from_yaml(config, "logo_button", 8);
  pimpl_->share_button =              get_param_from_yaml(config, "share_button", 11);
  pimpl_->left_thumbstick_button =    get_param_from_yaml(config, "left_thumbstick_button", 9);
  pimpl_->right_thumbstick_button =   get_param_from_yaml(config, "right_thumbstick_button", 10);
  pimpl_->left_trigger_button =       get_param_from_yaml(config, "left_trigger_button", 12);
  pimpl_->right_trigger_button =      get_param_from_yaml(config, "right_trigger_button", 13);
  pimpl_->dpad_up_button =            get_param_from_yaml(config, "dpad_up_button", 14);
  pimpl_->dpad_down_button =          get_param_from_yaml(config, "dpad_down_button", 15);
  pimpl_->dpad_left_button =          get_param_from_yaml(config, "dpad_left_button", 16);
  pimpl_->dpad_right_button =         get_param_from_yaml(config, "dpad_right_button", 17);

  // Declare axis parameters
  pimpl_->left_thumbstick_x_axis =    get_param_from_yaml(config, "left_thumbstick_x_axis", 0);
  pimpl_->left_thumbstick_y_axis =    get_param_from_yaml(config, "left_thumbstick_y_axis", 1);
  pimpl_->right_thumbstick_x_axis =   get_param_from_yaml(config, "right_thumbstick_x_axis", 2);
  pimpl_->right_thumbstick_y_axis =   get_param_from_yaml(config, "right_thumbstick_y_axis", 3);
  pimpl_->left_trigger_axis =         get_param_from_yaml(config, "left_trigger_axis", 4);
  pimpl_->right_trigger_axis =        get_param_from_yaml(config, "right_trigger_axis", 5);
  pimpl_->dpad_x_axis =               get_param_from_yaml(config, "dpad_x_axis", 6);
  pimpl_->dpad_y_axis =               get_param_from_yaml(config, "dpad_y_axis", 7);

  // Declare behavioral flags
  pimpl_->dpad_axis =                 get_param_from_yaml(config, "dpad_axis", true);
  pimpl_->triggers_axis =             get_param_from_yaml(config, "triggers_axis", true);
  pimpl_->triggers_reversed =         get_param_from_yaml(config, "triggers_reversed", false);
}

CHARMIEGamepad::~CHARMIEGamepad()
{
  delete pimpl_;
}

std::string CHARMIEGamepad::Impl::get_yaml_path_for_controller(const std::string &controller_name)
{
  std::string pkg_share = ament_index_cpp::get_package_share_directory("charmie_gamepad");
  std::filesystem::path config_dir = pkg_share + "/config";

  if (controller_name.find("Xbox") != std::string::npos) {
    RCLCPP_INFO(rclcpp::get_logger("CHARMIEGamepad"), "Xbox YAML config file selected");
    return (config_dir / "xbox.config.yaml").string();
  } else if (controller_name.find("Logitech") != std::string::npos) {
    RCLCPP_INFO(rclcpp::get_logger("CHARMIEGamepad"), "Logitech YAML config file selected");
    return (config_dir / "logitech.config.yaml").string();
  } else if (controller_name.find("Sony Interactive Entertainment Wireless Controller") != std::string::npos) {
    RCLCPP_INFO(rclcpp::get_logger("CHARMIEGamepad"), "PS4 YAML config file selected");
    return (config_dir / "ps4.config.yaml").string();
  } else {
    RCLCPP_INFO(rclcpp::get_logger("CHARMIEGamepad"), "Default YAML config file selected");
    return (config_dir / "ps4.config.yaml").string();  // Fallback
  }
}

void CHARMIEGamepad::Impl::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
{
  
  last_joy_msg_time = clock->now();
  
  // Letter Buttons (Right Side)
  a_button_state = get_button(joy_msg, a_button);
  b_button_state = get_button(joy_msg, b_button);
  x_button_state = get_button(joy_msg, x_button);
  y_button_state = get_button(joy_msg, y_button);

  // D-Pad Buttons
  if (dpad_axis) {
    // D-pad buttons are treated as axes
    dpad_up_button_state    = is_axis_pressed(joy_msg, dpad_y_axis,  1.0);
    dpad_down_button_state  = is_axis_pressed(joy_msg, dpad_y_axis, -1.0); 
    dpad_left_button_state  = is_axis_pressed(joy_msg, dpad_x_axis,  1.0);
    dpad_right_button_state = is_axis_pressed(joy_msg, dpad_x_axis, -1.0);
  } else {
    // D-pad buttons are treated as buttons
    dpad_up_button_state    = get_button(joy_msg, dpad_up_button);
    dpad_down_button_state  = get_button(joy_msg, dpad_down_button);
    dpad_left_button_state  = get_button(joy_msg, dpad_left_button);
    dpad_right_button_state = get_button(joy_msg, dpad_right_button);
  }

  // Center Buttons
  view_button_state  = get_button(joy_msg, view_button);
  menu_button_state  = get_button(joy_msg, menu_button);
  logo_button_state  = get_button(joy_msg, logo_button);
  share_button_state = get_button(joy_msg, share_button);

  // Bumpers
  left_bumper_button_state  = get_button(joy_msg, left_bumper_button);
  right_bumper_button_state = get_button(joy_msg, right_bumper_button);

  // Thumbsticks
  left_thumbstick_button_state  = get_button(joy_msg, left_thumbstick_button);
  right_thumbstick_button_state = get_button(joy_msg, right_thumbstick_button);

  // Triggers
  if (triggers_axis) {
    // D-pad buttons are treated as axes
    if (triggers_reversed) {
      // If triggers are reversed, we invert the axis values
      left_trigger_axis_state  = get_axis_position(joy_msg, left_trigger_axis,  1.0f, -1.0f, 0.0f, 1.0f);
      right_trigger_axis_state = get_axis_position(joy_msg, right_trigger_axis, 1.0f, -1.0f, 0.0f, 1.0f);
    } else {
        // Normal axis values
      left_trigger_axis_state  = get_axis_position(joy_msg, left_trigger_axis,  -1.0f, 1.0f, 0.0f, 1.0f);
      right_trigger_axis_state = get_axis_position(joy_msg, right_trigger_axis, -1.0f, 1.0f, 0.0f, 1.0f);
    }
  }
  else {
    // D-pad buttons are treated as buttons
    left_trigger_axis_state  = get_button(joy_msg, left_trigger_button) ? 1.0: 0.0;
    right_trigger_axis_state = get_button(joy_msg, right_trigger_button) ? 1.0: 0.0;
  }

  // If the trigger axis is pressed, we set the button state to true
  if (left_trigger_axis_state > 0.5) {
    left_trigger_button_state = true;
  } else {
    left_trigger_button_state = false;
  }

  if (right_trigger_axis_state > 0.5) {
    right_trigger_button_state = true;
  } else {
    right_trigger_button_state = false;
  }

  // Axis Thumbsticks
  left_thumbstick_x_axis_state = -get_axis_position(joy_msg, left_thumbstick_x_axis);
  left_thumbstick_y_axis_state = get_axis_position(joy_msg, left_thumbstick_y_axis);
  right_thumbstick_x_axis_state = -get_axis_position(joy_msg, right_thumbstick_x_axis);
  right_thumbstick_y_axis_state = get_axis_position(joy_msg, right_thumbstick_y_axis);

  // Angles and Distances from Thumbsticks
  float left_thumbstick_angle = std::atan2(-left_thumbstick_x_axis_state, left_thumbstick_y_axis_state) / M_PI * 180.0f;
  float left_thumbstick_distance = std::sqrt(
    left_thumbstick_x_axis_state * left_thumbstick_x_axis_state +
    left_thumbstick_y_axis_state * left_thumbstick_y_axis_state
  );
  float right_thumbstick_angle = std::atan2(-right_thumbstick_x_axis_state, right_thumbstick_y_axis_state) / M_PI * 180.0f;
  float right_thumbstick_distance = std::sqrt(
    right_thumbstick_x_axis_state * right_thumbstick_x_axis_state +
    right_thumbstick_y_axis_state * right_thumbstick_y_axis_state
  );
  
  left_thumbstick_distance*= 1.1f; // Scale the distance so that in the extremes is always 1.0
  left_thumbstick_distance = std::clamp(left_thumbstick_distance, 0.0f, 1.0f); // Ensure distance is between 0 and 1
  if (left_thumbstick_angle < 0.0) {
    left_thumbstick_angle += 360.0f;
  }
  
  right_thumbstick_distance*= 1.1f; // Scale the distance so that in the extremes is always 1.0
  right_thumbstick_distance = std::clamp(right_thumbstick_distance, 0.0f, 1.0f); // Ensure distance is between 0 and 1
  if (right_thumbstick_angle < 0.0) {
    right_thumbstick_angle += 360.0f;
  }

  /* 
  RCLCPP_INFO(rclcpp::get_logger("CHARMIEGamepad"),
              "A=%s, B=%s, X=%s, Y=%s",
              a_button_state ? "1" : "0",
              b_button_state ? "1" : "0",
              x_button_state ? "1" : "0",
              y_button_state ? "1" : "0"
  );

  RCLCPP_INFO(rclcpp::get_logger("CHARMIEGamepad"),
              "U=%s, D=%s, L=%s, R=%s",
              dpad_up_button_state    ? "1" : "0",
              dpad_down_button_state  ? "1" : "0",
              dpad_left_button_state  ? "1" : "0",
              dpad_right_button_state ? "1" : "0"
  );

  RCLCPP_INFO(rclcpp::get_logger("CHARMIEGamepad"),
              "Vi=%s, Me=%s, Lo=%s, Sh=%s",
              view_button_state  ? "1" : "0",
              menu_button_state  ? "1" : "0",
              logo_button_state  ? "1" : "0",
              share_button_state ? "1" : "0"
  );

  RCLCPP_INFO(rclcpp::get_logger("CHARMIEGamepad"),
              "LB=%s, RB=%s, LTB=%s, RTB=%s",
              left_bumper_button_state      ? "1" : "0",
              right_bumper_button_state     ? "1" : "0",
              left_thumbstick_button_state  ? "1" : "0",
              right_thumbstick_button_state ? "1" : "0"
  );

  RCLCPP_INFO(rclcpp::get_logger("CHARMIEGamepad"),
              "LT=%.2f, RT=%.2f, LTB=%s, RTB=%s",
              left_trigger_axis_state,
              right_trigger_axis_state,
              left_trigger_button_state  ? "1" : "0",
              right_trigger_button_state ? "1" : "0"
  );

  RCLCPP_INFO(rclcpp::get_logger("CHARMIEGamepad"),
              "LTBX=%.2f, LTBY=%.2f, RTBX=%.2f, RTBY=%.2f",
              left_thumbstick_x_axis_state,
              left_thumbstick_y_axis_state,
              right_thumbstick_x_axis_state,
              right_thumbstick_y_axis_state
  );


  RCLCPP_INFO(rclcpp::get_logger("CHARMIEGamepad"),
              "LTBAng=%.2f, LTBDist=%.2f, RTBAng=%.2f, RTBDist=%.2f",
              left_thumbstick_angle,
              left_thumbstick_distance,
              right_thumbstick_angle,
              right_thumbstick_distance
  ); 
  */

  charmie_interfaces::msg::GamepadController controller_msg;

  controller_msg.axes[0] = left_thumbstick_x_axis_state;
  controller_msg.axes[1] = left_thumbstick_y_axis_state;
  controller_msg.axes[2] = left_thumbstick_angle;
  controller_msg.axes[3] = left_thumbstick_distance;
  controller_msg.axes[4] = right_thumbstick_x_axis_state;
  controller_msg.axes[5] = right_thumbstick_y_axis_state;
  controller_msg.axes[6] = right_thumbstick_angle;
  controller_msg.axes[7] = right_thumbstick_distance;
  controller_msg.axes[8] = left_trigger_axis_state;
  controller_msg.axes[9] = right_trigger_axis_state;
  
  controller_msg.buttons[0] = a_button_state;
  controller_msg.buttons[1] = b_button_state;
  controller_msg.buttons[2] = x_button_state;
  controller_msg.buttons[3] = y_button_state;
  controller_msg.buttons[4] = left_bumper_button_state;
  controller_msg.buttons[5] = right_bumper_button_state;
  controller_msg.buttons[6] = left_trigger_button_state;
  controller_msg.buttons[7] = right_trigger_button_state;
  controller_msg.buttons[8] = left_thumbstick_button_state;
  controller_msg.buttons[9] = right_thumbstick_button_state;
  controller_msg.buttons[10] = dpad_up_button_state;
  controller_msg.buttons[11] = dpad_down_button_state;
  controller_msg.buttons[12] = dpad_left_button_state;
  controller_msg.buttons[13] = dpad_right_button_state;
  controller_msg.buttons[14] = view_button_state;
  controller_msg.buttons[15] = menu_button_state;
  controller_msg.buttons[16] = logo_button_state;
  controller_msg.buttons[17] = share_button_state;

  controller_msg.timeout = timeout;

  gamepad_controller_publisher->publish(controller_msg);

}

template<typename T>
T get_param_from_yaml(
  const YAML::Node & config,
  const std::string & key,
  const T & default_value)
{
  try {
    const auto & params = config["charmie_gamepad_node"]["ros__parameters"];
    if (params && params[key]) {
      return params[key].as<T>();
    }
  } catch (const YAML::Exception & e) {
    RCLCPP_WARN(rclcpp::get_logger("CHARMIEGamepad"), "YAML exception while reading '%s': %s", key.c_str(), e.what());
  }
  return default_value;
}

}  // namespace charmie_gamepad

RCLCPP_COMPONENTS_REGISTER_NODE(charmie_gamepad::CHARMIEGamepad)
