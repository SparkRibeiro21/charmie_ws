cmake_minimum_required(VERSION 3.8)
project(charmie_commander_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(charmie_interfaces REQUIRED)

add_executable(test_moveit src/test_moveit.cpp)
ament_target_dependencies(test_moveit rclcpp moveit_ros_planning_interface)

add_executable(test_avoid_obstacle src/test_avoid_obstacle.cpp)
ament_target_dependencies(test_avoid_obstacle rclcpp moveit_ros_planning_interface)

add_executable(test_pick_and_place_moveit src/test_pick_and_place_moveit.cpp)
ament_target_dependencies(test_pick_and_place_moveit rclcpp moveit_ros_planning_interface)

add_executable(commander src/commander_template.cpp)
ament_target_dependencies(commander rclcpp moveit_ros_planning_interface example_interfaces charmie_interfaces)

install(TARGETS
  test_moveit
  test_avoid_obstacle
  test_pick_and_place_moveit
  commander
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
