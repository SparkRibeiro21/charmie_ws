cmake_minimum_required(VERSION 3.8)
project(charmie_sdnl_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(charmie_interfaces REQUIRED)

add_executable(sdnl_nav src/sdnl_nav.cpp)

ament_target_dependencies(sdnl_nav
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  charmie_interfaces
)

install(TARGETS
  sdnl_nav
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
