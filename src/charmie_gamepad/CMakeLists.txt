cmake_minimum_required(VERSION 3.8)
project(charmie_gamepad)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(charmie_interfaces REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(SDL2 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)
include_directories(${SDL2_INCLUDE_DIRS})

# Build executable
add_executable(charmie_gamepad_node
  src/charmie_gamepad.cpp
  src/charmie_gamepad_node.cpp
  src/sdl_utils.cpp
)

ament_target_dependencies(charmie_gamepad_node
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  charmie_interfaces
  ament_index_cpp
)

target_link_libraries(charmie_gamepad_node 
${SDL2_LIBRARIES}
yaml-cpp
)

# Install executable
install(TARGETS charmie_gamepad_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch/config files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()