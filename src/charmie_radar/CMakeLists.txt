cmake_minimum_required(VERSION 3.8)
project(charmie_radar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(charmie_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io)
find_package(pcl_conversions REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# Build executable
# for new executables duplicate the following two lines and added the executable name to install()
add_executable(radar src/radar.cpp)
ament_target_dependencies(radar 
  rclcpp
  ament_index_cpp
  sensor_msgs
  charmie_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
)

# Manually link yaml-cpp
target_link_libraries(radar yaml-cpp)
target_link_libraries(radar
  ${PCL_LIBRARIES}
)

# Install
install(TARGETS
  radar
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
