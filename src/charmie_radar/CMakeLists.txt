cmake_minimum_required(VERSION 3.8)
project(charmie_radar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

# Build executable
# for new executables duplicate the following two lines and added the executable name to install()
add_executable(radar src/radar.cpp)
ament_target_dependencies(radar rclcpp)

# Manually link yaml-cpp
target_link_libraries(radar yaml-cpp)

# Install
install(TARGETS
  radar
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
