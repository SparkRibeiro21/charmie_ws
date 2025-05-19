cmake_minimum_required(VERSION 3.8)
project(charmie_radar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

# for new executables duplicate the following two lines and added the executable name to install()
add_executable(charmie_radar src/charmie_radar.cpp)
ament_target_dependencies(charmie_radar rclcpp)

install(TARGETS
  charmie_radar
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
