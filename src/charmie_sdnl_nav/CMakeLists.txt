cmake_minimum_required(VERSION 3.8)
project(charmie_sdnl_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(sdnl_nav src/sdnl_nav.cpp)

ament_target_dependencies(sdnl_nav
  rclcpp
)

install(TARGETS
  sdnl_nav
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
