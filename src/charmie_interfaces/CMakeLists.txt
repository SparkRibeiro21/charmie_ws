cmake_minimum_required(VERSION 3.5)
project(charmie_interfaces)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(example_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmController.msg"
  "msg/BoundingBox.msg"
  "msg/BoundingBoxAndPoints.msg"
  "msg/DetectedObject.msg"
  "msg/DetectedPerson.msg"
  "msg/Encoders.msg"
  "msg/ListOfImages.msg"
  "msg/ListOfFloats.msg"
  "msg/ListOfPoints.msg"
  "msg/ListOfStrings.msg"
  "msg/NeckPosition.msg"
  "msg/ObstacleInfo.msg"
  "msg/Obstacles.msg"
  "msg/PointCloudCoordinates.msg"
  "msg/PS4Controller.msg"
  "msg/RequestPointCloud.msg"
  "msg/RetrievePointCloud.msg"
  "msg/RGB.msg"
  "msg/RobotSpeech.msg"
  "msg/SearchForPerson.msg"
  "msg/SpeechType.msg"
  "msg/TarNavSDNL.msg"
  "msg/Yolov8Objects.msg"
  "msg/Yolov8Pose.msg"
  "srv/ActivateYoloObjects.srv"
  "srv/ActivateYoloPose.srv"
  "srv/ArmTrigger.srv"
  "srv/CalibrateAudio.srv"
  "srv/GetAudio.srv"
  "srv/GetNeckPosition.srv"
  "srv/GetPointCloud.srv"
  "srv/NavTrigger.srv"
  "srv/SaveSpeechCommand.srv"
  "srv/SetNeckCoordinates.srv"
  "srv/SetNeckPosition.srv"
  "srv/SpeechCommand.srv"
  "srv/TrackObject.srv"
  "srv/TrackPerson.srv"
  DEPENDENCIES example_interfaces geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
