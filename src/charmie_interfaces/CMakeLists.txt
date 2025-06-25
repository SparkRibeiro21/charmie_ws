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
  "action/CountUntil.action"
  "msg/ArmController.msg"
  "msg/BoundingBox.msg"
  "msg/BoundingBoxAndPoints.msg"
  "msg/ButtonsLowLevel.msg"
  "msg/DetectedObject.msg"
  "msg/DetectedPerson.msg"
  "msg/Encoders.msg"
  "msg/ErrorsMotorBoard.msg"
  "msg/GamepadController.msg"
  "msg/ListOfDetectedObject.msg"
  "msg/ListOfDetectedPerson.msg"
  "msg/ListOfImages.msg"
  "msg/ListOfMaskDetections.msg"
  "msg/ListOfFloats.msg"
  "msg/ListOfPoints.msg"
  "msg/ListOfStrings.msg"
  "msg/MaskDetection.msg"
  "msg/NeckPosition.msg"
  "msg/ObstacleInfo.msg"
  "msg/Obstacles.msg"
  "msg/PointCloudCoordinates.msg"
  "msg/PS4Controller.msg"
  "msg/RGB.msg"
  "msg/TarNavSDNL.msg"
  "msg/TaskStatesInfo.msg"
  "msg/TorsoPosition.msg"
  "msg/TrackingMask.msg"
  "msg/VCCsLowLevel.msg"
  "srv/ActivateBool.srv"
  "srv/ActivateObstacles.srv"
  "srv/ActivateTracking.srv"
  "srv/ActivateYoloObjects.srv"
  "srv/ActivateYoloPose.srv"
  "srv/CalibrateAudio.srv"
  "srv/ContinuousGetAudio.srv"
  "srv/GetAudio.srv"
  "srv/GetFaceTouchscreenMenu.srv"
  "srv/GetLLMConfirmCommand.srv"
  "srv/GetLLMDemo.srv"
  "srv/GetLLMGPSR.srv"
  "srv/GetLowLevelButtons.srv"
  "srv/GetNeckPosition.srv"
  "srv/GetPointCloudBB.srv"
  "srv/GetPointCloudMask.srv"
  "srv/GetTorso.srv"
  "srv/GetVCCs.srv"
  "srv/NodesUsed.srv"
  "srv/SaveSpeechCommand.srv"
  "srv/SetAcceleration.srv"
  "srv/SetFace.srv"
  "srv/SetFaceTouchscreenMenu.srv"
  "srv/SetInt.srv"
  "srv/SetNeckCoordinates.srv"
  "srv/SetNeckPosition.srv"
  "srv/SetPoseWithCovarianceStamped.srv"
  "srv/SetRGB.srv"
  "srv/SetString.srv"
  "srv/SetTextFace.srv"
  "srv/SetTorso.srv"
  "srv/SpeechCommand.srv"
  "srv/TrackContinuous.srv"
  "srv/TrackObject.srv"
  "srv/TrackPerson.srv"
  "srv/Trigger.srv"
  DEPENDENCIES example_interfaces geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
