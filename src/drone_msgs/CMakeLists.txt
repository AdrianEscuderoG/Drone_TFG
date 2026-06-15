cmake_minimum_required(VERSION 3.8)
project(drone_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CropAnalysis.msg"
  "msg/DroneStatus.msg"
  "msg/MissionPlan.msg"
  "msg/MissionWaypoint.msg"
  "msg/PilotCommand.msg"
  "srv/GetDroneState.srv"
  DEPENDENCIES
    std_msgs geometry_msgs sensor_msgs vision_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
