cmake_minimum_required(VERSION 3.8)
project(costmap_inspector_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CostmapQueryData.msg"
  "msg/CostmapQueryLayerInfo.msg"
  "srv/CostmapQuery.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

