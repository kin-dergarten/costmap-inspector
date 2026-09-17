cmake_minimum_required(VERSION 3.8)
project(costmap_inspector)

set(CMAKE_CXX_STANDARD 20)
set(lib_name ${PROJECT_NAME}_core)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(costmap_inspector_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(dep_pkgs
  costmap_inspector_msgs
  geometry_msgs
  nav2_costmap_2d
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  tf2_geometry_msgs
  tf2_ros
)

add_library(${lib_name} SHARED
  src/cell.cpp
  src/costmap_inspector.cpp
  src/render_utils.cpp
)
target_include_directories(${lib_name} PUBLIC include)
ament_target_dependencies(${lib_name} ${dep_pkgs})

install(TARGETS ${lib_name} DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES resource/costmap_inspector.xml DESTINATION share/${PROJECT_NAME})
pluginlib_export_plugin_description_file(nav2_costmap_2d resource/costmap_inspector.xml)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

ament_package()

