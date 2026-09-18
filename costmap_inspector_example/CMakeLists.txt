cmake_minimum_required(VERSION 3.8)
project(costmap_inspector_example)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(
  PROGRAMS
    scripts/fake_costmap_query.py
    scripts/fake_laser_scan.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
