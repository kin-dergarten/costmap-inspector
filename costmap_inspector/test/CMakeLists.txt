cmake_minimum_required(VERSION 3.8)

find_package(ament_cmake_gtest REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

ament_add_gtest(layer_query_test
        layer_query_test.cpp
        test_main.cpp
)
target_link_libraries(layer_query_test ${lib_name})
ament_target_dependencies(layer_query_test
        ${dep_pkgs}
        rclcpp_lifecycle
        nav2_costmap_2d
)
