from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    publish_fake_scan = LaunchConfiguration("publish_fake_scan")
    fake_scan_motion_period = LaunchConfiguration("fake_scan_motion_period")
    parameter_file = PathJoinSubstitution(
        [FindPackageShare("costmap_inspector_example"), "config", "costmap_inspector_demo.yaml"]
    )
    map_file = PathJoinSubstitution([FindPackageShare("costmap_inspector_example"), "config", "demo_map.yaml"])

    fake_scan = Node(
        package="costmap_inspector_example",
        executable="fake_laser_scan.py",
        condition=IfCondition(publish_fake_scan),
        parameters=[{"motion_period": fake_scan_motion_period}],
        output="screen",
    )

    query_fake_obstacle = Node(
        package="costmap_inspector_example",
        executable="fake_costmap_query.py",
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "publish_fake_scan",
                default_value="True",
                description="Publish a synthetic LaserScan with a moving obstacle.",
            ),
            DeclareLaunchArgument(
                "fake_scan_motion_period",
                default_value="60.0",
                description="Orbit period of the synthetic obstacle in seconds.",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom",
                arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="odom_to_base_link",
                arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_laser",
                arguments=["0", "0", "0", "0", "0", "0", "base_link", "laser_frame"],
                output="screen",
            ),
            Node(
                package="nav2_costmap_2d",
                executable="nav2_costmap_2d",
                parameters=[parameter_file],
                output="screen",
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                parameters=[parameter_file, {"yaml_filename": map_file}],
                output="screen",
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                parameters=[
                    {
                        "autostart": True,
                        "node_names": ["map_server"],
                    }
                ],
                output="screen",
            ),
            TimerAction(
                period=6.0,
                actions=[
                    ExecuteProcess(
                        cmd=["ros2", "lifecycle", "set", "/costmap/costmap", "configure"],
                        output="screen",
                    )
                ],
            ),
            TimerAction(
                period=9.0,
                actions=[
                    ExecuteProcess(
                        cmd=["ros2", "lifecycle", "set", "/costmap/costmap", "activate"],
                        output="screen",
                    )
                ],
            ),
            fake_scan,
            query_fake_obstacle,
            LogInfo(msg="Inspector results: /costmap/inspector_layer/query_result"),
            LogInfo(msg="Debug layers: /costmap/inspector_layer/debug/<layer_name>"),
        ]
    )
