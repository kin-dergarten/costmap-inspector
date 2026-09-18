#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class FakeLaserScan(Node):
    def __init__(self):
        super().__init__("fake_laser_scan")

        self.declare_parameter("topic", "/scan")
        self.declare_parameter("frame_id", "laser_frame")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("angular_resolution_degrees", 0.25)
        self.declare_parameter("motion_period", 60.0)
        self.declare_parameter("min_obstacle_distance", 0.6)
        self.declare_parameter("max_obstacle_distance", 2.0)
        self.declare_parameter("obstacle_radius", 0.12)

        topic = self.get_parameter("topic").value
        self.frame_id = self.get_parameter("frame_id").value
        publish_rate = self.get_parameter("publish_rate").value
        self.angular_resolution = math.radians(
            float(self.get_parameter("angular_resolution_degrees").value)
        )
        if self.angular_resolution <= 0.0:
            raise ValueError("angular_resolution_degrees must be greater than zero")
        self.motion_period = float(self.get_parameter("motion_period").value)
        if self.motion_period <= 0.0:
            raise ValueError("motion_period must be greater than zero")
        self.min_obstacle_distance = float(
            self.get_parameter("min_obstacle_distance").value
        )
        self.max_obstacle_distance = float(
            self.get_parameter("max_obstacle_distance").value
        )
        if not 0.0 < self.min_obstacle_distance <= self.max_obstacle_distance:
            raise ValueError("obstacle distances must satisfy 0 < min <= max")
        self.obstacle_radius = self.get_parameter("obstacle_radius").value

        self.publisher = self.create_publisher(LaserScan, topic, 10)
        self.start_time = time.monotonic()
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_scan)
        angular_speed = 2.0 * math.pi / self.motion_period
        self.get_logger().info(
            f"Moving obstacle orbit period: {self.motion_period:g} seconds "
            f"({angular_speed:g} rad/s)"
        )

    def publish_scan(self):
        angle_min = -math.pi
        angle_max = math.pi
        angle_increment = self.angular_resolution
        ray_count = round((angle_max - angle_min) / angle_increment) + 1
        range_min = 0.05
        range_max = 5.0

        elapsed = time.monotonic() - self.start_time
        obstacle_angle = (2.0 * math.pi * elapsed / self.motion_period) % (2.0 * math.pi)
        distance_midpoint = (self.min_obstacle_distance + self.max_obstacle_distance) / 2.0
        distance_amplitude = (self.max_obstacle_distance - self.min_obstacle_distance) / 2.0
        obstacle_range = distance_midpoint - distance_amplitude * math.cos(obstacle_angle)
        obstacle_x = obstacle_range * math.cos(obstacle_angle)
        obstacle_y = obstacle_range * math.sin(obstacle_angle)
        obstacle_bearing = math.atan2(obstacle_y, obstacle_x)
        angular_radius = math.asin(min(self.obstacle_radius / obstacle_range, 1.0))

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / float(self.get_parameter("publish_rate").value)
        scan.range_min = range_min
        scan.range_max = range_max
        scan.ranges = [math.inf] * ray_count
        scan.intensities = []

        for index in range(ray_count):
            ray_angle = angle_min + index * angle_increment
            angle_error = math.atan2(
                math.sin(ray_angle - obstacle_bearing),
                math.cos(ray_angle - obstacle_bearing),
            )
            if abs(angle_error) <= angular_radius:
                scan.ranges[index] = obstacle_range

        self.publisher.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = FakeLaserScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
