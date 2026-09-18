#!/usr/bin/env python3

import math
import random

import rclpy
from costmap_inspector_msgs.msg import CostmapQueryData
from costmap_inspector_msgs.srv import CostmapQuery
from geometry_msgs.msg import Point32, PoseStamped
from rclpy.node import Node


class FakeCostmapQuery(Node):
    def __init__(self):
        super().__init__("fake_costmap_query")

        self.declare_parameter("service_name", "/costmap/inspector_layer/polygon_query")
        self.declare_parameter("result_topic", "/costmap/inspector_layer/query_result")
        self.declare_parameter("query_period", 2.0)
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("query_area_width", 4.0)
        self.declare_parameter("query_area_height", 4.0)

        service_name = self.get_parameter("service_name").value
        result_topic = self.get_parameter("result_topic").value
        query_period = self.get_parameter("query_period").value
        self.frame_id = self.get_parameter("frame_id").value
        self.query_area_width = float(self.get_parameter("query_area_width").value)
        self.query_area_height = float(self.get_parameter("query_area_height").value)

        self.client = self.create_client(CostmapQuery, service_name)
        self.result_subscription = self.create_subscription(
            CostmapQueryData,
            result_topic,
            self.handle_result,
            10,
        )
        self.timer = self.create_timer(query_period, self.send_query)
        self.get_logger().info(
            f"Querying {service_name} every {query_period:g} seconds and listening on {result_topic}"
        )

    def send_query(self):
        if not self.client.service_is_ready():
            self.get_logger().info("Waiting for the costmap inspector query service")
            return

        request = CostmapQuery.Request()
        request.use_costmap_footprint = False
        request.footprint.points = [
            Point32(x=-0.35, y=0.3),
            Point32(x=0.35, y=0.3),
            Point32(x=0.35, y=-0.3),
            Point32(x=-0.35, y=-0.3),
        ]
        request.footprint_pose = PoseStamped()
        request.footprint_pose.header.frame_id = self.frame_id
        request.footprint_pose.pose.position.x = random.uniform(
            -self.query_area_width / 2.0, self.query_area_width / 2.0
        )
        request.footprint_pose.pose.position.y = random.uniform(
            -self.query_area_height / 2.0, self.query_area_height / 2.0
        )
        request.footprint_pose.pose.orientation.w = 1.0
        request.persist_result = False

        self.get_logger().info(
            f"Querying random point ({request.footprint_pose.pose.position.x:.2f}, "
            f"{request.footprint_pose.pose.position.y:.2f}) in {self.frame_id}"
        )

        future = self.client.call_async(request)
        future.add_done_callback(self.handle_request_response)

    def handle_request_response(self, future):
        try:
            response = future.result()
        except Exception as exception:
            self.get_logger().error(f"Costmap query failed: {exception}")
            return

        if not response.successfully_added_to_queue:
            self.get_logger().warning("Costmap inspector rejected the query")

    def handle_result(self, result: CostmapQueryData):
        if result.result == CostmapQueryData.RESULT_FAILURE:
            self.get_logger().warning("Costmap query returned a failure")
            return

        self.get_logger().info(
            "Costmap query: "
            f"{result.num_lethal_cells_in_most_lethal_layer} lethal cells, "
            f"layer={result.most_lethal_layer or 'none'}, "
            f"distance={result.mean_object_distance:.2f} m, "
            f"angle={math.degrees(result.mean_object_angle):.1f} deg"
        )


def main(args=None):
    rclpy.init(args=args)
    node = FakeCostmapQuery()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
