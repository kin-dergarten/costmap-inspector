#ifndef COSTMAP_INSPECTOR__COSTMAP_INSPECTOR_HPP
#define COSTMAP_INSPECTOR__COSTMAP_INSPECTOR_HPP

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

#include <costmap_inspector_msgs/msg/costmap_query_data.hpp>
#include <costmap_inspector_msgs/msg/costmap_query_layer_info.hpp>
#include <costmap_inspector_msgs/srv/costmap_query.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "costmap_inspector/cell.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace costmap_inspector
{

// Parameter name constants
constexpr const char* PARAM_ENABLED = "enabled";
constexpr const char* PARAM_LETHAL_LAYERS_TIMEOUT_SECONDS = "lethal_layers_timeout_seconds";
constexpr const char* PARAM_DEBUG_PUBLISH_CHECKED_FOOTPRINT = "debug.publish_checked_footprint";
constexpr const char* PARAM_DEBUG_PUBLISH_INDIVIDUAL_LAYERS = "debug.publish_individual_layers";
constexpr const char* PARAM_DEBUG_PUBLISH_INDIVIDUAL_LAYERS_PERIODICALLY =
    "debug.publish_individual_layers_periodically";
constexpr const char* PARAM_DEBUG_PUBLISH_PERIODICALLY_PERIOD_SECONDS = "debug.publish_periodically_period_seconds";
constexpr const char* PARAM_QUERY_RESULT_TOPIC = "query_result_topic";
constexpr const char* PARAM_LETHAL_POINTS_TOPIC = "lethal_points_topic";
constexpr const char* PARAM_QUERY_SERVICE = "query_service";
constexpr const char* PARAM_BASE_FRAME = "base_frame";

const int MAX_REQUESTS_QUEUE_SIZE = 5;

struct LayerData
{
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr debugPublisher;
  std::chrono::steady_clock::time_point lastLethalTime;
  bool publishedData = true;  // Send inital map always
};

class LayerInspector : public nav2_costmap_2d::Layer
{

 public:
  LayerInspector() = default;

  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                    double* max_y) override;
  void updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;

  void reset() override {}

  void onFootprintChanged() override;

  bool isClearable() override { return false; }

  static costmap_inspector_msgs::msg::CostmapQueryLayerInfo buildLayerInfo(
      const std::string& layerName, costmap_inspector_msgs::msg::CostmapQueryLayerInfo::_num_lethal_type numLethal)
  {
    costmap_inspector_msgs::msg::CostmapQueryLayerInfo info;
    info.layer_name = layerName;
    info.num_lethal = numLethal;
    return info;
  }

 protected:
  rclcpp::Service<costmap_inspector_msgs::srv::CostmapQuery>::SharedPtr costmapQuerySrv;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygonPublisher;
  rclcpp::Publisher<costmap_inspector_msgs::msg::CostmapQueryData>::SharedPtr resultPublisher;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dynParamsHandler;

  void costmapQueryCallback(const costmap_inspector_msgs::srv::CostmapQuery::Request::SharedPtr request,
                            costmap_inspector_msgs::srv::CostmapQuery::Response::SharedPtr response);

  std::unordered_map<std::string, LayerData> layerDataMap;
  void markLayerLethal(const std::string& layerName, const std::chrono::steady_clock::time_point currentTime);
  std::vector<std::string> getLethalLayers();

  void publishDebugLayer(const std::string& layerName, const nav2_costmap_2d::Costmap2D& layerCostmap,
                         bool containsData);
  void createLayerDebugPublishers(rclcpp_lifecycle::LifecycleNode::SharedPtr& node);
  static nav_msgs::msg::OccupancyGrid costmapToOccupancyGrid(const nav2_costmap_2d::Costmap2D& costmap,
                                                             const std::string& frameID, bool noData);
  bool transformPolygonToFrame(const geometry_msgs::msg::PolygonStamped& inputPolygon,
                               geometry_msgs::msg::PolygonStamped& outputPolygon, const std::string& targetFrame) const;

  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(const std::vector<rclcpp::Parameter>& parameters);

  bool paramDebugPublishCheckedFootprint;
  bool paramDebugPublishIndividualLayers;
  bool paramDebugPublishIndividualLayersPeriodically;
  double paramDebugPublishPeriodicallyPeriodSeconds;
  double paramLethalLayersTimeoutSeconds;
  std::string paramQueryResultTopic;
  std::string paramLethalPointsTopic;
  std::string paramQueryService;
  std::string paramBaseFrame;

  std::mutex requestDataMutex;
  std::queue<costmap_inspector_msgs::srv::CostmapQuery_Request::SharedPtr> pendingRequests;
  std::unordered_set<uint64_t> currentlyLethalCellsIndex;

  void loadParameters(rclcpp_lifecycle::LifecycleNode::SharedPtr& node);
  std::unordered_map<std::string, std::string> sourceNameMap;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lethalPointsPublisher;

  void queryLayer(const costmap_inspector_msgs::srv::CostmapQuery_Request::SharedPtr& request,
                  const std::vector<Cell>& polygonCells, const std::string& layerName,
                  const nav2_costmap_2d::Costmap2D& layerCostmap, costmap_inspector_msgs::msg::CostmapQueryData& resultData);
  bool preparePolygon(costmap_inspector_msgs::srv::CostmapQuery_Request::SharedPtr request,
                      geometry_msgs::msg::PolygonStamped& transformedFootprint);
  bool buildPointCloud2Message(const nav2_costmap_2d::Costmap2D& layerCostmap,
                               sensor_msgs::msg::PointCloud2& pointCloud, geometry_msgs::msg::Point& meanPoint);

  bool prepareRequest(costmap_inspector_msgs::srv::CostmapQuery_Request::SharedPtr request,
                      nav2_costmap_2d::Costmap2D& layerCostmap, costmap_inspector_msgs::msg::CostmapQueryData& resultData,
                      std::vector<Cell>& cellsInPolygon);
  bool handleRequestResults(costmap_inspector_msgs::msg::CostmapQueryData& resultData, nav2_costmap_2d::Costmap2D& layerCostmap);
};

}  // namespace costmap_inspector

#endif  // COSTMAP_INSPECTOR__COSTMAP_INSPECTOR_HPP
