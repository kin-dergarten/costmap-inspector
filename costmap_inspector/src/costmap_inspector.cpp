#include "costmap_inspector/costmap_inspector.hpp"

#include "costmap_inspector/constants.hpp"
#include "costmap_inspector/msg_utils.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/costmap_math.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include "costmap_inspector/render_utils.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace costmap_inspector
{
using costmap_inspector_msgs::msg::CostmapQueryData;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initialization
// of need_recalculation_ variable.
void LayerInspector::onInitialize()
{
  auto node = node_.lock();

  loadParameters(node);

  RCLCPP_INFO(logger_, "namespace: %s, name: %s", node->get_namespace(), node->get_name());
  std::string toplevelNamespace = node->get_namespace();
  const std::string nodeName = node->get_name();
  if (toplevelNamespace.ends_with(nodeName)) {
    toplevelNamespace = toplevelNamespace.erase(toplevelNamespace.size() - nodeName.size());
  }
  const auto resolveName = [&toplevelNamespace](const std::string& name) {
    return name.starts_with('/') ? name : toplevelNamespace + name;
  };

  std::string topic = resolveName(paramQueryResultTopic);
  resultPublisher = node->create_publisher<CostmapQueryData>(topic, rclcpp::SystemDefaultsQoS());
  RCLCPP_INFO(logger_, "Created result publisher on topic: %s", topic.c_str());

  topic = resolveName(paramLethalPointsTopic);
  lethalPointsPublisher = node->create_publisher<sensor_msgs::msg::PointCloud2>(topic, rclcpp::SystemDefaultsQoS());
  RCLCPP_INFO(logger_, "Created lethal points publisher on topic: %s", topic.c_str());

  createLayerDebugPublishers(node);
  polygonPublisher = node->create_publisher<geometry_msgs::msg::PolygonStamped>(name_ + "/checked_footprint",
                                                                                rclcpp::SystemDefaultsQoS());
  topic = resolveName(paramQueryService);
  costmapQuerySrv = node->create_service<costmap_inspector_msgs::srv::CostmapQuery>(
      topic, [this](const std::shared_ptr<costmap_inspector_msgs::srv::CostmapQuery::Request> request,
                    std::shared_ptr<costmap_inspector_msgs::srv::CostmapQuery::Response> response) {
        costmapQueryCallback(request, response);
      });

  // Set up dynamic parameter callback
  dynParamsHandler = node->add_on_set_parameters_callback([this]<typename InputType>(InputType&& parameter) {
    return dynamicParametersCallback(std::forward<InputType>(parameter));
  });

  current_ = true;
}

void LayerInspector::loadParameters(rclcpp_lifecycle::LifecycleNode::SharedPtr& node)
{
  declareParameter(PARAM_ENABLED, rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + PARAM_ENABLED, enabled_);

  declareParameter(PARAM_DEBUG_PUBLISH_CHECKED_FOOTPRINT, rclcpp::ParameterValue(false));
  node->get_parameter(name_ + "." + PARAM_DEBUG_PUBLISH_CHECKED_FOOTPRINT, paramDebugPublishCheckedFootprint);
  declareParameter(PARAM_DEBUG_PUBLISH_INDIVIDUAL_LAYERS, rclcpp::ParameterValue(false));
  node->get_parameter(name_ + "." + PARAM_DEBUG_PUBLISH_INDIVIDUAL_LAYERS, paramDebugPublishIndividualLayers);
  declareParameter(PARAM_DEBUG_PUBLISH_INDIVIDUAL_LAYERS_PERIODICALLY, rclcpp::ParameterValue(false));
  node->get_parameter(name_ + "." + PARAM_DEBUG_PUBLISH_INDIVIDUAL_LAYERS_PERIODICALLY,
                      paramDebugPublishIndividualLayersPeriodically);
  declareParameter(PARAM_DEBUG_PUBLISH_PERIODICALLY_PERIOD_SECONDS, rclcpp::ParameterValue(1.0));
  node->get_parameter(name_ + "." + PARAM_DEBUG_PUBLISH_PERIODICALLY_PERIOD_SECONDS,
                      paramDebugPublishPeriodicallyPeriodSeconds);
  declareParameter(PARAM_LETHAL_LAYERS_TIMEOUT_SECONDS, rclcpp::ParameterValue(0.5));
  node->get_parameter(name_ + "." + PARAM_LETHAL_LAYERS_TIMEOUT_SECONDS, paramLethalLayersTimeoutSeconds);
  declareParameter(PARAM_QUERY_RESULT_TOPIC, rclcpp::ParameterValue(topic::DEFAULT_QUERY_RESULT_TOPIC));
  node->get_parameter(name_ + "." + PARAM_QUERY_RESULT_TOPIC, paramQueryResultTopic);
  declareParameter(PARAM_LETHAL_POINTS_TOPIC, rclcpp::ParameterValue(topic::DEFAULT_LETHAL_POINTS_TOPIC));
  node->get_parameter(name_ + "." + PARAM_LETHAL_POINTS_TOPIC, paramLethalPointsTopic);
  declareParameter(PARAM_QUERY_SERVICE, rclcpp::ParameterValue(service::DEFAULT_QUERY_SERVICE));
  node->get_parameter(name_ + "." + PARAM_QUERY_SERVICE, paramQueryService);
  declareParameter(PARAM_BASE_FRAME, rclcpp::ParameterValue(frame::DEFAULT_BASE_FRAME));
  node->get_parameter(name_ + "." + PARAM_BASE_FRAME, paramBaseFrame);

  // load source map: maps layer names to their human-readable source names
  const std::string prefix = "source_names";
  std::vector<std::string> sources;
  declareParameter(prefix + ".sources", rclcpp::ParameterValue(std::vector<std::string>{}));
  node->get_parameter(name_ + "." + prefix + ".sources", sources);

  for (const auto& source : sources) {
    std::string sourceName;
    declareParameter(prefix + "." + source + ".name", rclcpp::ParameterValue(""));
    node->get_parameter(name_ + "." + prefix + "." + source + ".name", sourceName);

    std::vector<std::string> layers;
    declareParameter(prefix + "." + source + ".layers", rclcpp::ParameterValue(std::vector<std::string>{}));
    node->get_parameter(name_ + "." + prefix + "." + source + ".layers", layers);

    for (const auto& layer : layers) {
      sourceNameMap[layer] = sourceName;
      RCLCPP_INFO(logger_, "Source map: layer '%s' -> source '%s'", layer.c_str(), sourceName.c_str());
    }
  }
}

void LayerInspector::createLayerDebugPublishers(rclcpp_lifecycle::LifecycleNode::SharedPtr& node)
{
  if (paramDebugPublishIndividualLayers || paramDebugPublishIndividualLayersPeriodically) {
    RCLCPP_INFO(logger_, "Debug publishing of individual layers is ENABLED. Preparing publishers.");
    std::vector<std::string> layerNames;
    try {
      node->get_parameter("plugins", layerNames);
      for (const auto& layerName : layerNames) {
        if (!layerName.empty() && !layerDataMap.contains(layerName)) {
          layerDataMap[layerName].debugPublisher =
              node->create_publisher<nav_msgs::msg::OccupancyGrid>(name_ + "/debug/" + layerName, 1);
          RCLCPP_INFO(logger_, " - %s", layerName.c_str());
        }
      }

    } catch (const std::exception& e) {
      RCLCPP_ERROR(logger_,
                   "Failed to get 'plugins' parameter: %s. Publishers must be created later for each layer (first "
                   "message might be dropped).",
                   e.what());
    }
  }
}

bool LayerInspector::transformPolygonToFrame(const geometry_msgs::msg::PolygonStamped& inputPolygon,
                                             geometry_msgs::msg::PolygonStamped& outputPolygon,
                                             const std::string& targetFrame) const
{
  if (inputPolygon.header.frame_id == targetFrame) {
    outputPolygon = inputPolygon;
    return true;
  }
  if (!tf_) {
    RCLCPP_ERROR(logger_, "TF buffer is not available");
    return false;
  }

  try {
    // Transform the polygon
    tf2::doTransform(inputPolygon, outputPolygon,
                     tf_->lookupTransform(targetFrame, inputPolygon.header.frame_id, tf2::TimePointZero));

    RCLCPP_DEBUG(logger_, "Transformed polygon from frame '%s' to '%s'", inputPolygon.header.frame_id.c_str(),
                 targetFrame.c_str());
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(logger_, "Failed to transform polygon from '%s' to '%s': %s", inputPolygon.header.frame_id.c_str(),
                 targetFrame.c_str(), ex.what());
    return false;
  }
}

// This method is the callback handling the incoming query
// It takes the provided polygon and queries each layer in the costmap for the number of
// lethal obstacles present, reporting this in the result.
void LayerInspector::costmapQueryCallback(const costmap_inspector_msgs::srv::CostmapQuery::Request::SharedPtr request,
                                          costmap_inspector_msgs::srv::CostmapQuery::Response::SharedPtr response)
{
  const auto startTime = std::chrono::steady_clock::now();
  try {
    std::scoped_lock lock(requestDataMutex);
    if (pendingRequests.size() >= MAX_REQUESTS_QUEUE_SIZE) {
      RCLCPP_WARN(logger_, "Costmap query request queue is full (size: %lu). Dropping oldest request.",
                  pendingRequests.size());
      pendingRequests.pop();
    }
    pendingRequests.push(request);
    auto elapsedTime =
        std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - startTime).count();
    if (elapsedTime > 1000) {
      RCLCPP_WARN(logger_, "Costmap query callback is taking a long time: %ld us", elapsedTime);
    }
    response->successfully_added_to_queue = true;
  } catch (const std::exception& e) {
    RCLCPP_INFO(logger_, "CostmapQueryCallback failed: %s", e.what());
    response->successfully_added_to_queue = false;
  }
}

bool LayerInspector::preparePolygon(costmap_inspector_msgs::srv::CostmapQuery_Request::SharedPtr request,
                                    geometry_msgs::msg::PolygonStamped& transformedFootprint)
{
  // use footprint if requested
  geometry_msgs::msg::PolygonStamped footprintPolygonStamped;
  footprintPolygonStamped.header = request->footprint_pose.header;

  footprintPolygonStamped.polygon = request->footprint;

  if (request->use_costmap_footprint) {
    auto footprint = layered_costmap_->getFootprint();
    footprintPolygonStamped.polygon.points.clear();
    for (const auto& point : footprint) {
      footprintPolygonStamped.polygon.points.push_back(msg_utils::point32FromPoint(point));
    }
  }

  // transform footprintPolygon to the correct position (given by the pose)
  geometry_msgs::msg::TransformStamped poseToFootprintTransform;
  poseToFootprintTransform.header = request->footprint_pose.header;
  poseToFootprintTransform.transform.translation.x = request->footprint_pose.pose.position.x;
  poseToFootprintTransform.transform.translation.y = request->footprint_pose.pose.position.y;
  poseToFootprintTransform.transform.translation.z = request->footprint_pose.pose.position.z;
  poseToFootprintTransform.transform.rotation = request->footprint_pose.pose.orientation;

  geometry_msgs::msg::PolygonStamped localFramePolygon;
  tf2::doTransform(footprintPolygonStamped, localFramePolygon, poseToFootprintTransform);

  std::string costmapFrame = layered_costmap_->getGlobalFrameID();

  if (!transformPolygonToFrame(localFramePolygon, transformedFootprint, costmapFrame)) {
    RCLCPP_ERROR(logger_, "Failed to transform polygon to costmap frame '%s'", costmapFrame.c_str());
    return false;
  }
  if (transformedFootprint.header.stamp.sec == 0 && transformedFootprint.header.stamp.nanosec == 0) {
    transformedFootprint.header.stamp = rclcpp::Clock().now();
  }
  polygonPublisher->publish(transformedFootprint);
  return true;
}

void LayerInspector::queryLayer(const costmap_inspector_msgs::srv::CostmapQuery_Request::SharedPtr& request,
                                const std::vector<costmap_inspector::Cell>& polygonCells, const std::string& layerName,
                                const nav2_costmap_2d::Costmap2D& layerCostmap, CostmapQueryData& resultData)
{
  unsigned char* data = layerCostmap.getCharMap();

  int64_t lethalCount = 0;

  for (const auto& cell : polygonCells) {
    unsigned int index = layerCostmap.getIndex(cell.x, cell.y);
    if (data[index] == LETHAL_OBSTACLE) {
      lethalCount++;
      currentlyLethalCellsIndex.insert(index);
    }
  }

  // Add results to response
  resultData.layer_info.push_back(buildLayerInfo(layerName, lethalCount));
  if (lethalCount > 0) {
    if (request->persist_result) {
      markLayerLethal(layerName, std::chrono::steady_clock::now());
    }
    if (resultData.num_lethal_cells_in_most_lethal_layer < lethalCount) {
      resultData.most_lethal_layer = layerName;
      resultData.num_lethal_cells_in_most_lethal_layer = lethalCount;
    }
  }
}

void LayerInspector::publishDebugLayer(const std::string& layerName, const nav2_costmap_2d::Costmap2D& layerCostmap,
                                       bool containsData)
{
  if ((!paramDebugPublishIndividualLayersPeriodically && !paramDebugPublishIndividualLayers) || layerName.empty()) {
    return;
  }
  if (!layerDataMap.contains(layerName) || layerDataMap[layerName].debugPublisher == nullptr) {
    RCLCPP_INFO(logger_, "Creating debug publisher for layer '%s'", layerName.c_str());
    auto node = node_.lock();
    layerDataMap[layerName].debugPublisher =
        node->create_publisher<nav_msgs::msg::OccupancyGrid>(name_ + "/debug/" + layerName, 1);
  }
  // only create the occupancy map if map changed. Do not repeatedly send disabled layers. Disabled layers report UNKNOWN
  if (containsData || layerDataMap[layerName].publishedData) {
    auto occupancyGrid = costmapToOccupancyGrid(layerCostmap, layered_costmap_->getGlobalFrameID(), !containsData);
    layerDataMap[layerName].debugPublisher->publish(occupancyGrid);
  }
  layerDataMap[layerName].publishedData = containsData;
}

// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void LayerInspector::updateBounds(double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double* /*min_x*/,
                                  double* /*min_y*/, double* /*max_x*/, double* /*max_y*/)
{}

void LayerInspector::onFootprintChanged() {}

// The method is called when costmap recalculation is required.
// It updates the costmap within its window bounds.
// Inside this method the costmap gradient is generated and is writing directly
// to the resulting costmap master_grid without any merging with previous layers.
void LayerInspector::updateCosts(nav2_costmap_2d::Costmap2D& master_costmap, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    // Do nothing if disabled
    return;
  }
  // LayeredCostmap::updateMap holds the lock for the entire function call.
  auto startTime = std::chrono::steady_clock::now();
  static auto lastPublishTime = std::chrono::steady_clock::now();
  auto timeSinceLastPublish =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - lastPublishTime).count();
  costmap_inspector_msgs::srv::CostmapQuery_Request::SharedPtr queryToWorkOn = nullptr;
  try {
    std::lock_guard<std::mutex> lock(requestDataMutex);
    if (!pendingRequests.empty()) {
      queryToWorkOn = pendingRequests.front();
      pendingRequests.pop();
      if (pendingRequests.size() > 1) {
        RCLCPP_INFO(logger_, "Processing a costmap query request from the queue, remaining queue size: %lu",
                    pendingRequests.size());
      }
    }
  } catch (std::exception& e) {
    RCLCPP_INFO(logger_, "Error during getting next query: %s", e.what());
  }

  try {
    if (queryToWorkOn
        || (paramDebugPublishIndividualLayersPeriodically
            && static_cast<double>(timeSinceLastPublish)
                   > (paramDebugPublishPeriodicallyPeriodSeconds * 1000 /*ms*/))) {
      if (!layered_costmap_) {
        RCLCPP_ERROR(logger_, "Layered costmap pointer is null");
        return;
      }
      std::vector<std::shared_ptr<nav2_costmap_2d::Layer>>* plugins = layered_costmap_->getPlugins();
      if (!plugins) {
        RCLCPP_ERROR(logger_, "Failed to get plugins from layered costmap");
        return;
      }
      nav2_costmap_2d::Costmap2D layerCostmap(master_costmap.getSizeInCellsX(), master_costmap.getSizeInCellsY(),
                                              master_costmap.getResolution(), master_costmap.getOriginX(),
                                              master_costmap.getOriginY());

      CostmapQueryData resultData;
      std::vector<costmap_inspector::Cell> cellsInPolygon;
      currentlyLethalCellsIndex.clear();
      if (queryToWorkOn) {
        if (!prepareRequest(queryToWorkOn, layerCostmap, resultData, cellsInPolygon)) {
          return;
        }
      }

      for (auto& layer : *plugins) {
        if (!layer) {
          RCLCPP_INFO(logger_, "Skipping empty Layer");
          continue;
        }
        std::string layerName = layer->getName();
        if (layerName == name_) {
          continue;
        }
        layerCostmap.resetMap(0, 0, layerCostmap.getSizeInCellsX(), layerCostmap.getSizeInCellsY());
        if (!layer->isEnabled() || !layer->isCurrent()) {
          publishDebugLayer(layerName, layerCostmap, false);
          continue;
        }
        layer->updateCosts(layerCostmap, 0, 0, layerCostmap.getSizeInCellsX(), layerCostmap.getSizeInCellsY());
        if (paramDebugPublishIndividualLayersPeriodically || (queryToWorkOn && paramDebugPublishIndividualLayers)) {
          publishDebugLayer(layerName, layerCostmap, true);
        }
        if (queryToWorkOn) {
          queryLayer(queryToWorkOn, cellsInPolygon, layerName, layerCostmap, resultData);
        }
      }
      // query statistics over all layer results
      if (queryToWorkOn) {
        handleRequestResults(resultData, layerCostmap);
      }
      lastPublishTime = std::chrono::steady_clock::now();
      auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(lastPublishTime - startTime).count();
      if (elapsedTime > 10) {
        RCLCPP_WARN(logger_, "updateCosts called, published individual layers in %ld ms. Took longer than 10ms.",
                    elapsedTime);
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception in updateCosts: %s", e.what());
  }
}

bool LayerInspector::handleRequestResults(CostmapQueryData& resultData, nav2_costmap_2d::Costmap2D& layerCostmap)
{
  resultData.persisted_lethal_layer_names = getLethalLayers();
  if (resultData.persisted_lethal_layer_names.size() > 0) {
    std::unordered_set<std::string> uniqueSources;
    for (const auto& layerInfo : resultData.persisted_lethal_layer_names) {
      uniqueSources.insert(sourceNameMap.contains(layerInfo) ? sourceNameMap[layerInfo] : "N/A");
    }
    resultData.persisted_visible_sources = std::vector(uniqueSources.begin(), uniqueSources.end());
  }
  if (!resultData.most_lethal_layer.empty()) {
    resultData.result = CostmapQueryData::RESULT_LETHAL_DATA;
  } else if (resultData.persisted_lethal_layer_names.size() > 0) {
    resultData.result = CostmapQueryData::RESULT_PERSISTED_LETHAL_DATA;
  } else {
    resultData.result = CostmapQueryData::RESULT_NO_LETHAL_DATA;
  }
  // build pointcloud with lethal points
  sensor_msgs::msg::PointCloud2 lethalPointsCloud;
  geometry_msgs::msg::Point meanPoint;
  buildPointCloud2Message(layerCostmap, lethalPointsCloud, meanPoint);
  lethalPointsPublisher->publish(lethalPointsCloud);

  geometry_msgs::msg::Point meanPointBaseLink;
  try {
    tf2::doTransform(
        meanPoint, meanPointBaseLink,
        tf_->lookupTransform(paramBaseFrame, lethalPointsCloud.header.frame_id, tf2::TimePointZero));
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(logger_, "Failed to transform mean obstacle point from '%s' to '%s': %s",
                 lethalPointsCloud.header.frame_id.c_str(), paramBaseFrame.c_str(), ex.what());
    if (resultPublisher) {
      resultPublisher->publish(resultData);
    }
    return false;
  }
  resultData.mean_object_distance = std::hypot(meanPointBaseLink.x, meanPointBaseLink.y);
  resultData.mean_object_angle = std::atan2(meanPointBaseLink.y, meanPointBaseLink.x);
  if (std::isnan(resultData.mean_object_distance) || std::isnan(resultData.mean_object_angle)) {
    resultData.mean_object_distance = 0.0;
    resultData.mean_object_angle = 0.0;
  }
  // Publish result if publisher is configured
  if (resultPublisher) {
    resultPublisher->publish(resultData);
  }
  return true;
}

bool LayerInspector::prepareRequest(costmap_inspector_msgs::srv::CostmapQuery_Request::SharedPtr request,
                                    nav2_costmap_2d::Costmap2D& layerCostmap, CostmapQueryData& resultData,
                                    std::vector<costmap_inspector::Cell>& cellsInPolygon)
{
  geometry_msgs::msg::PolygonStamped transformedFootprint;
  resultData.header.stamp = rclcpp::Clock().now();
  if (!preparePolygon(request, transformedFootprint)) {
    resultData.result = CostmapQueryData::RESULT_FAILURE;
    return false;
  }

  std::vector<costmap_inspector::Cell> polygonCells;
  // Convert polygon from world coordinates to map cell coordinates
  for (const auto& point : transformedFootprint.polygon.points) {
    unsigned int mx = 0;
    unsigned int my = 0;
    if (layerCostmap.worldToMap(point.x, point.y, mx, my)) {
      polygonCells.emplace_back(mx, my);
    }
  }

  if (polygonCells.empty()) {
    RCLCPP_WARN(logger_, "Polygon has no valid map coordinates");
    resultData.result = CostmapQueryData::RESULT_FAILURE;
    return false;
  }

  // Use the existing scanline algorithm to get all cells within the polygon
  // std::vector<costmap_inspector::Cell> cellsInPolygon;
  costmap_inspector::getCellsOfFilledPolygon(&layerCostmap, polygonCells, cellsInPolygon);
  resultData.num_cells_in_polygon =
      std::max(static_cast<int64_t>(cellsInPolygon.size()), resultData.num_cells_in_polygon);
  return true;
}

bool LayerInspector::buildPointCloud2Message(const nav2_costmap_2d::Costmap2D& layerCostmap,
                                             sensor_msgs::msg::PointCloud2& pointCloud,
                                             geometry_msgs::msg::Point& meanPoint)
{
  meanPoint = geometry_msgs::msg::Point();
  pointCloud.header.frame_id = layered_costmap_->getGlobalFrameID();
  pointCloud.header.stamp = rclcpp::Clock().now();

  sensor_msgs::PointCloud2Modifier modifier(pointCloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(currentlyLethalCellsIndex.size());

  sensor_msgs::PointCloud2Iterator<float> iterX(pointCloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iterY(pointCloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iterZ(pointCloud, "z");

  for (const auto& cellIndex : currentlyLethalCellsIndex) {
    unsigned int mapX = 0;
    unsigned int mapY = 0;
    layerCostmap.indexToCells(static_cast<unsigned int>(cellIndex), mapX, mapY);
    double worldX = 0;
    double worldY = 0;
    layerCostmap.mapToWorld(mapX, mapY, worldX, worldY);

    meanPoint.x += worldX;
    meanPoint.y += worldY;

    *iterX = static_cast<float>(worldX);
    *iterY = static_cast<float>(worldY);
    *iterZ = 0.1f;

    ++iterX;
    ++iterY;
    ++iterZ;
  }
  meanPoint.x /= static_cast<double>(currentlyLethalCellsIndex.size());
  meanPoint.y /= static_cast<double>(currentlyLethalCellsIndex.size());
  return true;
}

void LayerInspector::markLayerLethal(const std::string& layerName,
                                     const std::chrono::steady_clock::time_point currentTime)
{
  layerDataMap[layerName].lastLethalTime = currentTime;
}
std::vector<std::string> LayerInspector::getLethalLayers()
{
  std::vector<std::string> lethalLayers;
  for (const auto& [layerName, layerData] : layerDataMap) {
    if ((static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                 std::chrono::steady_clock::now() - layerDataMap[layerName].lastLethalTime)
                                 .count())
         / 1000.0)
        < paramLethalLayersTimeoutSeconds) {
      lethalLayers.push_back(layerName);
    }
  }
  return lethalLayers;
}

nav_msgs::msg::OccupancyGrid LayerInspector::costmapToOccupancyGrid(const nav2_costmap_2d::Costmap2D& costmap,
                                                                    const std::string& frameID, const bool noData)
{
  nav_msgs::msg::OccupancyGrid grid;

  // Set header
  grid.header.frame_id = frameID;
  grid.header.stamp = rclcpp::Clock().now();

  // Set map metadata
  grid.info.resolution = costmap.getResolution();
  grid.info.width = costmap.getSizeInCellsX();
  grid.info.height = costmap.getSizeInCellsY();

  // Set origin (bottom-left corner)
  grid.info.origin.position.x = costmap.getOriginX();
  grid.info.origin.position.y = costmap.getOriginY();
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;

  // Convert costmap data to occupancy grid data
  // Costmap: 0-252 (free to occupied), 253 (inscribed), 254 (lethal), 255 (unknown)
  // OccupancyGrid: -1 (unknown), 0-100 (free to occupied)

  const unsigned char* data = costmap.getCharMap();
  grid.data.resize(grid.info.width * grid.info.height);

  for (unsigned int i = 0; i < (grid.info.width * grid.info.height); ++i) {
    if (data[i] == nav2_costmap_2d::NO_INFORMATION || noData) {
      grid.data[i] = -1;  // Unknown
    } else if (data[i] == nav2_costmap_2d::LETHAL_OBSTACLE) {
      grid.data[i] = 100;  // Lethal obstacle
    } else if (data[i] == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      grid.data[i] = 99;  // Inscribed obstacle
    } else {
      // Scale 0-252 to 0-98
      grid.data[i] = static_cast<int8_t>((data[i] * 98) / 252);
    }
  }
  return grid;
}

rcl_interfaces::msg::SetParametersResult LayerInspector::dynamicParametersCallback(
    const std::vector<rclcpp::Parameter>& parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  auto node = node_.lock();
  if (!node) {
    result.successful = false;
    result.reason = "Failed to lock node";
    return result;
  }

  for (const auto& parameter : parameters) {
    const auto& paramType = parameter.get_type();
    const auto& paramName = parameter.get_name();

    // Only handle parameters for this layer
    if (paramName.find(name_ + ".") != 0) {
      continue;
    }

    // Extract parameter name without the layer prefix
    std::string localParamName = paramName.substr(name_.length() + 1);

    if (localParamName == PARAM_ENABLED) {
      if (paramType == rclcpp::ParameterType::PARAMETER_BOOL) {
        enabled_ = parameter.as_bool();
        RCLCPP_INFO(logger_, "Parameter '%s' changed to: %s", paramName.c_str(), enabled_ ? "true" : "false");
      }
    } else if (localParamName == PARAM_DEBUG_PUBLISH_CHECKED_FOOTPRINT) {
      if (paramType == rclcpp::ParameterType::PARAMETER_BOOL) {
        paramDebugPublishCheckedFootprint = parameter.as_bool();
        RCLCPP_INFO(logger_, "Parameter '%s' changed to: %s", paramName.c_str(),
                    paramDebugPublishCheckedFootprint ? "true" : "false");
      }
    } else if (localParamName == PARAM_DEBUG_PUBLISH_INDIVIDUAL_LAYERS) {
      if (paramType == rclcpp::ParameterType::PARAMETER_BOOL) {
        bool new_value = parameter.as_bool();
        if (new_value != paramDebugPublishIndividualLayers) {
          paramDebugPublishIndividualLayers = new_value;
          if (paramDebugPublishIndividualLayers) {
            createLayerDebugPublishers(node);
            RCLCPP_INFO(logger_, "Parameter '%s' changed to: true (publishers created)", paramName.c_str());
          } else {
            RCLCPP_INFO(logger_, "Parameter '%s' changed to: false", paramName.c_str());
          }
        }
      }
    } else if (localParamName == PARAM_DEBUG_PUBLISH_INDIVIDUAL_LAYERS_PERIODICALLY) {
      if (paramType == rclcpp::ParameterType::PARAMETER_BOOL) {
        paramDebugPublishIndividualLayersPeriodically = parameter.as_bool();
        RCLCPP_INFO(logger_, "Parameter '%s' changed to: %s", paramName.c_str(),
                    paramDebugPublishIndividualLayersPeriodically ? "true" : "false");
      }
    } else if (localParamName == PARAM_DEBUG_PUBLISH_PERIODICALLY_PERIOD_SECONDS) {
      if (paramType == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        paramDebugPublishPeriodicallyPeriodSeconds = parameter.as_double();
        RCLCPP_INFO(logger_, "Parameter '%s' changed to: %.2f", paramName.c_str(),
                    paramDebugPublishPeriodicallyPeriodSeconds);
      }
    } else if (localParamName == PARAM_LETHAL_LAYERS_TIMEOUT_SECONDS) {
      if (paramType == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        paramLethalLayersTimeoutSeconds = parameter.as_double();
        RCLCPP_INFO(logger_, "Parameter '%s' changed to: %.2f", paramName.c_str(), paramLethalLayersTimeoutSeconds);
      }
    } else {
      RCLCPP_INFO(logger_, "Parameter '%s' unsupported for runtime reconfiguring, ignoring.", paramName.c_str());
    }
  }

  return result;
}

}  // namespace costmap_inspector

// This is the macro allowing a nav2_gradient_costmap_plugin::LayerInspector class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(costmap_inspector::LayerInspector, nav2_costmap_2d::Layer)