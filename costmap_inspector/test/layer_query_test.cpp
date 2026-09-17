#include "costmap_inspector/costmap_inspector.hpp"

#include "costmap_inspector/msg_utils.hpp"
#include <gtest/gtest.h>
#include <memory>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include "costmap_inspector/cell.hpp"
#include "costmap_inspector/render_utils.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/buffer.h>

using namespace costmap_inspector;
using nav2_costmap_2d::Costmap2D;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

using msg_utils::getPoint32;
using msg_utils::getPoseStamped;

// Testable wrapper that exposes protected/private methods for testing
class TestableLayerInspector : public LayerInspector
{
 public:
  // Expose private methods for testing
  void testQueryLayer(costmap_inspector_msgs::srv::CostmapQuery_Request::SharedPtr request,
                      const std::vector<costmap_inspector::Cell> polygonCells, const std::string& layerName,
                      nav2_costmap_2d::Costmap2D& layerCostmap, costmap_inspector_msgs::msg::CostmapQueryData& resultData)
  {
    queryLayer(request, polygonCells, layerName, layerCostmap, resultData);
  }

  void testMarkLayerLethal(const std::string& layerName, const std::chrono::steady_clock::time_point currentTime)
  {
    markLayerLethal(layerName, currentTime);
  }

  std::vector<std::string> testGetLethalLayers() { return getLethalLayers(); }

  bool testPreparePolygon(costmap_inspector_msgs::srv::CostmapQuery_Request::SharedPtr request,
                          geometry_msgs::msg::PolygonStamped& transformedFootprint)
  {
    return preparePolygon(request, transformedFootprint);
  }

  bool testBuildPointCloud2Message(const nav2_costmap_2d::Costmap2D& layerCostmap,
                                   sensor_msgs::msg::PointCloud2& pointCloud, geometry_msgs::msg::Point& meanPoint)
  {
    return buildPointCloud2Message(layerCostmap, pointCloud, meanPoint);
  }

  // Provide access to test the currently lethal cells
  const std::unordered_set<uint64_t>& getCurrentlyLethalCellsIndex() const { return currentlyLethalCellsIndex; }

  nav_msgs::msg::OccupancyGrid testCostmapToOccupancyGrid(const nav2_costmap_2d::Costmap2D& costmap,
                                                          const std::string& frameID, bool noData)
  {
    return costmapToOccupancyGrid(costmap, frameID, noData);
  }

  rcl_interfaces::msg::SetParametersResult testDynamicParametersCallback(
      const std::vector<rclcpp::Parameter>& parameters)
  {
    return dynamicParametersCallback(parameters);
  }

  // Accessors for verifying parameter state
  bool getParamDebugPublishCheckedFootprint() const { return paramDebugPublishCheckedFootprint; }
  bool getParamDebugPublishIndividualLayers() const { return paramDebugPublishIndividualLayers; }
  bool getParamDebugPublishIndividualLayersPeriodically() const
  {
    return paramDebugPublishIndividualLayersPeriodically;
  }
  double getParamDebugPublishPeriodicallyPeriodSeconds() const { return paramDebugPublishPeriodicallyPeriodSeconds; }
  double getParamLethalLayersTimeoutSeconds() const { return paramLethalLayersTimeoutSeconds; }
};

// Mock layer for testing
class MockCostmapLayer : public nav2_costmap_2d::Layer
{
 public:
  MockCostmapLayer() : enabled_(true), current_(true) {}

  void updateBounds(double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double* /*min_x*/, double* /*min_y*/,
                    double* /*max_x*/, double* /*max_y*/) override
  {}

  void updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override
  {
    // Copy our test data to the master grid
    if (testCostmap) {
      unsigned char* master_array = master_grid.getCharMap();
      unsigned char* test_array = testCostmap->getCharMap();
      for (int j = min_j; j < max_j; j++) {
        for (int i = min_i; i < max_i; i++) {
          unsigned int index = master_grid.getIndex(i, j);
          master_array[index] = test_array[index];
        }
      }
    }
  }

  void reset() override {}
  void onFootprintChanged() override {}
  bool isClearable() override { return false; }

  void setEnabled(bool enabled) { enabled_ = enabled; }
  bool isEnabled() { return enabled_; }
  void setCurrent(bool current) { current_ = current; }
  bool isCurrent() { return current_; }

  void setTestCostmap(std::shared_ptr<Costmap2D> costmap) { testCostmap = costmap; }

 private:
  bool enabled_;
  bool current_;
  std::shared_ptr<Costmap2D> testCostmap;
};

class LayerQueryTest : public ::testing::Test
{
 protected:
  void SetUp() override
  {
    // Initialize ROS if not already initialized
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    // Create lifecycle node for the inspector
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testCostmapinspector");

    // Initialize tf_buffer with the node's clock
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());

    // Create layered costmap
    layeredCostmap = std::make_shared<nav2_costmap_2d::LayeredCostmap>("test_frame", false, false);

    // Create callback group for the inspector
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Create inspector
    inspector = std::make_shared<TestableLayerInspector>();
    inspector->initialize(layeredCostmap.get(), "test_inspector", tf_buffer_.get(), node_, callback_group_);

    // Create test costmap (10x10 cells, 0.1m resolution, origin at 0,0)
    testCostmap = std::make_shared<Costmap2D>(10, 10, 0.1, 0.0, 0.0);

    // Create test request
    testRequest = std::make_shared<costmap_inspector_msgs::srv::CostmapQuery::Request>();
    testRequest->use_costmap_footprint = false;
    testRequest->persist_result = false;
  }

  void TearDown() override
  {
    inspector.reset();
    layeredCostmap.reset();
    testCostmap.reset();
    tf_buffer_.reset();
    callback_group_.reset();
    node_.reset();
  }

  // Helper function to create a rectangular polygon of cells
  std::vector<costmap_inspector::Cell> createRectangleCells(unsigned int x_start, unsigned int y_start,
                                                          unsigned int width, unsigned int height)
  {
    std::vector<costmap_inspector::Cell> cells;
    for (unsigned int x = x_start; x < x_start + width; ++x) {
      for (unsigned int y = y_start; y < y_start + height; ++y) {
        cells.emplace_back(x, y);
      }
    }
    return cells;
  }

  // Helper function to set lethal obstacles in the costmap
  void setLethalObstacles(const std::vector<std::pair<unsigned int, unsigned int>>& positions)
  {
    for (const auto& [x, y] : positions) {
      testCostmap->setCost(x, y, LETHAL_OBSTACLE);
    }
  }

  // Helper to clear costmap
  void clearCostmap() { testCostmap->resetMap(0, 0, testCostmap->getSizeInCellsX(), testCostmap->getSizeInCellsY()); }

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::shared_ptr<nav2_costmap_2d::LayeredCostmap> layeredCostmap;
  std::shared_ptr<TestableLayerInspector> inspector;
  std::shared_ptr<Costmap2D> testCostmap;
  std::shared_ptr<costmap_inspector_msgs::srv::CostmapQuery::Request> testRequest;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

TEST_F(LayerQueryTest, QueryLayerNoLethalObstacles)
{
  clearCostmap();

  // Create result data
  costmap_inspector_msgs::msg::CostmapQueryData resultData;
  resultData.num_lethal_cells_in_most_lethal_layer = 0;
  resultData.most_lethal_layer = "";

  // Create polygon cells
  auto polygonCells = createRectangleCells(2, 2, 2, 2);

  // Call the actual business logic!
  inspector->testQueryLayer(testRequest, polygonCells, "test_layer", *testCostmap, resultData);

  // Verify results
  EXPECT_EQ(resultData.layer_info.size(), 1) << "Should have info for one layer";
  EXPECT_EQ(resultData.layer_info[0].layer_name, "test_layer");
  EXPECT_EQ(resultData.layer_info[0].num_lethal, 0) << "No lethal obstacles in clear costmap";
  EXPECT_EQ(resultData.most_lethal_layer, "") << "No most lethal layer when no obstacles";
}

TEST_F(LayerQueryTest, QueryLayerSomeLethalObstacles)
{
  clearCostmap();
  setLethalObstacles({{2, 2}, {3, 2}, {2, 3}});

  costmap_inspector_msgs::msg::CostmapQueryData resultData;
  resultData.num_lethal_cells_in_most_lethal_layer = 0;

  auto polygonCells = createRectangleCells(2, 2, 2, 2);

  inspector->testQueryLayer(testRequest, polygonCells, "test_layer", *testCostmap, resultData);

  EXPECT_EQ(resultData.layer_info.size(), 1);
  EXPECT_EQ(resultData.layer_info[0].num_lethal, 3) << "Should find exactly 3 lethal obstacles";
  EXPECT_EQ(resultData.most_lethal_layer, "test_layer") << "test_layer should be marked as most lethal";
  EXPECT_EQ(resultData.num_lethal_cells_in_most_lethal_layer, 3);
}

TEST_F(LayerQueryTest, QueryLayerAllCellsLethal)
{
  clearCostmap();
  setLethalObstacles({{2, 2}, {3, 2}, {2, 3}, {3, 3}});

  costmap_inspector_msgs::msg::CostmapQueryData resultData;
  resultData.num_lethal_cells_in_most_lethal_layer = 0;

  auto polygonCells = createRectangleCells(2, 2, 2, 2);

  inspector->testQueryLayer(testRequest, polygonCells, "test_layer", *testCostmap, resultData);

  EXPECT_EQ(resultData.layer_info[0].num_lethal, 4) << "All 4 cells should be lethal";
  EXPECT_EQ(resultData.most_lethal_layer, "test_layer");

  // Verify the lethal cell indices were tracked
  EXPECT_EQ(inspector->getCurrentlyLethalCellsIndex().size(), 4) << "Should track all 4 lethal cell indices";
}

TEST_F(LayerQueryTest, QueryLayerObstaclesOutsidePolygon)
{
  clearCostmap();
  setLethalObstacles({{0, 0}, {9, 9}, {5, 5}});

  costmap_inspector_msgs::msg::CostmapQueryData resultData;
  auto polygonCells = createRectangleCells(2, 2, 2, 2);

  inspector->testQueryLayer(testRequest, polygonCells, "test_layer", *testCostmap, resultData);

  EXPECT_EQ(resultData.layer_info[0].num_lethal, 0) << "No lethal obstacles when they're outside the polygon";
}

TEST_F(LayerQueryTest, QueryLayerTracksMostLethalLayer)
{
  // First layer with 2 obstacles
  clearCostmap();
  setLethalObstacles({{2, 2}, {3, 2}});

  costmap_inspector_msgs::msg::CostmapQueryData resultData;
  resultData.num_lethal_cells_in_most_lethal_layer = 0;
  auto polygonCells = createRectangleCells(2, 2, 2, 2);

  inspector->testQueryLayer(testRequest, polygonCells, "layer1", *testCostmap, resultData);
  EXPECT_EQ(resultData.most_lethal_layer, "layer1");
  EXPECT_EQ(resultData.num_lethal_cells_in_most_lethal_layer, 2);

  // Second layer with 3 obstacles - should become most lethal
  clearCostmap();
  setLethalObstacles({{2, 2}, {3, 2}, {2, 3}});

  inspector->testQueryLayer(testRequest, polygonCells, "layer2", *testCostmap, resultData);
  EXPECT_EQ(resultData.most_lethal_layer, "layer2") << "layer2 should become most lethal";
  EXPECT_EQ(resultData.num_lethal_cells_in_most_lethal_layer, 3);

  // Third layer with 1 obstacle - should not change most lethal
  clearCostmap();
  setLethalObstacles({{2, 2}});

  inspector->testQueryLayer(testRequest, polygonCells, "layer3", *testCostmap, resultData);
  EXPECT_EQ(resultData.most_lethal_layer, "layer2") << "layer2 should still be most lethal";
  EXPECT_EQ(resultData.num_lethal_cells_in_most_lethal_layer, 3);
}

TEST_F(LayerQueryTest, LethalLayerTimeoutTracking)
{
  auto now = std::chrono::steady_clock::now();

  // Mark layer as lethal
  inspector->testMarkLayerLethal("layer1", now);

  // Should be in lethal layers (within timeout)
  auto lethalLayers = inspector->testGetLethalLayers();
  EXPECT_EQ(lethalLayers.size(), 1);
  EXPECT_EQ(lethalLayers[0], "layer1");

  // Mark with an old timestamp (simulating timeout)
  auto oldTime = now - std::chrono::seconds(100);
  inspector->testMarkLayerLethal("layer2", oldTime);

  lethalLayers = inspector->testGetLethalLayers();
  // layer2 should have timed out (default timeout is 1.0 seconds)
  EXPECT_EQ(lethalLayers.size(), 1) << "Only layer1 should be within timeout";
}

TEST_F(LayerQueryTest, QueryLayerInscribedObstaclesNotCounted)
{
  clearCostmap();
  testCostmap->setCost(2, 2, LETHAL_OBSTACLE);
  testCostmap->setCost(3, 2, INSCRIBED_INFLATED_OBSTACLE);
  testCostmap->setCost(2, 3, LETHAL_OBSTACLE);

  costmap_inspector_msgs::msg::CostmapQueryData resultData;
  auto polygonCells = createRectangleCells(2, 2, 2, 2);

  inspector->testQueryLayer(testRequest, polygonCells, "test_layer", *testCostmap, resultData);

  EXPECT_EQ(resultData.layer_info[0].num_lethal, 2) << "Only LETHAL_OBSTACLE cells should be counted, not INSCRIBED";
}

TEST_F(LayerQueryTest, QueryLayerUnknownCellsNotCounted)
{
  clearCostmap();
  testCostmap->setCost(2, 2, LETHAL_OBSTACLE);
  testCostmap->setCost(3, 2, NO_INFORMATION);
  testCostmap->setCost(2, 3, LETHAL_OBSTACLE);

  costmap_inspector_msgs::msg::CostmapQueryData resultData;
  auto polygonCells = createRectangleCells(2, 2, 2, 2);

  inspector->testQueryLayer(testRequest, polygonCells, "test_layer", *testCostmap, resultData);

  EXPECT_EQ(resultData.layer_info[0].num_lethal, 2)
      << "Only LETHAL_OBSTACLE cells should be counted, not NO_INFORMATION";
}

TEST_F(LayerQueryTest, BuildPointCloud2FromLethalCells)
{
  clearCostmap();
  setLethalObstacles({{2, 2}, {3, 3}, {5, 5}});

  // First query to populate lethal cells
  costmap_inspector_msgs::msg::CostmapQueryData resultData;
  auto polygonCells = createRectangleCells(0, 0, 10, 10);
  inspector->testQueryLayer(testRequest, polygonCells, "test_layer", *testCostmap, resultData);

  // Now build point cloud
  sensor_msgs::msg::PointCloud2 cloud;
  geometry_msgs::msg::Point meanPoint;
  bool success = inspector->testBuildPointCloud2Message(*testCostmap, cloud, meanPoint);

  EXPECT_TRUE(success);
  EXPECT_EQ(cloud.header.frame_id, "test_frame");
  EXPECT_EQ(cloud.width * cloud.height, 3u) << "Point cloud should have three points";
  EXPECT_TRUE(meanPoint.x > 0 && meanPoint.y > 0) << "Mean point should be in positive quadrant";
}

TEST_F(LayerQueryTest, BuildLayerInfoStructure)
{
  std::string layer_name = "test_layer";
  int64_t num_lethal = 42;

  auto layer_info = LayerInspector::buildLayerInfo(layer_name, num_lethal);

  EXPECT_EQ(layer_info.layer_name, layer_name);
  EXPECT_EQ(layer_info.num_lethal, num_lethal);
}

TEST_F(LayerQueryTest, QueryLayerEmptyPolygon)
{
  clearCostmap();
  setLethalObstacles({{5, 5}});

  costmap_inspector_msgs::msg::CostmapQueryData resultData;
  std::vector<costmap_inspector::Cell> empty_polygon;

  inspector->testQueryLayer(testRequest, empty_polygon, "test_layer", *testCostmap, resultData);

  EXPECT_EQ(resultData.layer_info[0].num_lethal, 0) << "Empty polygon should yield no obstacles";
}

TEST_F(LayerQueryTest, QueryLayerSingleCell)
{
  clearCostmap();
  setLethalObstacles({{5, 5}});

  costmap_inspector_msgs::msg::CostmapQueryData resultData;
  std::vector<costmap_inspector::Cell> single_cell = {{5, 5}};

  inspector->testQueryLayer(testRequest, single_cell, "test_layer", *testCostmap, resultData);

  EXPECT_EQ(resultData.layer_info[0].num_lethal, 1) << "Should find single lethal obstacle";
}

TEST_F(LayerQueryTest, QueryLayerBoundaryCells)
{
  clearCostmap();
  setLethalObstacles({{0, 0}, {9, 0}, {0, 9}, {9, 9}});

  costmap_inspector_msgs::msg::CostmapQueryData resultData;

  std::vector<costmap_inspector::Cell> corner_cells = {{0, 0}, {9, 0}, {0, 9}, {9, 9}};
  inspector->testQueryLayer(testRequest, corner_cells, "test_layer", *testCostmap, resultData);

  EXPECT_EQ(resultData.layer_info[0].num_lethal, 4) << "Should handle all boundary cells correctly";
}

TEST_F(LayerQueryTest, QueryLayerAccumulatesLethalCells)
{
  clearCostmap();
  setLethalObstacles({{2, 2}, {3, 3}});

  costmap_inspector_msgs::msg::CostmapQueryData resultData;
  auto polygonCells = createRectangleCells(0, 0, 5, 5);

  // First query
  inspector->testQueryLayer(testRequest, polygonCells, "layer1", *testCostmap, resultData);
  auto lethal_count_1 = inspector->getCurrentlyLethalCellsIndex().size();

  // Add more obstacles
  setLethalObstacles({{4, 4}, {1, 1}});
  inspector->testQueryLayer(testRequest, polygonCells, "layer2", *testCostmap, resultData);
  auto lethal_count_2 = inspector->getCurrentlyLethalCellsIndex().size();

  // The second query should accumulate more lethal cells
  EXPECT_GT(lethal_count_2, lethal_count_1) << "Should accumulate lethal cells across queries";
  EXPECT_EQ(lethal_count_2, 4) << "Should have all 4 unique lethal cells";
}

// ============================================================================
// PreparePolygon tests
// ============================================================================

TEST_F(LayerQueryTest, PreparePolygonSameFrameNoTransformNeeded)
{
  // Create a request with polygon in "test_frame" (same as costmap frame)
  auto request = std::make_shared<costmap_inspector_msgs::srv::CostmapQuery::Request>();
  request->use_costmap_footprint = false;
  request->persist_result = false;

  // Set the pose in the same frame as the costmap ("test_frame")
  request->footprint_pose = getPoseStamped("test_frame");

  // Set the footprint polygon
  request->footprint.points = {getPoint32(0.2, 0.2), getPoint32(0.8, 0.2), getPoint32(0.8, 0.8), getPoint32(0.2, 0.8)};

  geometry_msgs::msg::PolygonStamped transformedFootprint;
  bool result = inspector->testPreparePolygon(request, transformedFootprint);

  EXPECT_TRUE(result) << "preparePolygon should succeed when polygon is in the same frame";
  EXPECT_EQ(transformedFootprint.polygon.points.size(), 4u) << "Should have 4 polygon points";
}

TEST_F(LayerQueryTest, PreparePolygonAppliesPoseTranslation)
{
  auto request = std::make_shared<costmap_inspector_msgs::srv::CostmapQuery::Request>();
  request->use_costmap_footprint = false;
  request->persist_result = false;

  // Set the pose in the same frame as the costmap ("test_frame")
  request->footprint_pose = getPoseStamped("test_frame", rclcpp::Time(), 0.5, 0.5);
  // Create a unit square centered at origin
  request->footprint.points = {getPoint32(-0.1, -0.1), getPoint32(0.1, -0.1), getPoint32(0.1, 0.1),
                               getPoint32(-0.1, 0.1)};

  geometry_msgs::msg::PolygonStamped transformedFootprint;
  bool result = inspector->testPreparePolygon(request, transformedFootprint);

  EXPECT_TRUE(result);
  ASSERT_EQ(transformedFootprint.polygon.points.size(), 4u);

  // The polygon should be translated: center should be around (0.5, 0.5)
  for (const auto& point : transformedFootprint.polygon.points) {
    EXPECT_NEAR(point.x, 0.5, 0.2) << "X coordinate should be near 0.5 after translation";
    EXPECT_NEAR(point.y, 0.5, 0.2) << "Y coordinate should be near 0.5 after translation";
  }
}

TEST_F(LayerQueryTest, PreparePolygonDifferentFrameFailsWithoutTf)
{
  auto request = std::make_shared<costmap_inspector_msgs::srv::CostmapQuery::Request>();
  request->use_costmap_footprint = false;
  request->persist_result = false;

  // Set the pose in the same frame as the costmap ("test_frame")
  request->footprint_pose = getPoseStamped("unknown_frame");
  request->footprint.points = {getPoint32(0.5, 0.5)};

  geometry_msgs::msg::PolygonStamped transformedFootprint;
  bool result = inspector->testPreparePolygon(request, transformedFootprint);

  EXPECT_FALSE(result) << "preparePolygon should fail when frame transform is not available";
}

// ============================================================================
// CostmapToOccupancyGrid tests
// ============================================================================

TEST_F(LayerQueryTest, CostmapToOccupancyGridMetadata)
{
  // Create a 5x5 costmap
  Costmap2D costmap(5, 5, 0.05, 1.0, 2.0);

  auto grid = inspector->testCostmapToOccupancyGrid(costmap, "map_frame", false);

  EXPECT_EQ(grid.header.frame_id, "map_frame");
  EXPECT_EQ(grid.info.width, 5u);
  EXPECT_EQ(grid.info.height, 5u);
  EXPECT_NEAR(grid.info.resolution, 0.05, 1e-5);
  EXPECT_DOUBLE_EQ(grid.info.origin.position.x, 1.0);
  EXPECT_DOUBLE_EQ(grid.info.origin.position.y, 2.0);
  EXPECT_DOUBLE_EQ(grid.info.origin.orientation.w, 1.0);
  EXPECT_EQ(grid.data.size(), 25u);
}

TEST_F(LayerQueryTest, CostmapToOccupancyGridLethalMapsTo100)
{
  Costmap2D costmap(5, 5, 0.1, 0.0, 0.0);
  costmap.setCost(2, 2, LETHAL_OBSTACLE);

  auto grid = inspector->testCostmapToOccupancyGrid(costmap, "test_frame", false);

  unsigned int index = costmap.getIndex(2, 2);
  EXPECT_EQ(grid.data[index], 100) << "LETHAL_OBSTACLE should map to 100 in occupancy grid";
}

TEST_F(LayerQueryTest, CostmapToOccupancyGridInscribedMapsTo99)
{
  Costmap2D costmap(5, 5, 0.1, 0.0, 0.0);
  costmap.setCost(3, 3, INSCRIBED_INFLATED_OBSTACLE);

  auto grid = inspector->testCostmapToOccupancyGrid(costmap, "test_frame", false);

  unsigned int index = costmap.getIndex(3, 3);
  EXPECT_EQ(grid.data[index], 99) << "INSCRIBED_INFLATED_OBSTACLE should map to 99";
}

TEST_F(LayerQueryTest, CostmapToOccupancyGridNoInformationMapsToNeg1)
{
  Costmap2D costmap(5, 5, 0.1, 0.0, 0.0);
  costmap.setCost(1, 1, NO_INFORMATION);

  auto grid = inspector->testCostmapToOccupancyGrid(costmap, "test_frame", false);

  unsigned int index = costmap.getIndex(1, 1);
  EXPECT_EQ(grid.data[index], -1) << "NO_INFORMATION should map to -1 (unknown)";
}

TEST_F(LayerQueryTest, CostmapToOccupancyGridFreeCellMapsToZero)
{
  Costmap2D costmap(5, 5, 0.1, 0.0, 0.0);
  // Default value after construction is 0 (FREE_SPACE)

  auto grid = inspector->testCostmapToOccupancyGrid(costmap, "test_frame", false);

  unsigned int index = costmap.getIndex(0, 0);
  EXPECT_EQ(grid.data[index], 0) << "Free space (cost 0) should map to 0";
}

TEST_F(LayerQueryTest, CostmapToOccupancyGridIntermediateValuesScaled)
{
  Costmap2D costmap(5, 5, 0.1, 0.0, 0.0);
  // Set an intermediate cost value (126 is roughly middle of 0-252 range)
  costmap.setCost(2, 2, 126);

  auto grid = inspector->testCostmapToOccupancyGrid(costmap, "test_frame", false);

  unsigned int index = costmap.getIndex(2, 2);
  // Expected: (126 * 98) / 252 = 49
  EXPECT_EQ(grid.data[index], 49) << "Intermediate cost 126 should scale to ~49";
}

TEST_F(LayerQueryTest, CostmapToOccupancyGridNoDataFlagSetsAllUnknown)
{
  Costmap2D costmap(5, 5, 0.1, 0.0, 0.0);
  costmap.setCost(0, 0, LETHAL_OBSTACLE);
  costmap.setCost(1, 1, INSCRIBED_INFLATED_OBSTACLE);
  costmap.setCost(2, 2, 100);

  auto grid = inspector->testCostmapToOccupancyGrid(costmap, "test_frame", true);

  // When noData is true, ALL cells should be -1
  for (size_t i = 0; i < grid.data.size(); ++i) {
    EXPECT_EQ(grid.data[i], -1) << "All cells should be -1 (unknown) when noData=true, failed at index " << i;
  }
}

// ============================================================================
// Dynamic Parameter Callback tests
// ============================================================================

TEST_F(LayerQueryTest, DynamicParameterCallbackEnabled)
{
  // The parameter name must have the layer prefix "test_inspector."
  std::vector<rclcpp::Parameter> params;
  params.emplace_back("test_inspector.enabled", false);

  auto result = inspector->testDynamicParametersCallback(params);

  EXPECT_TRUE(result.successful);
  // After setting enabled to false, the layer should report not enabled
  EXPECT_FALSE(inspector->isEnabled()) << "Layer should be disabled after parameter change";
}

TEST_F(LayerQueryTest, DynamicParameterCallbackReEnablesLayer)
{
  // First disable
  std::vector<rclcpp::Parameter> params;
  params.emplace_back("test_inspector.enabled", false);
  inspector->testDynamicParametersCallback(params);
  EXPECT_FALSE(inspector->isEnabled());

  // Then re-enable
  params.clear();
  params.emplace_back("test_inspector.enabled", true);
  inspector->testDynamicParametersCallback(params);
  EXPECT_TRUE(inspector->isEnabled()) << "Layer should be re-enabled after parameter change";
}

TEST_F(LayerQueryTest, DynamicParameterCallbackPublishIncomingFootprint)
{
  std::vector<rclcpp::Parameter> params;
  params.emplace_back("test_inspector.debug.publish_checked_footprint", true);

  auto result = inspector->testDynamicParametersCallback(params);

  EXPECT_TRUE(result.successful);
  EXPECT_TRUE(inspector->getParamDebugPublishCheckedFootprint());
}

TEST_F(LayerQueryTest, DynamicParameterCallbackPublishIndividualLayersPeriodically)
{
  std::vector<rclcpp::Parameter> params;
  params.emplace_back("test_inspector.debug.publish_individual_layers_periodically", true);

  auto result = inspector->testDynamicParametersCallback(params);

  EXPECT_TRUE(result.successful);
  EXPECT_TRUE(inspector->getParamDebugPublishIndividualLayersPeriodically());
}

TEST_F(LayerQueryTest, DynamicParameterCallbackPublishPeriodSeconds)
{
  std::vector<rclcpp::Parameter> params;
  params.emplace_back("test_inspector.debug.publish_periodically_period_seconds", 5.0);

  auto result = inspector->testDynamicParametersCallback(params);

  EXPECT_TRUE(result.successful);
  EXPECT_DOUBLE_EQ(inspector->getParamDebugPublishPeriodicallyPeriodSeconds(), 5.0);
}

TEST_F(LayerQueryTest, DynamicParameterCallbackLethalLayersTimeout)
{
  std::vector<rclcpp::Parameter> params;
  params.emplace_back("test_inspector.lethal_layers_timeout_seconds", 3.5);

  auto result = inspector->testDynamicParametersCallback(params);

  EXPECT_TRUE(result.successful);
  EXPECT_DOUBLE_EQ(inspector->getParamLethalLayersTimeoutSeconds(), 3.5);
}

TEST_F(LayerQueryTest, DynamicParameterCallbackIgnoresOtherLayerParams)
{
  // Parameters not prefixed with "test_inspector." should be ignored
  std::vector<rclcpp::Parameter> params;
  params.emplace_back("other_layer.enabled", false);

  auto result = inspector->testDynamicParametersCallback(params);

  EXPECT_TRUE(result.successful);
  // The inspector's enabled state should remain unchanged (still true from initialization)
  EXPECT_TRUE(inspector->isEnabled()) << "Should not change state for parameters of other layers";
}

TEST_F(LayerQueryTest, DynamicParameterCallbackMultipleParamsAtOnce)
{
  std::vector<rclcpp::Parameter> params;
  params.emplace_back("test_inspector.debug.publish_checked_footprint", true);
  params.emplace_back("test_inspector.debug.publish_periodically_period_seconds", 2.0);
  params.emplace_back("test_inspector.lethal_layers_timeout_seconds", 10.0);

  auto result = inspector->testDynamicParametersCallback(params);

  EXPECT_TRUE(result.successful);
  EXPECT_TRUE(inspector->getParamDebugPublishCheckedFootprint());
  EXPECT_DOUBLE_EQ(inspector->getParamDebugPublishPeriodicallyPeriodSeconds(), 2.0);
  EXPECT_DOUBLE_EQ(inspector->getParamLethalLayersTimeoutSeconds(), 10.0);
}
