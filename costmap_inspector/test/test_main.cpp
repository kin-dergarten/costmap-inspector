#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  // Initialize ROS before running tests
  rclcpp::init(argc, argv);

  // Initialize Google Test
  testing::InitGoogleTest(&argc, argv);

  // Run all tests
  int result = RUN_ALL_TESTS();

  // Shutdown ROS
  rclcpp::shutdown();

  return result;
}
