#ifndef COSTMAP_INSPECTOR__MSG_UTILS_HPP_
#define COSTMAP_INSPECTOR__MSG_UTILS_HPP_
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/time.hpp>
#include <cmath>
#include <string>
namespace costmap_inspector::msg_utils
{
inline geometry_msgs::msg::Point32 point32FromPoint(const geometry_msgs::msg::Point& point)
{
  geometry_msgs::msg::Point32 result;
  result.x = static_cast<float>(point.x);
  result.y = static_cast<float>(point.y);
  result.z = static_cast<float>(point.z);
  return result;
}

inline geometry_msgs::msg::Point32 getPoint32(float x = 0.0F, float y = 0.0F, float z = 0.0F)
{
  geometry_msgs::msg::Point32 result;
  result.x = x;
  result.y = y;
  result.z = z;
  return result;
}

inline geometry_msgs::msg::PoseStamped getPoseStamped(const std::string& frame = "", const rclcpp::Time& stamp = rclcpp::Time(),
                                                      double x = 0.0, double y = 0.0, double yaw = 0.0)
{
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = frame;
  result.header.stamp = stamp;
  result.pose.position.x = x;
  result.pose.position.y = y;
  result.pose.orientation.z = std::sin(yaw / 2.0);
  result.pose.orientation.w = std::cos(yaw / 2.0);
  return result;
}
}  // namespace costmap_inspector::msg_utils
#endif  // COSTMAP_INSPECTOR__MSG_UTILS_HPP_
