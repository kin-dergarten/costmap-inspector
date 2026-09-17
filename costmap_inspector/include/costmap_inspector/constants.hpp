#ifndef COSTMAP_INSPECTOR__CONSTANTS_HPP_
#define COSTMAP_INSPECTOR__CONSTANTS_HPP_
namespace costmap_inspector::topic
{
inline constexpr char COSTMAP_QUERY_RESULT[] = "local_costmap/inspector_layer/query_result";
inline constexpr char COSTMAP_LETHAL_POINTS[] = "local_costmap/inspector_layer/lethal_points";
}  // namespace costmap_inspector::topic
namespace costmap_inspector::service
{
inline constexpr char COSTMAP_INSPECTOR_QUERY[] = "local_costmap/inspector_layer/polygon_query";
}  // namespace costmap_inspector::service
namespace costmap_inspector::frame
{
inline constexpr char BASE_LINK[] = "base_link";
}  // namespace costmap_inspector::frame
#endif  // COSTMAP_INSPECTOR__CONSTANTS_HPP_
