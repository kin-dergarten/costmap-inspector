#ifndef COSTMAP_INSPECTOR__CONSTANTS_HPP_
#define COSTMAP_INSPECTOR__CONSTANTS_HPP_
namespace costmap_inspector::topic
{
inline constexpr char DEFAULT_QUERY_RESULT_TOPIC[] = "costmap/inspector_layer/query_result";
inline constexpr char DEFAULT_LETHAL_POINTS_TOPIC[] = "costmap/inspector_layer/lethal_points";
}  // namespace costmap_inspector::topic
namespace costmap_inspector::service
{
inline constexpr char DEFAULT_QUERY_SERVICE[] = "costmap/inspector_layer/polygon_query";
}  // namespace costmap_inspector::service
namespace costmap_inspector::frame
{
inline constexpr char DEFAULT_BASE_FRAME[] = "base_link";
}  // namespace costmap_inspector::frame
#endif  // COSTMAP_INSPECTOR__CONSTANTS_HPP_
