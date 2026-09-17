
#ifndef COSTMAP_INSPECTOR__RENDER_UTILS_HPP_
#define COSTMAP_INSPECTOR__RENDER_UTILS_HPP_

#include "cell.hpp"

#include <vector>

namespace nav2_costmap_2d
{
class Costmap2D;
}  // namespace nav2_costmap_2d

namespace costmap_inspector
{

// get cells

void getCellsOfLine(const Cell& start, const Cell& end, std::vector<Cell>& cells);
void getCellsOfPolygonOutline(const std::vector<Cell>& corners, std::vector<Cell>& outline);
void getCellsOfFilledPolygon(nav2_costmap_2d::Costmap2D* costmap, const std::vector<Cell>& corners,
                             std::vector<Cell>& cells);

}  // namespace costmap_inspector

#endif  // COSTMAP_INSPECTOR__RENDER_UTILS_HPP_
