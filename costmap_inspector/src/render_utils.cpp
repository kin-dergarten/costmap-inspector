
#include "costmap_inspector/render_utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>

namespace costmap_inspector
{

// get cells

// See https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
void getCellsOfLine(const Cell& start, const Cell& end, std::vector<Cell>& cells)
{
  const int deltaX = abs(end.x - start.x);
  const int stepX = start.x < end.x ? 1 : -1;
  const int deltaY = -abs(end.y - start.y);
  const int stepY = start.y < end.y ? 1 : -1;
  int error = deltaX + deltaY;
  int currentX = start.x;
  int currentY = start.y;

  while (true) {
    cells.emplace_back(currentX, currentY);

    const int error2 = error * 2;

    if (error2 >= deltaY) {
      if (currentX == end.x) {
        break;
      }
      error += deltaY;
      currentX += stepX;
    }

    if (error2 <= deltaX) {
      if (currentY == end.y) {
        break;
      }
      error += deltaX;
      currentY += stepY;
    }
  }
}

// See polygonOutlineCells() in costmap_2d.cpp in the nav2_costmap_2d package
void getCellsOfPolygonOutline(const std::vector<Cell>& corners, std::vector<Cell>& outline)
{
  for (unsigned int i = 0; i < corners.size() - 1; ++i) {
    getCellsOfLine(corners[i], corners[i + 1], outline);
  }

  if (!corners.empty()) {
    // close the polygon
    getCellsOfLine(corners[corners.size() - 1], corners[0], outline);
  }
}

// See convexFillCells() in costmap_2d.cpp in the nav2_costmap_2d package
void getCellsOfFilledPolygon(nav2_costmap_2d::Costmap2D* costmap, const std::vector<Cell>& corners,
                             std::vector<Cell>& cells)
{
  // we need a minimum polygon of a triangle
  if (corners.size() < 3) {
    RCLCPP_WARN(rclcpp::get_logger("costmap_inspector"), "getCellsOfFilledPolygon(): polygon has less than 3 corners");
    return;
  }

  // first get the cells that make up the outline of the polygon
  std::vector<Cell> outline;

  getCellsOfPolygonOutline(corners, outline);

  // quick bubble sort to sort points by x
  unsigned int i = 0;

  while (i < outline.size() - 1) {
    if (outline[i].x > outline[i + 1].x) {
      const Cell swap = outline[i];
      outline[i] = outline[i + 1];
      outline[i + 1] = swap;

      if (i > 0) {
        --i;
      }
    } else {
      ++i;
    }
  }

  // skip cells where x is outside the costmap

  i = 0;

  while (i < outline.size() && outline[i].x < 0) {
    i++;
  }

  // add all cells for each valid x value
  const int minX = std::max(outline[0].x, 0);
  const int maxX = std::min(outline[outline.size() - 1].x, static_cast<int>(costmap->getSizeInCellsX() - 1));

  for (int x = minX; x <= maxX; ++x) {
    if (i >= outline.size() - 1) {
      break;
    }

    // find the min and max y for this x

    Cell minY;
    Cell maxY;

    if (outline[i].y < outline[i + 1].y) {
      minY = outline[i];
      maxY = outline[i + 1];
    } else {
      minY = outline[i + 1];
      maxY = outline[i];
    }

    i += 2;

    while (i < outline.size() && outline[i].x == x) {
      if (outline[i].y < minY.y) {
        minY = outline[i];
      } else if (outline[i].y > maxY.y) {
        maxY = outline[i];
      }
      ++i;
    }

    const int min_y = std::max(minY.y, 0);
    const int max_y = std::min(maxY.y, static_cast<int>(costmap->getSizeInCellsY() - 1));

    // loop though the valid cells in the column
    Cell cell{x, min_y};

    while (cell.y <= max_y) {
      cells.push_back(cell);
      cell.y++;
    }
  }
}

}  // namespace costmap_inspector
