
#include "costmap_inspector/cell.hpp"

namespace costmap_inspector
{

Cell::Cell() : Cell(0, 0) {}
Cell::Cell(const int newX, const int newY) : x(newX), y(newY) {}
Cell::Cell(const unsigned int newX, const unsigned int newY) : x(newX), y(newY) {}

}  // namespace costmap_inspector
