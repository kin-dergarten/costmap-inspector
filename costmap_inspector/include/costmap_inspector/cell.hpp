
#ifndef COSTMAP_INSPECTOR__CELL_HPP_
#define COSTMAP_INSPECTOR__CELL_HPP_

namespace costmap_inspector
{

class Cell
{
 public:
  int x;
  int y;

  Cell();
  Cell(int newX, int newY);
  Cell(unsigned int newX, unsigned int newY);
};

}  // namespace costmap_inspector

#endif  // COSTMAP_INSPECTOR__MATH_HPP_
