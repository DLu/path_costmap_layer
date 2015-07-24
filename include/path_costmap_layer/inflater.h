#ifndef INFLATER_H_
#define INFLATER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/inflation_layer.h>
#include <queue>

using costmap_2d::CellData;

namespace path_costmap_layer
{

class Inflater
{
public:
  Inflater();
  
  void inflate(costmap_2d::Costmap2D* costmap, std::vector<int> indexes, double radius);
  
private:
  inline void enqueue(unsigned int index, unsigned int mx, unsigned int my, unsigned int src_x, unsigned int src_y);  
  bool* seen_;
  int seen_size_;
  std::priority_queue<CellData> inflation_queue_;
  
  double inflation_radius_;
  unsigned int cell_inflation_radius_;
  unsigned int cached_cell_inflation_radius_;

  /**
   * @brief  Lookup pre-computed distances
   * @param mx The x coordinate of the current cell
   * @param my The y coordinate of the current cell
   * @param src_x The x coordinate of the source cell
   * @param src_y The y coordinate of the source cell
   * @return
   */
  inline double distanceLookup(int mx, int my, int src_x, int src_y)
  {
    unsigned int dx = abs(mx - src_x);
    unsigned int dy = abs(my - src_y);
    return cached_distances_[dx][dy];
  }
  
  void computeCaches();
  void deleteKernels();
  double** cached_distances_;
};

}  // namespace path_costmap_layer

#endif  // INFLATER_H_
