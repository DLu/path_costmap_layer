#include <path_costmap_layer/inflater.h>

namespace path_costmap_layer
{

Inflater::Inflater() : seen_(NULL), cached_distances_(NULL), inflation_radius_(0),
   cell_inflation_radius_(0), cached_cell_inflation_radius_(0)
{
    
}


void Inflater::inflate(costmap_2d::Costmap2D* costmap, std::vector<int> indexes, double radius)
{
  unsigned int size_x = costmap->getSizeInCellsX(), size_y = costmap->getSizeInCellsY();
  if (seen_ == NULL) {
    seen_size_ = size_x * size_y;
    seen_ = new bool[seen_size_];
  }
  else if (seen_size_ != size_x * size_y)
  {
    delete[] seen_;
    seen_size_ = size_x * size_y;
    seen_ = new bool[seen_size_];
  }
  memset(seen_, false, size_x * size_y * sizeof(bool));
  
  inflation_radius_ = radius;
  cell_inflation_radius_ = (unsigned int) (radius / costmap->getResolution() );
  computeCaches();
  
  for(int i=0;i<indexes.size();i++){
      unsigned int mx, my;
      costmap->indexToCells(indexes[i], mx, my);
      enqueue(indexes[i], mx, my, mx, my);
  }
  
  while (!inflation_queue_.empty())
  {
    // get the highest priority cell and pop it off the priority queue
    const CellData& current_cell = inflation_queue_.top();

    unsigned int index = current_cell.index_;
    unsigned int mx = current_cell.x_;
    unsigned int my = current_cell.y_;
    unsigned int sx = current_cell.src_x_;
    unsigned int sy = current_cell.src_y_;
    
    costmap->setCost(mx, my, 100);

    // pop once we have our cell info
    inflation_queue_.pop();

    // attempt to put the neighbors of the current cell onto the queue
    if (mx > 0)
      enqueue(index - 1, mx - 1, my, sx, sy);
    if (my > 0)
      enqueue(index - size_x, mx, my - 1, sx, sy);
    if (mx < size_x - 1)
      enqueue(index + 1, mx + 1, my, sx, sy);
    if (my < size_y - 1)
      enqueue(index + size_x, mx, my + 1, sx, sy);
  }
}

inline void Inflater::enqueue(unsigned int index, unsigned int mx, unsigned int my, unsigned int src_x, unsigned int src_y)
{
  if(seen_[index])
    return;
    
  // we compute our distance table one cell further than the inflation radius dictates so we can make the check below
  double distance = distanceLookup(mx, my, src_x, src_y);

  // we only want to put the cell in the queue if it is within the inflation radius of the obstacle point
  if (distance > cell_inflation_radius_)
    return;

  // push the cell data onto the queue and mark
  seen_[index] = true;
  CellData data(distance, index, mx, my, src_x, src_y);
  inflation_queue_.push(data);
}

void Inflater::computeCaches()
{
  if (cell_inflation_radius_ == 0)
    return;

  // based on the inflation radius... compute distance and cost caches
  if (cell_inflation_radius_ != cached_cell_inflation_radius_)
  {
    deleteKernels();

    cached_distances_ = new double*[cell_inflation_radius_ + 2];

    for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i)
    {
      cached_distances_[i] = new double[cell_inflation_radius_ + 2];
      for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j)
      {
        cached_distances_[i][j] = hypot(i, j);
      }
    }

    cached_cell_inflation_radius_ = cell_inflation_radius_;
  }
}

void Inflater::deleteKernels()
{
  if (cached_distances_ != NULL)
  {
    for (unsigned int i = 0; i <= cached_cell_inflation_radius_ + 1; ++i)
    {
      if (cached_distances_[i])
        delete[] cached_distances_[i];
    }
    if (cached_distances_)
      delete[] cached_distances_;
    cached_distances_ = NULL;
  }
}

}  // namespace path_costmap_layer
