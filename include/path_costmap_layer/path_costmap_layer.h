#ifndef PATH_COSTMAP_LAYER_H_
#define PATH_COSTMAP_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Path.h>

namespace path_costmap_layer
{
class PathLayer;

class PathSubscriber
{
public:
    PathSubscriber(ros::NodeHandle nh, PathLayer* main, std::string topic, unsigned int id);

    void incomingPath(const nav_msgs::PathConstPtr& message);
private:
    unsigned int id_;
    PathLayer* main_;
    ros::Subscriber subscriber_;
};

class PathLayer : public costmap_2d::CostmapLayer
{
public:
  PathLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  bool isDiscretized()
  {
    return true;
  }

  virtual void matchSize();

  void incomingPath(const nav_msgs::PathConstPtr& message, int id);

private:
  std::vector< ros::Subscriber > subscribers_;

  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
};
}
#endif
