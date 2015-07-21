#include <path_costmap_layer/path_costmap_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(path_costmap_layer::PathLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace path_costmap_layer
{
    
PathSubscriber::PathSubscriber(ros::NodeHandle nh, PathLayer* main, std::string topic, unsigned int id) : main_(main), id_(id) {
    subscriber_ = nh.subscribe(topic, 1, &PathSubscriber::incomingPath, this);
}

void PathSubscriber::incomingPath(const nav_msgs::PathConstPtr& message)
{
    main_->incomingPath(message, id_);
}

PathLayer::PathLayer() {}

void PathLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();
  
  std::vector<std::string> topic_list;
  nh.getParam("path_topics", topic_list);
  for(unsigned i=0; i < topic_list.size(); i++) {
      ROS_INFO("TOPIC: %s", topic_list[i].c_str());
      new PathSubscriber(nh, this, topic_list[i], i);
  }

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &PathLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}


void PathLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}


void PathLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void PathLayer::incomingPath(const nav_msgs::PathConstPtr& path, int id)
{
    for(int i=0;i<path->poses.size(); i++){
        double x = path->poses[i].pose.position.x, 
               y = path->poses[i].pose.position.y;
        
        unsigned int mx;
        unsigned int my;
        if(!worldToMap(x, y, mx, my)){
            continue;
        }
        addExtraBounds(x,y,x,y);
        setCost(mx,my, LETHAL_OBSTACLE);
    }
}

void PathLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;
  useExtraBounds(min_x, min_y, max_x, max_y);
}

void PathLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;

  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = getIndex(i, j);
      if (costmap_[index] == NO_INFORMATION)
        continue;
      master_grid.setCost(i, j, costmap_[index]); 
    }
  }
}

} // end namespace
