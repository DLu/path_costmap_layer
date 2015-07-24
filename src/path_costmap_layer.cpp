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

PathLayer::PathLayer() : needsUpdate(false), old_x0_(1), old_x1_(0) {}

void PathLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();

  int i_cost;
  nh.param("path_cost", i_cost, 100);
  cost_ = i_cost;
  
  nh.param("radius", radius_, 0.5);
  
  std::vector<std::string> topic_list;
  nh.getParam("path_topics", topic_list);
  for(unsigned i=0; i < topic_list.size(); i++) {
      ROS_INFO("TOPIC: %s", topic_list[i].c_str());
      new PathSubscriber(nh, this, topic_list[i], i);
  }
  paths_.resize( topic_list.size() );

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
    if(paths_[id].header.stamp == path->header.stamp)
        return;
    paths_[id] = *path;
    drawAllPaths();
}

void PathLayer::drawAllPaths()
{
    if(old_x0_ < old_x1_){
        unsigned int mx0, mx1, my0, my1;
        worldToMap(old_x0_, old_y0_, mx0, my0);
        worldToMap(old_x1_, old_y1_, mx1, my1);
        resetMap(mx0, my0, mx1, my1);
    }
    
    std::vector<int> pixels;
    
    double min_x = 1e6, min_y = 1e6, max_x=-1e6, max_y = -1e6;

    for(int j=0;j<paths_.size();j++){
        
        for(int i=0;i<paths_[j].poses.size(); i++){
            double x = paths_[j].poses[i].pose.position.x, 
                   y = paths_[j].poses[i].pose.position.y;
            
            unsigned int mx;
            unsigned int my;
            if(!worldToMap(x, y, mx, my)){
                continue;
            }
            
            min_x = std::min(x-radius_, min_x);
            max_x = std::max(x+radius_, max_x);
            min_y = std::min(y-radius_, min_y);
            max_y = std::max(y+radius_, max_y);
    
            pixels.push_back( getIndex(mx, my) );
        }
    }
    inflater.inflate(this, pixels, radius_);

    if(old_x0_ < old_x1_){
         min_x_ = std::min(min_x, old_x0_);
         min_y_ = std::min(min_y, old_y0_);
         max_x_ = std::max(max_x, old_x1_);
         max_y_ = std::max(max_y, old_y1_);
    }else{
        min_x_ = min_x;
        min_y_ = min_y;
        max_x_ = max_x;
        max_y_ = max_y;    
    }
    old_x0_ = min_x;
    old_x1_ = max_x;
    old_y0_ = min_y;
    old_y1_ = max_y;
    needsUpdate = true;
        
}


void PathLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;
  if(needsUpdate){    
    *min_x = std::min(min_x_, *min_x);
    *min_y = std::min(min_y_, *min_y);
    *max_x = std::max(max_x_, *max_x);
    *max_y = std::max(max_y_, *max_y);
    needsUpdate = false;
  }
}

void PathLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

} // end namespace
