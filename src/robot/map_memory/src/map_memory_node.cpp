#include "map_memory_node.hpp"

// nav_msgs::msg::OccupancyGrid global_map_;
// double last_x_, last_y_;
// const double distance_threshold_;
// bool costmap_updated_ = false;
MapMemoryNode::MapMemoryNode() : Node("map_memory"), last_x_(0.0), last_y_(0.0), distance_threshold_(1.5),
costmap_updated_(false), should_update_map_(false) {
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", 10,std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10,std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  timer_ = this->create_wall_timer(std::chrono::duration<double>(1),std::bind(&MapMemoryNode::updateMap, this));

}

// Global map and robot position

 
// Callback for costmap updates
void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        // Store the latest costmap
    latest_costmap_ = *msg;
    costmap_updated_ = true;

    if (global_map_.data.empty()) {
      global_map_ = *msg; // Copy metadata
      global_map_.data.assign(global_map_.data.size(), -1);
    }
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
  double dx = msg->pose.pose.position.x;
  double dy = msg->pose.pose.position.y;

  // Compute distance traveled
        double distance = std::sqrt(std::pow(dx - last_x_, 2) + std::pow(dy - last_y_, 2));
        if (distance >= distance_threshold_) {
            last_x_ = dx;
            last_y_ = dy;
            should_update_map_ = true;
        }
}

void MapMemoryNode::updateMap() {
    if (should_update_map_ && costmap_updated_) {
        integrateCostmap();
        map_pub_->publish(global_map_);
        should_update_map_ = false;
    }
}
//Timer-based map update


void MapMemoryNode::integrateCostmap(){
  // Transform and merge the latest costmap into the global map
  // (Implementation would handle grid alignment and merging logic)
  int width = latest_costmap_.info.width;
  int height = latest_costmap_.info.height;

  for(int y=0; y<height; ++y){
    for (int x=0; x<width; ++x){
      int index = y*width +x;
      if (latest_costmap_.data[index]!=-1){
        global_map_.data[index] = latest_costmap_.data[index];
      }

    }
  }



}

//bool should_update_map_ = false;
//nav_msgs::msg::OccupancyGrid latest_costmap_;


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
