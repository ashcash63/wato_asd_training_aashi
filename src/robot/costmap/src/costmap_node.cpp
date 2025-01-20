#include <chrono>
#include <memory>
#include <cmath>
#include <vector>
#include "costmap_node.hpp"

const int gridwidth = 300;
const int gridheight = 300;
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
  //lidar subscriber
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::lidar_sub, this, std::placeholders::_1));
  //odom subscriber
  //odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&CostmapNode::odom_sub, this, std::placeholders::_1));
  grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}
 
//Define the timer to publish a message every 500ms
void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}

void CostmapNode::initializeCostmap(){
  double resolution = 0.1;

  //size of costmap
  int width = gridwidth;
  int height = gridheight;

  //create 2d array with default value 0
  occupancy_grid_.resize(height, std::vector<int>(width, 0));

  //metadata??
  costmap_msg_.info.resolution = resolution;
  costmap_msg_.info.width = width;
  costmap_msg_.info.height = height;
  costmap_msg_.info.origin.position.x = 0.0;
  costmap_msg_.info.origin.position.y = 0.0;
  costmap_msg_.info.origin.position.z = 0.0;
  costmap_msg_.info.origin.orientation.w = 1.0; 

}

void CostmapNode::markObstacle(int x_grid, int y_grid){
  //check boundary conditions 
  if (x_grid >=0 && x_grid < gridwidth && y_grid >= 0 && y_grid < gridheight){
    //set obstacle
    occupancy_grid_[x_grid][y_grid] = 100;
  }


}

void CostmapNode::convertToGrid(double range, double angle, int &x_grid, int &y_grid){
  //& because modifying value of x_grid & y_grid
  double x = range * cos(angle);
  double y = range * sin(angle);

  //transform these coordinates to grid indices using resolution and origin
  //note since grid indices are integers use static_cast<int> to round 
  x_grid = static_cast<int>((x - costmap_msg_.info.origin.position.x)/costmap_msg_.info.resolution);
  y_grid = static_cast<int>((y - costmap_msg_.info.origin.position.y)/costmap_msg_.info.resolution);


}

void CostmapNode::inflateObstacles(int grid[300][300], double inflation_radius, int max_cost){
  //int inflation_radius = 1;
  //int max_cost = 100;
  int inflation_radius_cells = static_cast<int>(inflation_radius *10);
  

  for (int y = 0; y < gridheight; ++y){
    for(int x= 0; x < gridwidth; ++x){
      // only check surrounding cells for cells that have max_cost
      if (grid[x][y] == max_cost){

        for (int ry = -inflation_radius_cells; ry <= inflation_radius_cells; ry++){
          for (int rx = -inflation_radius_cells; rx <= inflation_radius_cells; rx++){
            int dx = x + rx; //surrounding possibilities
            int dy = y + ry;
            //double check boundary conditions
            if (dx >=0 && dx <gridwidth && dy>=0 && dy<gridheight){
              double euclidean_dist = std::sqrt(rx * rx + ry * ry) /10;
              // only consider cells within the inflation radius
              if(euclidean_dist<=inflation_radius){

              int cost = max_cost * (1.0 - (euclidean_dist/inflation_radius));
              // update with max either cost or original

              grid[dy][dx] = std::max(grid[dy][dx], cost);
            }

          }
             
        }
      }
    }
    }
  }

}

void CostmapNode::publishCostmap(int (*grid)[gridwidth]) {
    costmap_msg_.header.stamp = this->now();
    costmap_msg_.header.frame_id = "map";

    costmap_msg_.data.clear();
    for (int y = 0; y < gridheight; ++y) {
        for (int x = 0; x < gridwidth; ++x) {
            costmap_msg_.data.push_back(grid[y][x]);
        }
    }

    grid_pub_->publish(costmap_msg_);
}



void CostmapNode::lidar_sub(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    if (!scan) {
        RCLCPP_ERROR(this->get_logger(), "Received null LaserScan pointer!");
        return;
    }
    // Step 1: Initialize costmap
    initializeCostmap();
    int array[gridwidth][gridheight] = {0};
 
    // Step 2: Convert LaserScan to grid and mark obstacles
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];
        if (range < scan->range_max && range > scan->range_min) {
            // Calculate grid coordinates
            int x_grid, y_grid;
            convertToGrid(range, angle, x_grid, y_grid);
            markObstacle(x_grid, y_grid);
        }
    }
 
    // Step 3: Inflate obstacles
    inflateObstacles(array, 1.6, 100);
 
    // Step 4: Publish costmap
    publishCostmap(array);
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}