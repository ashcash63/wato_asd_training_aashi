#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
 
#include "costmap_core.hpp"
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();

    // Place callback function here
    void publishMessage();
   // void publishMessage(int (*arr)[300]);
    void publishCostmap(int (*arr)[300]);
    void lidar_sub(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    //void odom_sub(const nav_msgs::msg::Odometry::SharedPtr odom);

    void initializeCostmap();
    void markObstacle(int x_grid, int y_grid);
    void convertToGrid(double range, double angle, int &x_grid, int &y_grid);
    void inflateObstacles(int grid[300][300], double inflation_radius, int max_cost);
    //void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    
    // Place callback function here
    
 
  private:
    robot::CostmapCore costmap_;
    // Place these constructs here
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    float x_ = -1;
    float y_ = -1;
    float dir_x_ = -100;
    float dir_ = 0;
    float dir_y_ = -100;
    int x_grid_ = 0;
    int y_grid_ = 0;
    std::vector<std::vector<int>> occupancy_grid_;
    nav_msgs::msg::OccupancyGrid costmap_msg_;
    rclcpp::TimerBase::SharedPtr timer_;
};
 
#endif 