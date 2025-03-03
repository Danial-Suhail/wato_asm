#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "costmap_core.hpp"

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"  
#include "nav_msgs/msg/occupancy_grid.hpp"  
#include "nav_msgs/msg/odometry.hpp"

class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    
    // Place callback function here
    void publishMessage();
    
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void inflateObstacles();

  private:
    robot::CostmapCore costmap_;
    // Place these constructs here
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

    nav_msgs::msg::OccupancyGrid costmap_msg_;
};
 
#endif 