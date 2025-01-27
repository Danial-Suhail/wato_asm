#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <optional>
#include <cmath>

namespace robot
{

class ControlCore {
  public:
    ControlCore(const rclcpp::Logger& logger);
  
    geometry_msgs::msg::Twist computeVelocity(const nav_msgs::msg::Path& path,const nav_msgs::msg::Odometry& odom);
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(const nav_msgs::msg::Path& path, const geometry_msgs::msg::Pose& robot_pose);
    double computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b);
    double getYaw(const geometry_msgs::msg::Quaternion& q);
    bool isAtGoal(const geometry_msgs::msg::Pose& robot_pose, const nav_msgs::msg::Path& path);
    bool isNearObstacle(const nav_msgs::msg::OccupancyGrid& map, const geometry_msgs::msg::Pose& pose);

  private:
    rclcpp::Logger logger_;
    
    const double LOOKAHEAD_DISTANCE = 1.0;  
    const double LINEAR_SPEED = 0.5;       
    const double MAX_ANGULAR_SPEED = 1.0;   
    const double GOAL_TOLERANCE = 0.2;     
    const double COLLISION_THRESHOLD = 0.5;  // meters
};

} 

#endif 
