#include "control_core.hpp"

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

geometry_msgs::msg::Twist ControlCore::computeVelocity(
    const nav_msgs::msg::Path& path,
    const nav_msgs::msg::Odometry& odom) {
    
    geometry_msgs::msg::Twist cmd_vel;
    
    // Check if we've reached the goal
    if (isAtGoal(odom.pose.pose, path)) {
        RCLCPP_INFO(logger_, "Goal reached!");
        return cmd_vel;  // Return zero velocity
    }
    
    auto lookahead_point = findLookaheadPoint(path, odom.pose.pose);
    if (!lookahead_point) {
        RCLCPP_WARN(logger_, "No valid lookahead point found");
        return cmd_vel;  
    }
    
    const auto& robot_pose = odom.pose.pose;
    double robot_yaw = getYaw(robot_pose.orientation);
    
    double dx = lookahead_point->pose.position.x - robot_pose.position.x;
    double dy = lookahead_point->pose.position.y - robot_pose.position.y;
    double target_angle = std::atan2(dy, dx);
    
    double angle_error = target_angle - robot_yaw;
    if (angle_error > M_PI) angle_error -= 2 * M_PI;
    if (angle_error < -M_PI) angle_error += 2 * M_PI;
    
    cmd_vel.linear.x = LINEAR_SPEED;
    cmd_vel.angular.z = std::clamp(2.0 * angle_error, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
    
    return cmd_vel;
}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(
    const nav_msgs::msg::Path& path,
    const geometry_msgs::msg::Pose& robot_pose) {
    
    if (path.poses.empty()) {
        return std::nullopt;
    }
    
    for (size_t i = 0; i < path.poses.size(); ++i) {
        double distance = computeDistance(path.poses[i].pose.position, robot_pose.position);
        if (distance >= LOOKAHEAD_DISTANCE) {
            return path.poses[i];
        }
    }
    
    return path.poses.back();
}

double ControlCore::computeDistance(
    const geometry_msgs::msg::Point& a,
    const geometry_msgs::msg::Point& b) {
    
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

double ControlCore::getYaw(const geometry_msgs::msg::Quaternion& q) {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

bool ControlCore::isAtGoal(
    const geometry_msgs::msg::Pose& robot_pose,
    const nav_msgs::msg::Path& path) {
    
    if (path.poses.empty()) {
        return false;
    }
    const auto& goal = path.poses.back().pose.position;
    double distance = computeDistance(goal, robot_pose.position);
    
    if (path.poses.size() <= 1) {
        RCLCPP_WARN(logger_, "Path has only 1 point, not considering goal reached");
        return false;
    }
    
    bool at_goal = distance < GOAL_TOLERANCE;
    if (at_goal) {
        RCLCPP_INFO(logger_, "Actually reached goal! Distance: %.2f", distance);
    }
    return at_goal;
}

bool ControlCore::isNearObstacle(
    const nav_msgs::msg::OccupancyGrid& map,
    const geometry_msgs::msg::Pose& pose) {

    int x = static_cast<int>((pose.position.x - map.info.origin.position.x) / map.info.resolution);
    int y = static_cast<int>((pose.position.y - map.info.origin.position.y) / map.info.resolution);
    
    const int check_radius = static_cast<int>(COLLISION_THRESHOLD / map.info.resolution);
    
    for (int dx = -check_radius; dx <= check_radius; dx++) {
        for (int dy = -check_radius; dy <= check_radius; dy++) {
            int nx = x + dx;
            int ny = y + dy;
            
            if (nx >= 0 && nx < static_cast<int>(map.info.width) &&
                ny >= 0 && ny < static_cast<int>(map.info.height)) {
                int index = ny * map.info.width + nx;
                if (map.data[index] >= 50) { 
                    RCLCPP_WARN(logger_, "Obstacle detected nearby!");
                    return true;
                }
            }
        }
    }
    return false;
}

}  
