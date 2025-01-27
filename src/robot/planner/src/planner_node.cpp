#include "planner_node.hpp"
#include <cmath>

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())), state_(State::WAITING_FOR_GOAL) {
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
    point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&PlannerNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Planner node initialized");
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    current_map_ = *msg;
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        planPath();
    }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    goal_ = *msg;
    goal_received_ = true;
    state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
    RCLCPP_INFO(this->get_logger(), "Received new goal: x=%f, y=%f", goal_.point.x, goal_.point.y);
    planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback() {
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        if (goalReached()) {
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            state_ = State::WAITING_FOR_GOAL;
            goal_received_ = false;
        } else {
            RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
            planPath();
        }
    }
}

bool PlannerNode::goalReached() {
    if (!goal_received_) return false;
    
    double dx = goal_.point.x - robot_pose_.position.x;
    double dy = goal_.point.y - robot_pose_.position.y;
    return std::sqrt(dx * dx + dy * dy) < 0.5;  
}

void PlannerNode::planPath() {
    if (!goal_received_ || current_map_.data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
        return;
    }

    nav_msgs::msg::Path path;
    path.header.stamp = this->now();
    path.header.frame_id = "sim_world";

    robot::CellIndex start(static_cast<int>((robot_pose_.position.x - current_map_.info.origin.position.x) / current_map_.info.resolution), static_cast<int>((robot_pose_.position.y - current_map_.info.origin.position.y) / current_map_.info.resolution));
    robot::CellIndex goal(static_cast<int>((goal_.point.x - current_map_.info.origin.position.x) / current_map_.info.resolution), static_cast<int>((goal_.point.y - current_map_.info.origin.position.y) / current_map_.info.resolution));

    std::vector<robot::CellIndex> path_indices = planner_.planPath(current_map_, start, goal);

    for (const auto& idx : path_indices) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = idx.x * current_map_.info.resolution + current_map_.info.origin.position.x;
        pose.pose.position.y = idx.y * current_map_.info.resolution + current_map_.info.origin.position.y;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
    }

    path_pub_->publish(path);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}
