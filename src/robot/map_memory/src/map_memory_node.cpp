#include "map_memory_node.hpp"
#include <cmath>

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
    
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    latest_costmap_ = msg;
    costmap_updated = true;

    if (global_map_.data.empty()) {
        global_map_ = *msg;
        global_map_.header.frame_id = "sim_world";
        global_map_.info.origin.position.x = -15.0;
        global_map_.info.origin.position.y = -15.0;
        global_map_.info.origin.orientation.w = 1.0;
    }
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    latest_odometry_ = msg;
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    double distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));
    
    if (distance >= distance_threshold) {
        last_x = x;
        last_y = y;
        should_update_map = true;
    }
}

void MapMemoryNode::updateMap() {
    if (should_update_map && costmap_updated) {
        integrateCostmap();
        global_map_.header.stamp = this->now();
        map_pub_->publish(global_map_);
        should_update_map = false;
        costmap_updated = false;
    }
}

void MapMemoryNode::integrateCostmap() {
    if (!latest_costmap_ || !latest_odometry_) {
        return;
    }

    float robotYaw = 2.0f * std::atan2(
        latest_odometry_->pose.pose.orientation.z,
        latest_odometry_->pose.pose.orientation.w);
    float cosYaw = std::cos(robotYaw);
    float sinYaw = std::sin(robotYaw);
    float robotX = latest_odometry_->pose.pose.position.x;
    float robotY = latest_odometry_->pose.pose.position.y;

    for (size_t i = 0; i < latest_costmap_->data.size(); ++i) {
        int costmap_x = i % latest_costmap_->info.width;
        int costmap_y = i / latest_costmap_->info.width;

        float localX = costmap_x * latest_costmap_->info.resolution + latest_costmap_->info.origin.position.x;
        float localY = costmap_y * latest_costmap_->info.resolution + latest_costmap_->info.origin.position.y;

        float globalX = cosYaw * localX - sinYaw * localY + robotX;
        float globalY = sinYaw * localX + cosYaw * localY + robotY;

        int global_x = static_cast<int>((globalX - global_map_.info.origin.position.x) / global_map_.info.resolution);
        int global_y = static_cast<int>((globalY - global_map_.info.origin.position.y) / global_map_.info.resolution);

        if (global_x >= 0 && global_x < static_cast<int>(global_map_.info.width) && global_y >= 0 && global_y < static_cast<int>(global_map_.info.height)) {
            size_t global_index = global_y * global_map_.info.width + global_x;
            if (latest_costmap_->data[i] > 0) {
                global_map_.data[global_index] = std::max(global_map_.data[global_index], latest_costmap_->data[i]);
            }
        }
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapMemoryNode>());
    rclcpp::shutdown();
    return 0;
}
