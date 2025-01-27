#include <chrono>
#include <memory>
#include <cmath>
#include <vector>  
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    try {
        if (msg->ranges.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty laser scan");
            return;
        }

        const float resolution = 0.05f;  
        const float map_size = 30.0f;   
        const int grid_size = static_cast<int>(map_size / resolution);
        
        costmap_msg_.header.stamp = this->now();
        costmap_msg_.header.frame_id = "robot/chassis/lidar";
        costmap_msg_.info.resolution = resolution;
        costmap_msg_.info.width = grid_size;
        costmap_msg_.info.height = grid_size;
        costmap_msg_.info.origin.position.x = -map_size / 2.0;
        costmap_msg_.info.origin.position.y = -map_size / 2.0;
        costmap_msg_.info.origin.orientation.w = 1.0;
        costmap_msg_.data.assign(grid_size * grid_size, 0);

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            double angle = msg->angle_min + i * msg->angle_increment;
            double range = msg->ranges[i];

            if (std::isfinite(range) && range < msg->range_max && range > msg->range_min) {
                double x = range * std::cos(angle);
                double y = range * std::sin(angle);

                int x_grid = static_cast<int>((x - costmap_msg_.info.origin.position.x) / resolution);
                int y_grid = static_cast<int>((y - costmap_msg_.info.origin.position.y) / resolution);

                if (x_grid >= 0 && x_grid < grid_size && y_grid >= 0 && y_grid < grid_size) {
                    size_t index = y_grid * grid_size + x_grid;
                    costmap_msg_.data[index] = 100;
                }
            }
        }

        std::vector<int8_t> inflated_map = costmap_msg_.data;
        float inflation_radius = 2.0;
        int cell_inflation_radius = static_cast<int>(inflation_radius / resolution);
        
        for (int y = 0; y < grid_size; ++y) {
            for (int x = 0; x < grid_size; ++x) {
                if (costmap_msg_.data[y * grid_size + x] == 100) {
                    for (int dy = -cell_inflation_radius; dy <= cell_inflation_radius; ++dy) {
                        for (int dx = -cell_inflation_radius; dx <= cell_inflation_radius; ++dx) {
                            int new_y = y + dy;
                            int new_x = x + dx;
                            
                            if (new_y >= 0 && new_y < grid_size && new_x >= 0 && new_x < grid_size) {
                                float distance = std::sqrt(dx * dx + dy * dy) * resolution;
                                
                                if (distance <= inflation_radius) {
                                    float cost = 100.0 * std::pow((1.0 - distance / inflation_radius), 2);
                                    size_t index = new_y * grid_size + new_x;
                                    inflated_map[index] = std::max(inflated_map[index], static_cast<int8_t>(cost));
                                }
                            }
                        }
                    }
                }
            }
        }

        costmap_msg_.data = inflated_map;
        costmap_pub_->publish(costmap_msg_);

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in laserCallback: %s", e.what());
    }
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}