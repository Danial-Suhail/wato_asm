#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
    path_sub_ = create_subscription<nav_msgs::msg::Path>("/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    timer_ = create_wall_timer(std::chrono::milliseconds(100),std::bind(&ControlNode::timerCallback, this));
    
    RCLCPP_INFO(get_logger(), "Control node initialized");
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    current_path_ = msg;
    RCLCPP_INFO(get_logger(), "Received new path with %zu points", msg->poses.size());
    
    if (msg->poses.empty()) {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel);
        return;
    }
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_odom_ = msg;
}

void ControlNode::timerCallback() {
    if (!current_path_ || !current_odom_) {
        return;  
    }
    geometry_msgs::msg::Twist cmd_vel = control_.computeVelocity(*current_path_, *current_odom_);
    cmd_vel_pub_->publish(cmd_vel);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
