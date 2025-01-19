#include "control_node.hpp"

ControlNode::ControlNode()
    : Node("control"),
      control_(robot::ControlCore(this->get_logger())),
      lookahead_distance_(1.0), // Lookahead distance in meters
      goal_tolerance_(0.1)      // Tolerance to consider goal reached
{
    // Subscribers
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

    // Publisher
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Timer for periodic control updates
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    current_path_ = msg;
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_odom_ = msg;
}

void ControlNode::controlLoop() {
    if (!current_path_ || !current_odom_) {
        RCLCPP_WARN(this->get_logger(), "Waiting for path and odometry data...");
        return;
    }

    // Find the lookahead point
    auto lookahead_point = findLookaheadPoint();
    if (!lookahead_point) {
        RCLCPP_INFO(this->get_logger(), "No valid lookahead point found. Stopping robot.");
        geometry_msgs::msg::Twist stop_cmd;
        cmd_vel_pub_->publish(stop_cmd); // Stop the robot
        return;
    }

    // Compute velocity commands
    geometry_msgs::msg::Twist cmd_vel = computeVelocity(*lookahead_point);

    // Publish velocity commands
    cmd_vel_pub_->publish(cmd_vel);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
    for (const auto &pose : current_path_->poses) {
        double distance = computeDistance(
            current_odom_->pose.pose.position, pose.pose.position);
        if (distance >= lookahead_distance_) {
            return pose; // Return the first valid lookahead point
        }
    }
    return std::nullopt; // No valid lookahead point found
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
    geometry_msgs::msg::Twist cmd_vel;

    // Calculate the robot's current yaw
    double robot_yaw = extractYaw(current_odom_->pose.pose.orientation);

    // Calculate the heading to the target
    double dx = target.pose.position.x - current_odom_->pose.pose.position.x;
    double dy = target.pose.position.y - current_odom_->pose.pose.position.y;
    double heading_to_target = std::atan2(dy, dx);

    // Calculate the angular error
    double angular_error = heading_to_target - robot_yaw;

    // Wrap angular error to [-pi, pi]
    if (angular_error > M_PI) angular_error -= 2 * M_PI;
    if (angular_error < -M_PI) angular_error += 2 * M_PI;

    // Set linear and angular velocities
    cmd_vel.linear.x = 0.5; // Constant forward speed
    cmd_vel.angular.z = 1.0 * angular_error; // Proportional control for angular velocity

    return cmd_vel;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
    return std::sqrt(std::pow(b.x - a.x, 2) + std::pow(b.y - a.y, 2));
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
    // Convert quaternion to yaw (Euler angle)
    double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
    double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
