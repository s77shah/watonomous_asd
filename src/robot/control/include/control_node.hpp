#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "control_core.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    robot::ControlCore control_;

    // Subscribers and Publisher
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // Data
    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr current_odom_;
    double lookahead_distance_;
    double goal_tolerance_;

    // Timer
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Methods
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void controlLoop();
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint();
    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped &target);
    double computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b);
    double extractYaw(const geometry_msgs::msg::Quaternion &quat);
};

#endif
