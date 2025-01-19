#include "map_memory_node.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <cmath>

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())), last_x_(0.0), last_y_(0.0), distance_threshold_(1.5), costmap_updated_(false)  {
  // Initialize subscribers
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

    // Initialize publisher
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

    // Initialize timer
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    latest_costmap_ = *msg;
    costmap_updated_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    // Compute distance traveled
    double distance = std::sqrt(std::pow(x - last_x_, 2) + std::pow(y - last_y_, 2));
    if (distance >= distance_threshold_) {
        last_x_ = x;
        last_y_ = y;
        should_update_map_ = true;
    }
}

void MapMemoryNode::updateMap() {
    if (should_update_map_ && costmap_updated_) {
        integrateCostmap();
        map_pub_->publish(global_map_);
        should_update_map_ = false;
        costmap_updated_ = false;
    }
}

void MapMemoryNode::integrateCostmap() {
    // Merge the latest costmap into the global map
    if (global_map_.info.resolution == 0) {
        global_map_ = latest_costmap_; // Initialize the global map with the first costmap
        return;
    }

    // Transform and integrate the costmap into the global map
    for (int y = 0; y < latest_costmap_.info.height; ++y) {
        for (int x = 0; x < latest_costmap_.info.width; ++x) {
            int index = y * latest_costmap_.info.width + x;
            int global_x = x + std::round((latest_costmap_.info.origin.position.x - global_map_.info.origin.position.x) / global_map_.info.resolution);
            int global_y = y + std::round((latest_costmap_.info.origin.position.y - global_map_.info.origin.position.y) / global_map_.info.resolution);
            int global_index = global_y * global_map_.info.width + global_x;

            if (global_x >= 0 && global_x < global_map_.info.width && global_y >= 0 && global_y < global_map_.info.height) {
                if (latest_costmap_.data[index] != -1) { // Overwrite known data
                    global_map_.data[global_index] = latest_costmap_.data[index];
                }
            }
        }
    }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
