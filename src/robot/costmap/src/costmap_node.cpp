#include <chrono>
#include <memory>

#include <cmath>


#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  // Initialize costmap parameters
  resolution_ = 0.1; // 0.1 meters per cell
  width_ = 100;      // 10 meters
  height_ = 100;     // 10 meters
  origin_x_ = -5.0;  // -5 meters
  origin_y_ = -5.0;  // -5 meters

  initializeCostmap();
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  initializeCostmap();

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];
    if (range > scan->range_min && range < scan->range_max) {
      int x_grid, y_grid;
      convertToGrid(range, angle, x_grid, y_grid);
      markObstacle(x_grid, y_grid);
    }
  }

  inflateObstacles();
  publishCostmap();
}

// Function to initialize the costmap
void CostmapNode::initializeCostmap() {
  costmap_grid.resize(height_, std::vector<int>(width_, 0)); // Initialize to free space
}

// Convert LaserScan data to grid coordinates
void CostmapNode::convertToGrid(double range, double angle, int &x_grid, int &y_grid) {
  double x = range * std::cos(angle);
  double y = range * std::sin(angle);

  x_grid = static_cast<int>((x - origin_x_) / resolution_);
  y_grid = static_cast<int>((y - origin_y_) / resolution_);
}

// Mark obstacle positions in the costmap
void CostmapNode::markObstacle(int x_grid, int y_grid) {
  if (x_grid >= 0 && x_grid < width_ && y_grid >= 0 && y_grid < height_) {
    costmap_grid[y_grid][x_grid] = 100; // Mark as occupied
  }
}

// Inflate obstacles to account for uncertainty
void CostmapNode::inflateObstacles() {
  std::vector<std::vector<int>> inflated_costmap = costmap_grid; // Copy for inflation
  int inflation_radius = static_cast<int>(1.0 / resolution_); // 1 meter radius

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (costmap_grid[y][x] == 100) { // Obstacle cell
        for (int dy = -inflation_radius; dy <= inflation_radius; ++dy) {
          for (int dx = -inflation_radius; dx <= inflation_radius; ++dx) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
              double distance = std::sqrt(dx * dx + dy * dy) * resolution_;
              if (distance <= 1.0) {
                int cost = static_cast<int>(100 * (1 - distance / 1.0));
                inflated_costmap[ny][nx] = std::max(inflated_costmap[ny][nx], cost);
              }
            }
          }
        }
      }
    }
  }
  costmap_grid = inflated_costmap;
}

// Publish the costmap to /costmap topic
void CostmapNode::publishCostmap() {
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "map";
  msg.info.resolution = resolution_;
  msg.info.width = width_;
  msg.info.height = height_;
  msg.info.origin.position.x = origin_x_;
  msg.info.origin.position.y = origin_y_;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  msg.data.resize(width_ * height_);
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      msg.data[y * width_ + x] = costmap_grid[y][x];
    }
  }

  costmap_pub_->publish(msg);
}

// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}