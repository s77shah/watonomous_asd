#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();

    // Timer callback to publish a test message
    void publishMessage();

    // Callback function to handle LaserScan messages
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    // Initialize the costmap
    void initializeCostmap();

    // Convert LaserScan range and angle to grid coordinates
    void convertToGrid(double range, double angle, int &x_grid, int &y_grid);

    // Mark an obstacle in the costmap
    void markObstacle(int x_grid, int y_grid);

    // Inflate obstacles in the costmap
    void inflateObstacles();

    // Publish the costmap as an OccupancyGrid
    void publishCostmap();


  private:

    // costmap object
    robot::CostmapCore costmap_;
    // publishers/subscribers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

    // periodic test message publishing timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Costmap parameters
    double resolution_; // grid res
    int width_;         // number of x cells
    int height_;        // number of y cells
    double origin_x_;   // costmap origin
    double origin_y_;   // costmap origin

    // Costmap data
    std::vector<std::vector<int>> costmap_grid; //array
};

#endif 