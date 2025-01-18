#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>
#include <cmath>

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    void processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr& scan);
    nav_msgs::msg::OccupancyGrid generateOccupancyGrid(const rclcpp::Time& timestamp, const std::string& lidar_frame_id);

  private:
    bool convertToGrid(double x, double y, int &x_grid, int &y_grid);
    void markObstacle(int x_grid, int y_grid);
    void inflateObstacles();
    
    rclcpp::Logger logger_;
    std::vector<int8_t> costmap_;
    double resolution_;
    int size_;

};

}  

#endif  