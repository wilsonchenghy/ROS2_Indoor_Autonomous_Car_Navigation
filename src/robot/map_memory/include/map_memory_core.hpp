#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <optional>

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    void updateCostmap(const nav_msgs::msg::OccupancyGrid &costmap);
    void updateOdometry(const nav_msgs::msg::Odometry &odom);
    std::optional<nav_msgs::msg::OccupancyGrid> generateGlobalMap(const nav_msgs::msg::Odometry &msg);

  private:
    rclcpp::Logger logger_;
    
    nav_msgs::msg::OccupancyGrid global_map_;
    nav_msgs::msg::OccupancyGrid latest_costmap_;

    double last_x, last_y;
    const double distance_threshold;
    bool should_update_map_ = false;
    bool costmap_updated_ = false;
    bool global_map_initialized_ = false;

    void initializeGlobalMap();
    void integrateCostmap(const nav_msgs::msg::Odometry &msg);
    bool isWithinBounds(int x, int y);
};

}  

#endif  
