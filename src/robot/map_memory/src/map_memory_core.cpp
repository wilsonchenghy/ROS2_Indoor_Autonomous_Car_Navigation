#include "map_memory_core.hpp"
#include <cmath>
#include <optional>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger), last_x(0.0), last_y(0.0), distance_threshold(1.5), costmap_updated_(false) {}

void MapMemoryCore::updateCostmap(const nav_msgs::msg::OccupancyGrid &costmap) {
  latest_costmap_ = costmap;
  costmap_updated_ = true;
}

void MapMemoryCore::updateOdometry(const nav_msgs::msg::Odometry &odom) {
  double current_x = odom.pose.pose.position.x;
  double current_y = odom.pose.pose.position.y;

  double distance = std::sqrt(std::pow(current_x - last_x, 2) + std::pow(current_y - last_y, 2));
  if (distance >= distance_threshold) {
    last_x = current_x;
    last_y = current_y;
    should_update_map_ = true;
  }
}

std::optional<nav_msgs::msg::OccupancyGrid> MapMemoryCore::generateGlobalMap(const nav_msgs::msg::Odometry &msg) {
  if (!should_update_map_ || !costmap_updated_) {
    return std::nullopt;
  }

  if (!global_map_initialized_) {
    initializeGlobalMap();
  } else {
    integrateCostmap(msg);
  }

  should_update_map_ = false;
  return global_map_;
}

void MapMemoryCore::initializeGlobalMap() {
  int global_width = 300.0;
  int global_height = 300.0;

  global_map_.header.frame_id = "sim_world";

  global_map_.info.resolution = latest_costmap_.info.resolution;
  global_map_.info.width = global_width;
  global_map_.info.height = global_height;
  global_map_.info.origin.position.x = -global_width * global_map_.info.resolution / 2.0; // center the map
  global_map_.info.origin.position.y = -global_width * global_map_.info.resolution / 2.0; // // center the map
  global_map_.info.origin.orientation.w = 1.0;

  global_map_.data.resize(global_map_.info.width * global_map_.info.height, 0);

  global_map_initialized_ = true;
}

void MapMemoryCore::integrateCostmap(const nav_msgs::msg::Odometry &odom) {
  // Get robot's orientation
  double qx = odom.pose.pose.orientation.x;
  double qy = odom.pose.pose.orientation.y;
  double qz = odom.pose.pose.orientation.z;
  double qw = odom.pose.pose.orientation.w;

  // Calculate robot's yaw
  double yaw_temp1 = 2.0 * (qw * qz + qx * qy);
  double yaw_temp2 = 1.0 - 2.0 * (qy * qy + qz * qz);
  double robot_yaw = std::atan2(yaw_temp1, yaw_temp2);

  // Get robot's position
  double robot_x = odom.pose.pose.position.x;
  double robot_y = odom.pose.pose.position.y;

  // Get costmap info
  double costmap_resolution = latest_costmap_.info.resolution;
  double costmap_origin_x = latest_costmap_.info.origin.position.x;
  double costmap_origin_y = latest_costmap_.info.origin.position.y;
  unsigned int costmap_width = latest_costmap_.info.width;
  unsigned int costmap_height = latest_costmap_.info.height;

  // Loop over each points in the costmap to integrate it to the global map
  for (unsigned int y = 0; y < costmap_height; ++y) {
    for (unsigned int x = 0; x < costmap_width; ++x) {
      // Transform from costmap frame to global map frame
      double c_x = costmap_origin_x + (x + 0.5) * costmap_resolution;
      double c_y = costmap_origin_y + (y + 0.5) * costmap_resolution;

      double robot_yaw_cos = std::cos(robot_yaw);
      double robot_yaw_sin = std::sin(robot_yaw);
      double global_x_in_meter = robot_x + (c_x * robot_yaw_cos - c_y * robot_yaw_sin);
      double global_y_in_meter = robot_y + (c_x * robot_yaw_sin + c_y * robot_yaw_cos);

      // Convert to in grid
      double global_map_origin_x = global_map_.info.origin.position.x;
      double global_map_origin_y = global_map_.info.origin.position.y;
      double global_map_resolution = global_map_.info.resolution;

      int global_x_in_grid = (global_x_in_meter - global_map_origin_x) / global_map_resolution;
      int global_y_in_grid = (global_y_in_meter - global_map_origin_y) / global_map_resolution;

      if (isWithinBounds(global_x_in_grid, global_y_in_grid)) {
        // Merge into global map
        size_t global_index = global_y_in_grid * global_map_.info.width + global_x_in_grid;
        int8_t &global_val = global_map_.data[global_index];
        int global_val_int = global_val;
        int costmap_val = latest_costmap_.data[y * costmap_width + x];
        int merged_cost = std::max(global_val_int, costmap_val);
        global_val = merged_cost;
      }
    }
  }
}

bool MapMemoryCore::isWithinBounds(int x, int y) {
  if (x < 0 || x >= global_map_.info.width || y < 0 || y >= global_map_.info.height) {
    return false;
  }
  return true;
}

}