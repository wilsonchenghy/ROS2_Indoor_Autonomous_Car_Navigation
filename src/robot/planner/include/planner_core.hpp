#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot
{

struct CellIndex
{
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }

  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};

struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

struct AStarNode
{
  CellIndex index;
  double f_score;

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    return a.f_score > b.f_score;
  }
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    nav_msgs::msg::Path planPath(
      const nav_msgs::msg::OccupancyGrid &map,
      const geometry_msgs::msg::Pose &start,
      const geometry_msgs::msg::PointStamped &goal);

  private:
    rclcpp::Logger logger_;

    // A* Algorithm Helpers
    bool isValid(const CellIndex &cell, const nav_msgs::msg::OccupancyGrid &map);
    double heuristic(const CellIndex &a, const CellIndex &b, const nav_msgs::msg::OccupancyGrid &map);
    void reconstructPath(
      const std::unordered_map<CellIndex, CellIndex, CellIndexHash> &came_from,
      const CellIndex &current,
      const CellIndex &start_cell,
      const nav_msgs::msg::OccupancyGrid &map,
      nav_msgs::msg::Path &path);
    std::vector<CellIndex> getNeighbors(const CellIndex &cell);
    
};

}  

#endif  
