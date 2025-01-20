#include "planner_core.hpp"
#include <queue>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <algorithm>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) : logger_(logger) {}

nav_msgs::msg::Path PlannerCore::planPath(
    const nav_msgs::msg::OccupancyGrid &map,
    const geometry_msgs::msg::Pose &start,
    const geometry_msgs::msg::PointStamped &goal)
{
    nav_msgs::msg::Path path;
    path.header.stamp = rclcpp::Clock().now();
    path.header.frame_id = "sim_world";

    // Convert start and goal to grid
    CellIndex start_cell((start.position.x - map.info.origin.position.x) / map.info.resolution, (start.position.y - map.info.origin.position.y) / map.info.resolution);
    CellIndex goal_cell((goal.point.x - map.info.origin.position.x) / map.info.resolution, (goal.point.y - map.info.origin.position.y) / map.info.resolution);

    // A* open set (priority queue)
    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
    open_set.emplace(start_cell, heuristic(start_cell, goal_cell, map));

    // A* cost maps
    std::unordered_map<CellIndex, double, CellIndexHash> g_score;
    g_score[start_cell] = 0.0;
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

    while (!open_set.empty())
    {
        AStarNode current_node = open_set.top();
        open_set.pop();

        if (current_node.index == goal_cell)
        {
            reconstructPath(came_from, current_node.index, start_cell, map, path);
            return path;
        }

        // Get 8 neighbor cells
        // std::vector<CellIndex> neighbors = getNeighbors(current_node.index);
        std::vector<CellIndex> neighbors = { // somehow this works better
            CellIndex(current_node.index.x + 1, current_node.index.y),
            CellIndex(current_node.index.x - 1, current_node.index.y),
            CellIndex(current_node.index.x, current_node.index.y + 1),
            CellIndex(current_node.index.x, current_node.index.y - 1),
            CellIndex(current_node.index.x + 1, current_node.index.y + 1),
            CellIndex(current_node.index.x - 1, current_node.index.y - 1),
            CellIndex(current_node.index.x + 1, current_node.index.y - 1),
            CellIndex(current_node.index.x - 1, current_node.index.y + 1)
        };

        for (const auto &neighbor : neighbors)
        {
            if (isValid(neighbor, map)) {
                // Apply diagonal/straight step cost
                double step_cost = (std::abs(neighbor.x - current_node.index.x) + std::abs(neighbor.y - current_node.index.y) == 2) ? std::sqrt(2.0) : 1.0;
                double temp_g_score = g_score[current_node.index] + step_cost;

                if (g_score.find(neighbor) == g_score.end() || temp_g_score < g_score[neighbor])
                {
                    came_from[neighbor] = current_node.index;
                    g_score[neighbor] = temp_g_score;
                    double f_score = temp_g_score + heuristic(neighbor, goal_cell, map);
                    open_set.emplace(neighbor, f_score);
                }
            }
        }
    }

    return path;
}

bool PlannerCore::isValid(const CellIndex &cell, const nav_msgs::msg::OccupancyGrid &map)
{
    int idx = cell.y * map.info.width + cell.x;
    return idx >= 0 && idx < map.data.size() && map.data[idx] < 20;
}

double PlannerCore::heuristic(const CellIndex &a, const CellIndex &b, const nav_msgs::msg::OccupancyGrid &map)
{
    double base_heuristic = std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));

    // Add an additional penalty based on proximity to obstacles
    double penalty = 0.0;
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            int nx = a.x + dx;
            int ny = a.y + dy;

            if (nx >= 0 && nx < map.info.width && ny >= 0 && ny < map.info.height) {
                int index = ny * map.info.width + nx;
                if (map.data[index] >= 20) {
                    penalty += 7.0 / (std::sqrt(dx*dx + dy*dy) + 1);
                }
            }
        }
    }

    return base_heuristic + penalty;
}

void PlannerCore::reconstructPath(
    const std::unordered_map<CellIndex, CellIndex, CellIndexHash> &came_from,
    const CellIndex &current,
    const CellIndex &start_cell,
    const nav_msgs::msg::OccupancyGrid &map,
    nav_msgs::msg::Path &path)
{
    CellIndex current_cell = current;
    while (came_from.find(current_cell) != came_from.end() || current_cell == start_cell)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "sim_world";
        pose.pose.position.x = current_cell.x * map.info.resolution + map.info.origin.position.x;
        pose.pose.position.y = current_cell.y * map.info.resolution + map.info.origin.position.y;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);

        if (current_cell == start_cell) {
            break;
        }

        current_cell = came_from.at(current_cell);
    }

    std::reverse(path.poses.begin(), path.poses.end());
}

std::vector<CellIndex> PlannerCore::getNeighbors(const CellIndex &cell)
{
    int directions[8][2] = {
        {1, 0},  {0, 1},  {-1, 0}, {0, -1},
        {1, 1},  {-1, -1}, {1, -1}, {-1, 1}
    };

    std::vector<CellIndex> neighbors;
    for (int i = 0; i < 8; ++i)
    {
        int nx = cell.x + directions[i][0];
        int ny = cell.y + directions[i][1];
        neighbors.emplace_back(nx, ny);
    }

    return neighbors;
}

}