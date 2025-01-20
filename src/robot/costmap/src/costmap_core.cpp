#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger), resolution_(0.1), size_(100) {
    costmap_.resize(size_ * size_, 0);
}

void CostmapCore::processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
    std::fill(costmap_.begin(), costmap_.end(), 0);

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];

        if (range > scan->range_min && range < scan->range_max) {
            double x = range * cos(angle);
            double y = range * sin(angle);

            // if the point is within the costmap area, mark the point on the costmap
            int x_grid, y_grid;
            if (convertToGrid(x, y, x_grid, y_grid)) {
                markObstacle(x_grid, y_grid);
            }
        }
    }

    inflateObstacles();
}

bool CostmapCore::convertToGrid(double x, double y, int &x_grid, int &y_grid) {
    // adjust for the offset
    x_grid = static_cast<int>((x + size_ * resolution_ / 2) / resolution_);
    y_grid = static_cast<int>((y + size_ * resolution_ / 2) / resolution_);

    return (x_grid >= 0 && x_grid < size_ && y_grid >= 0 && y_grid < size_); // check if within costmap area bound
}

void CostmapCore::markObstacle(int x_grid, int y_grid) {
    costmap_[y_grid * size_ + x_grid] = 100;
}

void CostmapCore::inflateObstacles() {
    std::vector<int8_t> inflated_costmap = costmap_;
    int inflation_radius_in_cells = static_cast<int>((1.0 / resolution_)*5.0);

    // Loop over each point and find the obstacle point
    for (int y = 0; y < size_; ++y) {
        for (int x = 0; x < size_; ++x) {
            if (costmap_[y * size_ + x] == 100) {
                // Loop over every point within the inflation radius
                for (int dy = -inflation_radius_in_cells; dy <= inflation_radius_in_cells; ++dy) { // define the range of points to loop over
                    for (int dx = -inflation_radius_in_cells; dx <= inflation_radius_in_cells; ++dx) {
                        int nx = x + dx; // actual coordinates of the points within the radius
                        int ny = y + dy;
                        if (nx >= 0 && nx < size_ && ny >= 0 && ny < size_) { // check if within the costmap area bound
                            double distance = sqrt(dx * dx + dy * dy) * resolution_;
                            if (distance <= 4.0) {
                                int max_cost = 100;
                                int inflation_radius_in_meters = 1.0;
                                int cost = static_cast<int>(max_cost * (1 - distance / inflation_radius_in_meters));
                                inflated_costmap[ny * size_ + nx] = std::max(
                                    static_cast<int>(inflated_costmap[ny * size_ + nx]), cost);
                            }
                        }
                    }
                }
            }
        }
    }

    costmap_ = inflated_costmap;
}

nav_msgs::msg::OccupancyGrid CostmapCore::generateOccupancyGrid(const rclcpp::Time& timestamp, const std::string& lidar_frame_id) {
    nav_msgs::msg::OccupancyGrid grid_msg;
    grid_msg.header.stamp = timestamp;
    grid_msg.header.frame_id = lidar_frame_id;

    grid_msg.info.resolution = resolution_;
    grid_msg.info.width = size_;
    grid_msg.info.height = size_;
    grid_msg.info.origin.position.x = -size_ * resolution_ / 2; // center the grid around the robot
    grid_msg.info.origin.position.y = -size_ * resolution_ / 2; // center the grid around the robot
    grid_msg.info.origin.orientation.w = 1.0;

    grid_msg.data = costmap_;
    return grid_msg;
}

}