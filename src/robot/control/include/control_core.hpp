#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);

    void processPath(const nav_msgs::msg::Path::SharedPtr msg);
    void processOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);

    geometry_msgs::msg::Twist generateCommand();
  
  private:
    rclcpp::Logger logger_;

    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr current_odometry_;

    // Control parameters (example values)
    double lookahead_distance_;
    double linear_speed_;

    // Helper functions for calculation
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(double robot_x, double robot_y);
    double calculateSteeringAngle(double robot_x, double robot_y, const geometry_msgs::msg::PoseStamped& lookahead_point);
    double computeDistance(double x1, double y1, double x2, double y2);
    double extractYaw(const geometry_msgs::msg::Quaternion &quat);
};

} 

#endif 
