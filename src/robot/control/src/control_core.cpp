#include "control_core.hpp"

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger), lookahead_distance_(1.0), linear_speed_(0.5) {}

void ControlCore::processPath(const nav_msgs::msg::Path::SharedPtr msg) {
    current_path_ = msg;
}

void ControlCore::processOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_odometry_ = msg;
}

geometry_msgs::msg::Twist ControlCore::generateCommand() {
    geometry_msgs::msg::Twist cmd_vel;

    if (!current_path_ || !current_odometry_) {
        return cmd_vel;
    }

    double robot_x = current_odometry_->pose.pose.position.x;
    double robot_y = current_odometry_->pose.pose.position.y;

    auto lookahead_point = findLookaheadPoint(robot_x, robot_y);
    if (!lookahead_point) {
        return cmd_vel;
    }

    double steering_angle = calculateSteeringAngle(robot_x, robot_y, *lookahead_point);
    cmd_vel.linear.x = linear_speed_;
    cmd_vel.angular.z = 2.0 * steering_angle;

    return cmd_vel;
}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(double robot_x, double robot_y) {
    if (!current_path_->poses.empty()) {
        for (size_t i = 0; i < current_path_->poses.size(); ++i) {
            double dist = computeDistance(robot_x, robot_y, current_path_->poses[i].pose.position.x, current_path_->poses[i].pose.position.y);
            if (dist >= lookahead_distance_) {
                return current_path_->poses[i];
            }
        }
    }
    return std::nullopt;
}

double ControlCore::calculateSteeringAngle(double robot_x, double robot_y, const geometry_msgs::msg::PoseStamped& lookahead_point) {
    double angle_to_target = atan2(lookahead_point.pose.position.y - robot_y,
                                   lookahead_point.pose.position.x - robot_x);
    double current_yaw = extractYaw(current_odometry_->pose.pose.orientation);
    return angle_to_target - current_yaw;
}

double ControlCore::computeDistance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
    double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
    double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

}  
