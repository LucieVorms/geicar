#ifndef SPACE_ENVIRONMENT_H
#define SPACE_ENVIRONMENT_H

#include <space_environment_debug.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/detail/laser_scan__struct.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <rclcpp/executors.hpp>


#define SPACE_RADIUS 0.5
#define SPACE_DEPTH 2.0
#define ANGLE_OFFSET 0.0


class SpaceEnvironment : public rclcpp::Node {
public:
    SpaceEnvironment();

private:
    SpaceEnvironmentDebug *logger;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscription;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr enough_space_publisher;

    void scan_space(const std::shared_ptr<const sensor_msgs::msg::LaserScan> &msg) const;
};

#endif //SPACE_ENVIRONMENT_H
