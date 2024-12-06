#ifndef SPACE_ENVIRONMENT_H
#define SPACE_ENVIRONMENT_H


#define SPACE_RADIUS 0.5
#define SPACE_DEPTH 2.0
#define ANGLE_OFFSET 0.0


#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <interfaces/msg/space_environment_data.hpp>


class SpaceEnvironment : public rclcpp::Node {
public:
    SpaceEnvironment();

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscription;
    rclcpp::Publisher<interfaces::msg::SpaceEnvironmentData>::SharedPtr enough_space_publisher;

    void scan_space(const std::shared_ptr<const sensor_msgs::msg::LaserScan> &msg) const;
};

#endif //SPACE_ENVIRONMENT_H
