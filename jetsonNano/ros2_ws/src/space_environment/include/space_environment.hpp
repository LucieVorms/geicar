#ifndef SPACE_ENVIRONMENT_H
#define SPACE_ENVIRONMENT_H

#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/u_int8.hpp>

typedef struct {
    const float angle_reference;
    const float angle_increment;
    const std::vector<float> values;
} Scan;


class SpaceEnvironment final : public rclcpp::Node {
public:
    SpaceEnvironment();

    bool is_enough_space(const Scan &scan, float offset) const;

private:
    float max_depth;
    float width;
    float static_angle_offset;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscription;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr enough_space_publisher;

    void scan_space(const std::shared_ptr<const sensor_msgs::msg::LaserScan> &msg) const;
};

#endif //SPACE_ENVIRONMENT_H
