#ifndef SPACE_ENVIRONMENT_H
#define SPACE_ENVIRONMENT_H

#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>

typedef struct {
    const float angle_reference;
    const float angle_increment;
    const std::vector<float> values;
} Scan;


class SpaceEnvironment final : public rclcpp::Node {
public:
    SpaceEnvironment();

private:
    float max_depth;
    float width;
    float static_angle_offset;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_laser_scan_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_enough_width_space_;

    bool is_enough_space(const Scan &scan, float offset) const;

    void LaserScanCallback(const sensor_msgs::msg::LaserScan &msg) const;
};

#endif //SPACE_ENVIRONMENT_H