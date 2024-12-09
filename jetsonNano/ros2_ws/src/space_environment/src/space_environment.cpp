#include "space_environment.hpp"


SpaceEnvironment::SpaceEnvironment() : Node("space_environment_node") {
    auto callback = [this](const std::shared_ptr<const sensor_msgs::msg::LaserScan> &msg) {
        scan_space(msg);
    };
    laser_scan_subscription = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, callback);
    enough_space_publisher = this->create_publisher<std_msgs::msg::UInt8>("/space_from_lidar", 10);

    max_depth = 6.0;
    width = 1.0;
    static_angle_offset = 0.0;

    RCLCPP_INFO(this->get_logger(), "Space environment node has been started.");
}

bool SpaceEnvironment::is_enough_space(const Scan &scan, const float offset) const {
    double rad = scan.angle_reference;
    const double field_of_view_offset = scan.angle_reference + static_angle_offset + offset;

    for (const auto &value: scan.values) {
        if (
            value < max_depth &&
            -M_PI / 2 < rad - field_of_view_offset &&
            rad - field_of_view_offset < M_PI / 2 &&
            value * std::cos(rad) * 2 > width
        )
            return false;

        rad += scan.angle_increment;
        if (rad > M_PI)
            rad -= 2 * M_PI;
        else if (rad < -M_PI)
            rad += 2 * M_PI;
    }
    return true;
}

void SpaceEnvironment::scan_space(const std::shared_ptr<const sensor_msgs::msg::LaserScan> &msg) const {
    const auto scan = Scan{msg->angle_min, msg->angle_increment, msg->ranges};
    std_msgs::msg::UInt8 msg_out;
    msg_out.data = is_enough_space(scan, 0.0);

    if (msg_out.data) {
        RCLCPP_INFO(this->get_logger(), "[X] True");
    } else {
        RCLCPP_INFO(this->get_logger(), "[ ] False");
    }
    enough_space_publisher->publish(msg_out);
}

int main(const int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    spin(std::make_shared<SpaceEnvironment>());
    rclcpp::shutdown();
    return 0;
}
