#include "space_environment.hpp"


SpaceEnvironment::SpaceEnvironment() : Node("space_environment") {
    auto callback = [this](const std::shared_ptr<const sensor_msgs::msg::LaserScan> &msg) {
        scan_space(msg);
    };
    laser_scan_subscription = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, callback);
    enough_space_publisher = this->create_publisher<interfaces::msg::SpaceData>("/space_from_lidar", 10);
}

void SpaceEnvironment::scan_space(const std::shared_ptr<const sensor_msgs::msg::LaserScan> &msg) const {
    double rad = msg->angle_min + ANGLE_OFFSET;
    bool enough_space = true;
    for (const auto &range: msg->ranges) {
        if (range >= SPACE_DEPTH) {
        } else if (const auto space_radius = SPACE_RADIUS / std::cos(rad); range < space_radius) {
            enough_space = false;
            break;
        }
        rad += msg->angle_increment;
    }

    interfaces::msg::SpaceData msg_out;
    msg_out.have_enough_space = enough_space ? 1 : 0;
    enough_space_publisher->publish(msg_out);
}

int main(const int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    spin(std::make_shared<SpaceEnvironment>());
    rclcpp::shutdown();
    return 0;
}
