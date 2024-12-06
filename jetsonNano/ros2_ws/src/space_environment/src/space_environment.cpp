#include "space_environment.hpp"


SpaceEnvironment::SpaceEnvironment() : Node("space_environment_node") {
    auto callback = [this](const std::shared_ptr<const sensor_msgs::msg::LaserScan> &msg) {
        scan_space(msg);
    };
    laser_scan_subscription = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, callback);
    enough_space_publisher = this->create_publisher<interfaces::msg::SpaceEnvironmentData>("/space_from_lidar", 10);

    RCLCPP_INFO(this->get_logger(), "Space environment node has been started.");
}

void SpaceEnvironment::scan_space(const std::shared_ptr<const sensor_msgs::msg::LaserScan> &msg) const {
    double rad = msg->angle_min + ANGLE_OFFSET;
    interfaces::msg::SpaceEnvironmentData msg_out;

    for (const auto &range: msg->ranges) {
        if (range >= SPACE_DEPTH) {
        } else if (const auto space_radius = SPACE_RADIUS / std::cos(rad); range < space_radius) {
            RCLCPP_INFO(this->get_logger(), "YA PLUS DE PLACE :/");
            msg_out.have_enough_space = 0;
            return;
        }
        rad += msg->angle_increment;
    }

    RCLCPP_INFO(this->get_logger(), "YA MAX DE SPACE FRER");
    msg_out.have_enough_space = 1;
    enough_space_publisher->publish(msg_out);
}

int main(const int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    spin(std::make_shared<SpaceEnvironment>());
    rclcpp::shutdown();
    return 0;
}
