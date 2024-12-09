#include "space_environment.hpp"


SpaceEnvironment::SpaceEnvironment() : Node("space_environment") {
    auto callback = [this](const std::shared_ptr<const sensor_msgs::msg::LaserScan> &msg) {
        scan_space(msg);
    };
    laser_scan_subscription = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, callback);
    enough_space_publisher = this->create_publisher<std_msgs::msg::UInt8>("/have_enough_space", 10);
    //
    // RCLCPP_INFO(this->get_logger(), "SpaceEnvironment is Ready");
    //
    // logger = new SpaceEnvironmentDebug(this->get_logger());
}

void SpaceEnvironment::scan_space(const std::shared_ptr<const sensor_msgs::msg::LaserScan> &msg) const {
    double rad = msg->angle_min + ANGLE_OFFSET;
    bool enough_space = true;
    logger->log_new_scan();
    for (const auto &range: msg->ranges) {
        // logger->log_init(rad, range);
        if (range >= SPACE_DEPTH) {
            // logger->log_space(true, SPACE_DEPTH);
        } else if (const auto space_radius = SPACE_RADIUS / std::cos(rad); range < space_radius) {
            // logger->log_space(false, SPACE_DEPTH);
            // logger->log_space(false, space_radius);
            enough_space = false;
            break;
        } else {
            // logger->log_space(false, SPACE_DEPTH);
            // logger->log_space(true, space_radius);
        }
        rad += msg->angle_increment;
    }

    std_msgs::msg::UInt8 msg_out;
    msg_out.data = enough_space ? 1 : 0;
    enough_space_publisher->publish(msg_out);
}

int main(const int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    spin(std::make_shared<SpaceEnvironment>());
    rclcpp::shutdown();
    return 0;
}
