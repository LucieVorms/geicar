#include "space_environment.hpp"

#include <functional>
#include <cmath>

using namespace std;
using placeholders::_1;

SpaceEnvironment::SpaceEnvironment() : Node("space_environment_node"),
                                       max_depth(5.0), width(1.2), static_angle_offset(0.0) {
    subscription_laser_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&SpaceEnvironment::LaserScanCallback, this, _1));

    publisher_enough_width_space_ = this->create_publisher<std_msgs::msg::Bool>("enough_width_space", 10);

    RCLCPP_INFO(this->get_logger(), "Space environment node has been started.");
}

bool SpaceEnvironment::is_enough_space(const Scan &scan, const float offset) const {
    double rad = scan.angle_reference + offset;

    for (const auto &value: scan.values) {
        while (rad >= M_PI) rad -= 2 * M_PI;
        while (rad < -M_PI) rad += 2 * M_PI;
	//RCLCPP_INFO(this->get_logger(), "scan value  %f",value);
	
        if (isnormal(value) && value < max_depth && value > 0.45 && rad < 0 && std::abs(value * std::cos(rad)) * 2 < width){            
		//RCLCPP_INFO(this->get_logger(), "isnormal %f",value);
		return false;
	}

        rad += scan.angle_increment;
    }
    return true;
}

bool SpaceEnvironment::is_enough_space_in_range(const Scan &scan, const float max_abs_angle) const {
    if (is_enough_space(scan, 0.0)) return true;
    float offset = scan.angle_increment;
    while (offset <= max_abs_angle) {
        if (is_enough_space(scan, offset)) return true;
        if (is_enough_space(scan, -offset)) return true;
        offset += scan.angle_increment;
    }
    return false;
}

void SpaceEnvironment::LaserScanCallback(const sensor_msgs::msg::LaserScan &msg) const {
    const auto scan = Scan{msg.angle_min, msg.angle_increment, msg.ranges};
    std_msgs::msg::Bool msg_out;
    msg_out.data = is_enough_space_in_range(scan, (M_PI*2.) /(9.));

    publisher_enough_width_space_->publish(msg_out);
}

int main(const int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    spin(std::make_shared<SpaceEnvironment>());
    rclcpp::shutdown();
    return 0;
}
