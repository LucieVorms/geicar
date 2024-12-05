#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std;

class motors_control : public rclcpp::Node {
public:
    motors_control()
    : Node("motors_control_node")
    {
        RCLCPP_INFO(this->get_logger(), "motors_control_node READY");
    }

};


int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<motors_control>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
