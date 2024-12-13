#include <algorithm>
#include <functional>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

/* INTERFACE MESSAGES */
#include "interfaces/msg/motors_order.hpp"
#include "interfaces/msg/motors_feedback.hpp"
#include "interfaces/msg/car_motion_order.hpp"

/* CONFIGURATION */
#define PERIOD_UPDATE_CMD 1ms 
#define TOPIC_QUEUE_SIZE  10

#define DEBUG_SPEED_0     1
#undef  DEBUG_SPEED_0     

using namespace std;
using placeholders::_1;

class motors_control : public rclcpp::Node {
public:
    motors_control()
    : Node("motors_control_node")
    {
	car_speed_ = 0;
	front_wheel_rotation_ = 0;
        leftRearPwmCmd_ = rightRearPwmCmd_ = steeringPwmCmd_ = 50;
	subscription_car_motion_order_ = this->create_subscription<interfaces::msg::CarMotionOrder>(
        "car_motion_order", 10, std::bind(&motors_control::carMotionOrderCallback, this, _1));

        subscription_motors_feedback_ = this->create_subscription<interfaces::msg::MotorsFeedback>(
        "motors_feedback", 10, std::bind(&motors_control::motorsFeedbackCallback, this, _1));

        publisher_motors_order_ = this->create_publisher<interfaces::msg::MotorsOrder>("motors_order", 10);

        timer_ = this->create_wall_timer(PERIOD_UPDATE_CMD, std::bind(&motors_control::controlLoop, this));
        
	RCLCPP_INFO(this->get_logger(), "motors_control_node READY");
    }

private:

// TBD: create a config file for this :
#define CAR_MAX_SPEED            10.0f    // ?? 	m/s
#define FRONT_WHEEL_MAX_ROTATION 30.0f    // °

    void motorsFeedbackCallback(const interfaces::msg::MotorsFeedback & motorsFeedback){
	currentFrontWheelRotation_ = motorsFeedback.steering_angle * FRONT_WHEEL_MAX_ROTATION;
    }

    /*
       Update car motion order
     */
    void carMotionOrderCallback(const interfaces::msg::CarMotionOrder & carMotionOrder){
     	car_speed_            = carMotionOrder.car_speed;
     	front_wheel_rotation_ = carMotionOrder.front_wheel_rotation;
	// assert (abs(car_speed) <= CAR_MAX_SPEED);
	// assert (abs(front_wheel_rotation) <= FRONT_WHEEL_MAX_ROTATION);
    }
	
    /*
       Apply the local motor's pwm orders to the motors 
       each 1 ms (1KHz control loop)
     */
    void controlLoop() {   
	float diff_front_wheel_rotation = (front_wheel_rotation_ - currentFrontWheelRotation_); 
	// assert (abs(diff_front_wheel_rotation) <= 2*FRONT_WHEEL_MAX_ROTATION);

	// Rear wheels
	leftRearPwmCmd_ = rightRearPwmCmd_ = 50 + static_cast<int8_t>(50 * car_speed_/CAR_MAX_SPEED);
	// Front wheel
	steeringPwmCmd_ = 50 + std::clamp(static_cast<int>(50 * diff_front_wheel_rotation /FRONT_WHEEL_MAX_ROTATION), -50, 50);
	// Note: Some angles will be updated in more than one step.
        // Eg : diff_front_wheel_rotation = 50 °
	// step 1 : currentFrontWheelRotation_ += 30° (Max), diff_front_wheel_rotation = 20°
	// step 2 : currentFrontWheelRotation_ += 20°      , diff_front_wheel_rotation =  0°   

        auto motorsOrder = interfaces::msg::MotorsOrder();

#ifdef DEBUG_SPEED_0
	motorsOrder.left_rear_pwm  = 50;
        motorsOrder.right_rear_pwm = 50;
        motorsOrder.steering_pwm   = 50;
#else
        motorsOrder.left_rear_pwm  = leftRearPwmCmd_;
        motorsOrder.right_rear_pwm = rightRearPwmCmd_;
        motorsOrder.steering_pwm   = steeringPwmCmd_;
#endif  // DEBUG_SPEED_0        

	publisher_motors_order_->publish(motorsOrder);
    }

    // Car motion order
    float car_speed_;
    float front_wheel_rotation_; 

    // Motors pwm (duty cycle) order
    uint8_t leftRearPwmCmd_;          // 0...100
    uint8_t rightRearPwmCmd_;         // 0...100
    uint8_t steeringPwmCmd_;          // 0...100

    // Motors feedback
    float currentFrontWheelRotation_;     

    //Publishers
    rclcpp::Publisher<interfaces::msg::MotorsOrder>::SharedPtr publisher_motors_order_;

    //Subscribers
    rclcpp::Subscription<interfaces::msg::CarMotionOrder>::SharedPtr subscription_car_motion_order_;
    rclcpp::Subscription<interfaces::msg::MotorsFeedback>::SharedPtr subscription_motors_feedback_;

    //Timer
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<motors_control>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
