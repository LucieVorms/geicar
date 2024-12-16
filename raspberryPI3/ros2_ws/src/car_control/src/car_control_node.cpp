#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>

//#include "interfaces/msg/motors_order.hpp"
#include "interfaces/msg/motors_feedback.hpp"

#include "interfaces/msg/car_motion_order.hpp"
#include "interfaces/msg/steering_calibration.hpp"
#include "interfaces/msg/joystick_order.hpp"
#include "interfaces/msg/gnss_status.hpp"
#include "interfaces/msg/obstacle_info.hpp"
#include "std_msgs/msg/bool.hpp"  


#include "std_srvs/srv/empty.hpp"

#include "../include/car_control/steeringCmd.h"
#include "../include/car_control/propulsionCmd.h"
#include "../include/car_control/car_control_node.h"

using namespace std;
using placeholders::_1;
#define REVERSE_DURATION 2000  
#define REVERSE_PWM 30  

class car_control : public rclcpp::Node {

public:
    car_control()
    : Node("car_control_node")
    {
        start = false;
        mode = 0;
        requestedThrottle = 0;
        requestedSteerAngle = 0;

	reverse = false;
	carSpeed = frontWheelRotation = 0.0f;
	
        reversing = false;
        rear_obstacle = false;
    
        // Create publishers
        publisher_car_motion_order_   = this->create_publisher<interfaces::msg::CarMotionOrder>("car_motion_order", 10);       

	publisher_steeringCalibration_ = this->create_publisher<interfaces::msg::SteeringCalibration>("steering_calibration", 10);
      	
	// Create subscribers 
        subscription_joystick_order_ = this->create_subscription<interfaces::msg::JoystickOrder>(
            "joystick_order", 10, std::bind(&car_control::joystickOrderCallback, this, _1));

        subscription_gnss_status_ = this->create_subscription<interfaces::msg::GnssStatus>(
            "gnss_status", 10, std::bind(&car_control::GnssStatusCallback, this, _1));

        subscription_motors_feedback_ = this->create_subscription<interfaces::msg::MotorsFeedback>(
            "motors_feedback", 10, std::bind(&car_control::motorsFeedbackCallback, this, _1));

        subscription_steering_calibration_ = this->create_subscription<interfaces::msg::SteeringCalibration>(
            "steering_calibration", 10, std::bind(&car_control::steeringCalibrationCallback, this, _1));
	
        

        
        subscription_obstacle_info_ = this->create_subscription<interfaces::msg::ObstacleInfo>(
            "obstacle_info", 10, std::bind(&car_control::ObstacleInfoCallback, this, _1));
       
	subscription_steering_calibration_ = this->create_subscription<interfaces::msg::SteeringCalibration>(
		"steering_calibration", 10, std::bind(&car_control::steeringCalibrationCallback, this, _1));
	
	// Create calibration server 
	server_calibration_ = this->create_service<std_srvs::srv::Empty>( "steering_calibration", std::bind(&car_control::steeringCalibration, this, std::placeholders::_1, std::placeholders::_2));

	timer_ = this->create_wall_timer(PERIOD_UPDATE_CMD, std::bind(&car_control::updateCmd, this));
	        
        RCLCPP_INFO(this->get_logger(), "car_control_node READY");
}
    
private:

    /* Update start, mode, requestedThrottle, requestedSteerAngle and reverse from joystick order [callback function]  :
    *
    * This function is called when a message is published on the "/joystick_order" topic
    * 
    */
    void ObstacleInfoCallback(const interfaces::msg::ObstacleInfo & Obstaclemsg) {

        if (Obstaclemsg.obstacle_detected) { // If an obstacle is detected
            if (Obstaclemsg.critical_detected) {
                reversing = true; 
                reverse_timer = this->now(); 
                if(Obstaclemsg.rear_obstacles_detected){
                    rear_obstacle = true;
                }else {
                    rear_obstacle = false;
                }
            }
        } else {
            reversing = false;
        }
        obstacle_detected = Obstaclemsg.obstacle_detected; 
    }

     /* Callback to handle ultrasonic sensor data */
     void GnssStatusCallback(const interfaces::msg::GnssStatus & gnssMsg) {
        turn_angle = gnssMsg.turn_angle;
    }

    /* Callback to handle joystick commands */
    void joystickOrderCallback(const interfaces::msg::JoystickOrder & joyOrder) {

        if (joyOrder.start != start){
            start = joyOrder.start;

            if (start)
                RCLCPP_INFO(this->get_logger(), "START");
            else 
                RCLCPP_INFO(this->get_logger(), "STOP");
        }
        

        if (joyOrder.mode != mode && joyOrder.mode != -1){ //if mode change
            mode = joyOrder.mode;

            if (mode==0){
                RCLCPP_INFO(this->get_logger(), "Switching to MANUAL Mode");
            }else if (mode==1){
                RCLCPP_INFO(this->get_logger(), "Switching to AUTONOMOUS Mode");
            }else if (mode==2){
                RCLCPP_INFO(this->get_logger(), "Switching to STEERING CALIBRATION Mode");
                startSteeringCalibration();
            }
        }
        
        if (mode == 0 && start){  //if manual mode -> update requestedThrottle, requestedSteerAngle and reverse from joystick order
            requestedThrottle = joyOrder.throttle;
            requestedSteerAngle = joyOrder.steer;
            reverse = joyOrder.reverse;
        }
    }

    /* Update currentAngle from motors feedback [callback function]  :
    *
    *  This function is called when a message is published on the "/motors_feedback" topic
    * 
    */

     void motorsFeedbackCallback(const interfaces::msg::MotorsFeedback & motorsFeedback){
         currentAngle = motorsFeedback.steering_angle;
     }
    

    /* Update PWM commands : leftRearPwmCmd, rightRearPwmCmd, steeringPwmCmd
    *
    * This function is called periodically by the timer [see PERIOD_UPDATE_CMD in "car_control_node.h"]
    * 
    * In MANUAL mode, the commands depends on :
    * - requestedThrottle, reverse, requestedSteerAngle [from joystick orders]
    * - currentAngle [from motors feedback]
    */
     
// TBD: create a config file for this :
#define CAR_MAX_SPEED             10.0f    // ?? 	m/s
#define FRONT_WHEEL_MAX_ROTATION  30.0f    // °

    void updateCmd(){
        
	if (start && reversing && (this->now() - reverse_timer).nanoseconds() / 1e6 < REVERSE_DURATION) {
        
            if (!rear_obstacle) {	
		carSpeed = -(CAR_MAX_SPEED/2);
		frontWheelRotation = 0;
            } else {
                RCLCPP_WARN(this->get_logger(), "Obstacle detected at the Rear, Stopping the car");
		carSpeed = 0; 
		frontWheelRotation = 0;
		reversing = false;
            } 
        } else if (!start || obstacle_detected){ 
            // Stop the car 
		carSpeed = 0; 
		frontWheelRotation = 0;
        } else { //Car started
            //Manual Mode
            if (mode==0){	
		// Denormalize requested speed
		float  reqCarSpeed = requestedThrottle * CAR_MAX_SPEED;
		int    carSpeedSign = reverse ? -1 : 1;
		carSpeed = carSpeedSign * reqCarSpeed;
               
	       	// Denormalize requested throttle
		frontWheelRotation = requestedSteerAngle * FRONT_WHEEL_MAX_ROTATION; 


	   //Autonomous Mode
            } else if (mode==1){
                carSpeed = (CAR_MAX_SPEED/2);
                frontWheelRotation = 0;
            }
        }

        auto carMotionOrder = interfaces::msg::CarMotionOrder();
	carMotionOrder.car_speed  = carSpeed;
	carMotionOrder.front_wheel_rotation = frontWheelRotation;

        publisher_car_motion_order_->publish(carMotionOrder);
    }


    /* Start the steering calibration process :
    *
    * Publish a calibration request on the "/steering_calibration" topic
    */
    void startSteeringCalibration(){

        auto calibrationMsg = interfaces::msg::SteeringCalibration();
        calibrationMsg.request = true;

        RCLCPP_INFO(this->get_logger(), "Sending calibration request .....");
        publisher_steeringCalibration_->publish(calibrationMsg);
    }


    /* Function called by "steering_calibration" service
    * 1. Switch to calibration mode
    * 2. Call startSteeringCalibration function
    */
    void steeringCalibration([[maybe_unused]] std_srvs::srv::Empty::Request::SharedPtr req,
                            [[maybe_unused]] std_srvs::srv::Empty::Response::SharedPtr res)
    {

        mode = 2;    //Switch to calibration mode
        RCLCPP_WARN(this->get_logger(), "Switching to STEERING CALIBRATION Mode");
        startSteeringCalibration();
    }
    

    /* Manage steering calibration process [callback function]  :
    *
    * This function is called when a message is published on the "/steering_calibration" topic
    */
    void steeringCalibrationCallback (const interfaces::msg::SteeringCalibration & calibrationMsg){

        if (calibrationMsg.in_progress == true && calibrationMsg.user_need == false){
        RCLCPP_INFO(this->get_logger(), "Steering Calibration in progress, please wait ....");

        } else if (calibrationMsg.in_progress == true && calibrationMsg.user_need == true){
            RCLCPP_WARN(this->get_logger(), "Please use the buttons (L/R) to center the steering wheels.\nThen, press the blue button on the NucleoF103 to continue");
        
        } else if (calibrationMsg.status == 1){
            RCLCPP_INFO(this->get_logger(), "Steering calibration [SUCCESS]");
            RCLCPP_INFO(this->get_logger(), "Switching to MANUAL Mode");
            mode = 0;    //Switch to manual mode
            start = false;  //Stop car
        
        } else if (calibrationMsg.status == -1){
            RCLCPP_ERROR(this->get_logger(), "Steering calibration [FAILED]");
            RCLCPP_INFO(this->get_logger(), "Switching to MANUAL Mode");
            mode = 0;    //Switch to manual mode
            start = false;  //Stop car
        }
    
    }
    
    // ---- Private variables ----

    //General variables
    bool start;
    int  mode;    //0 : Manual    1 : Auto    2 : Calibration
    
    //Motors feedback variables
    float currentAngle;
    float actualSpeed;  

    bool rear_obstacle;
    bool obstacle_detected;
    bool reversing;  // Indique si la voiture est en marche arrière
    rclcpp::Time reverse_timer;  // Timer pour la marche arrière


    //Manual Mode variables (with joystick control)
    bool  reverse;
    float requestedThrottle;
    float requestedSteerAngle;

    //Control variables
    float carSpeed;
    float frontWheelRotation;

    float turn_angle;

    //Publishers
    rclcpp::Publisher<interfaces::msg::CarMotionOrder>::SharedPtr      publisher_car_motion_order_;
    rclcpp::Publisher<interfaces::msg::SteeringCalibration>::SharedPtr publisher_steeringCalibration_;

    //Subscribers

    rclcpp::Subscription<interfaces::msg::GnssStatus>::SharedPtr subscription_gnss_status_;
    rclcpp::Subscription<interfaces::msg::JoystickOrder>::SharedPtr subscription_joystick_order_;
    rclcpp::Subscription<interfaces::msg::MotorsFeedback>::SharedPtr subscription_motors_feedback_;
    rclcpp::Subscription<interfaces::msg::SteeringCalibration>::SharedPtr subscription_steering_calibration_;
    rclcpp::Subscription<interfaces::msg::ObstacleInfo>::SharedPtr subscription_obstacle_info_;

    //Timer
    rclcpp::TimerBase::SharedPtr timer_;

    //Steering calibration Service
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr server_calibration_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<car_control>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
