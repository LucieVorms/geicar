#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>

#include "interfaces/msg/motors_order.hpp"
#include "interfaces/msg/motors_feedback.hpp"
#include "interfaces/msg/steering_calibration.hpp"
#include "interfaces/msg/joystick_order.hpp"

#include "interfaces/msg/vehicle_speed.hpp"

#include "interfaces/msg/ultrasonic.hpp"
#include "interfaces/msg/obstacle_info.hpp"
#include "std_msgs/msg/bool.hpp"  

#include "std_srvs/srv/empty.hpp"

#include "../include/car_control/steeringCmd.h"
#include "../include/car_control/propulsionCmd.h"
#include "../include/car_control/car_control_node.h"

using namespace std;
using placeholders::_1;
#define OBSTACLE_THRESHOLD 60   // Threshold for obstacle detection in centimeters
#define REVERSE_SPEED 20
class car_control : public rclcpp::Node {

public:
    car_control()
    : Node("car_control_node")

    {
        // Initialize variables
        start = false;
        mode = 0;
        requestedThrottle = 0;
        requestedSteerAngle = 0;
        obstacle_detected = false;
    
        // Create publishers
        publisher_can_= this->create_publisher<interfaces::msg::MotorsOrder>("motors_order", 10);
        publisher_steeringCalibration_ = this->create_publisher<interfaces::msg::SteeringCalibration>("steering_calibration", 10);

        publisher_vehicle_speed_ = this->create_publisher<interfaces::msg::VehicleSpeed>("vehicle_speed", 10);

        publisher_obstacle_info_ = this->create_publisher<interfaces::msg::ObstacleInfo>("ObstacleInfo", 10);


        // Create subscriptions
        subscription_joystick_order_ = this->create_subscription<interfaces::msg::JoystickOrder>(
            "joystick_order", 10, std::bind(&car_control::joystickOrderCallback, this, _1));
        subscription_motors_feedback_ = this->create_subscription<interfaces::msg::MotorsFeedback>(
            "motors_feedback", 10, std::bind(&car_control::motorsFeedbackCallback, this, _1));
        subscription_steering_calibration_ = this->create_subscription<interfaces::msg::SteeringCalibration>(

        "steering_calibration", 10, std::bind(&car_control::steeringCalibrationCallback, this, _1));
        
        // Create service
        server_calibration_ = this->create_service<std_srvs::srv::Empty>(
                            "steering_calibration", std::bind(&car_control::steeringCalibration, this, std::placeholders::_1, std::placeholders::_2));
       
        // 
        subscription_obstacle_detected_ = this->create_subscription<std_msgs::msg::Bool>(
            "obstacle_detected", 10, std::bind(&car_control::obstacleDetectedCallback, this, _1));
        subscription_ultrasonic_ = this->create_subscription<interfaces::msg::Ultrasonic>(
            "us_data", 10, std::bind(&car_control::ultrasonicCallback, this, _1));    
        
        // Create service for steering calibration
       

        // Create timer to update commands periodically
      
        timer_ = this->create_wall_timer(PERIOD_UPDATE_CMD, std::bind(&car_control::updateCmd, this));

        RCLCPP_INFO(this->get_logger(), "car_control_node READY");
    }

    
private:


    /* Callback to handle joystick orders */

     /* Update obstacledetection from obstacledetected topic [callback function]  :
    *
    * This function is called when a message is published on the "/obstacle_detected" topic
    * 
    */
     void obstacleDetectedCallback(const std_msgs::msg::Bool & msg) {
        auto obstacle_info_msg = interfaces::msg::ObstacleInfo();
        obstacle_info_msg.obstacle_detected = msg.data;

        if (msg.data) { // If an obstacle is detected
            std::string detected_sides; 
        
            // Determine which sides have obstacles
            if (front_left < OBSTACLE_THRESHOLD) detected_sides += "Avant Gauche, ";
            if (front_center < OBSTACLE_THRESHOLD) detected_sides += "Avant Centre, ";
            if (front_right < OBSTACLE_THRESHOLD) detected_sides += "Avant Droit, ";
            if (rear_left < OBSTACLE_THRESHOLD) detected_sides += "Arrière Gauche, ";
            if (rear_center < OBSTACLE_THRESHOLD) detected_sides += "Arrière Centre, ";
            if (rear_right < OBSTACLE_THRESHOLD) detected_sides += "Arrière Droit, ";

            if (front_left < OBSTACLE_THRESHOLD || front_center < OBSTACLE_THRESHOLD || front_right < OBSTACLE_THRESHOLD) {
                reversing = true; 
                reverse_timer = this->now(); 
                detected_sides += "Obstacle devant, ";
            }

            // Remove trailing comma and space if present
            if (!detected_sides.empty()) {
                detected_sides = detected_sides.substr(0, detected_sides.size() - 2);
            } else {
                detected_sides = "Aucun obstacle";
            }

            obstacle_info_msg.sides_detected = detected_sides;
        } else {
            obstacle_info_msg.sides_detected = "Aucun obstacle";
            reversing = false;
        }

    
        publisher_obstacle_info_->publish(obstacle_info_msg);
        obstacle_detected = msg.data;  
    }

    /* Callback to handle ultrasonic sensor data */
     void ultrasonicCallback(const interfaces::msg::Ultrasonic & ultrasonicMsg) {
        front_left = ultrasonicMsg.front_left;
        front_center = ultrasonicMsg.front_center;
        front_right = ultrasonicMsg.front_right;
        rear_left = ultrasonicMsg.rear_left;
        rear_center = ultrasonicMsg.rear_center;
        rear_right = ultrasonicMsg.rear_right;
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
        


        if (joyOrder.mode != mode && joyOrder.mode != -1){  // Handle mode changes

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
        
        // Update throttle and steer angle in manual mode
        if (mode == 0 && start){  
            requestedThrottle = joyOrder.throttle;
            requestedSteerAngle = joyOrder.steer;
            reverse = joyOrder.reverse;
        }
    }


    /* Calculate vehicle speed from RPM */
    float calculateSpeedFromRpm(float rpm, float wheelRadius, float gearRatio) {
        float speed_m_s = (2 * M_PI * wheelRadius * rpm) / (60.0 * gearRatio);  // Speed in m/s 
        return speed_m_s * 3.6; // Speed in km/h
    }


    /* Update currentAngle,leftand right wheel speed from motors feedback [callback function]  :
    *
    * This function is called when a message is published on the "/motors_feedback" topic
    * 
    */

    /* Callback to handle motor feedback */

    void motorsFeedbackCallback(const interfaces::msg::MotorsFeedback & motorsFeedback){
        currentAngle = motorsFeedback.steering_angle;

        const float wheelRadius = 0.10; // Wheel radius in meters
        const float gearRatio = 1.0;    // Gear ratio

        // Calculate wheel speeds
        float leftWheelSpeed = calculateSpeedFromRpm(motorsFeedback.left_rear_speed, wheelRadius, gearRatio);
        float rightWheelSpeed = calculateSpeedFromRpm(motorsFeedback.right_rear_speed, wheelRadius, gearRatio);

        // Average wheel speed
        actualSpeed = (leftWheelSpeed + rightWheelSpeed) / 2.0;
    }

   


    /* Periodic function to update motor commands */
    void updateCmd(){
        auto motorsOrder = interfaces::msg::MotorsOrder();

        if (reversing && (this->now() - reverse_timer).nanoseconds() / 1e6 < REVERSE_DURATION) {
        
            motorsOrder.left_rear_pwm = REVERSE_PWM;
            motorsOrder.right_rear_pwm = REVERSE_PWM;
            motorsOrder.steering_pwm = STOP; 
        }
        else if (!start || obstacle_detected){ 
            // Stop the car 
            leftRearPwmCmd = STOP;
            rightRearPwmCmd = STOP;
            steeringPwmCmd = STOP;
            

        }else{ 
            // Handle manual mode
            if (mode==0){
                manualPropulsionCmd(requestedThrottle, reverse, leftRearPwmCmd,rightRearPwmCmd);
                steeringCmd(requestedSteerAngle,currentAngle, steeringPwmCmd);
            } else if (mode==1){    //Autonomous Mode
                leftRearPwmCmd = 70;        
                rightRearPwmCmd = 70;
                steeringCmd(0 ,currentAngle, steeringPwmCmd);  // Center the wheels


                // Publish vehicle speed
                interfaces::msg::VehicleSpeed vehicleSpeed;
                

                // Negative speed if in reverse 
                if(leftRearPwmCmd < 50){
                    vehicleSpeed.actual_speed = -actualSpeed;
                }else {
                    vehicleSpeed.actual_speed = actualSpeed;
                }
                vehicleSpeed.speed_unit = "km/h";

                // Publish vehicle speed
                publisher_vehicle_speed_->publish(vehicleSpeed);

            }
        }

        // Send motor commands
        motorsOrder.left_rear_pwm = leftRearPwmCmd;
        motorsOrder.right_rear_pwm = rightRearPwmCmd;
        motorsOrder.steering_pwm = steeringPwmCmd;
        publisher_can_->publish(motorsOrder);
    }


    float calculateActualSpeed(uint8_t leftPwm, uint8_t rightPwm) {
        //Estimation of the speed
        float maxSpeed = 2.1;
        float averagePwm = (leftPwm + rightPwm) / 2.0;
        if (averagePwm < 50){
            return -((50 - averagePwm) / 50) * maxSpeed;
        } else {
            return ((averagePwm - 50)/(100-50))*maxSpeed;
        }        
    }



    /* Start the steering calibration process :
    *
    * Publish a calibration request on the "/steering_calibration" topic
    */

    /* Start the steering calibration process */

    void startSteeringCalibration(){
        auto calibrationMsg = interfaces::msg::SteeringCalibration();
        calibrationMsg.request = true;

        RCLCPP_INFO(this->get_logger(), "Sending calibration request .....");
        publisher_steeringCalibration_->publish(calibrationMsg);
    }


    /* Callback to handle the steering calibration process */
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
            mode = 0;
            start = false;  //Stop car
        
        } else if (calibrationMsg.status == -1){
            RCLCPP_ERROR(this->get_logger(), "Steering calibration [FAILED]");
            RCLCPP_INFO(this->get_logger(), "Switching to MANUAL Mode");
            mode = 0;
            start = false;  //Stop car
        }
    
    }
    
    // ---- Private variables ----

    //General variables
    bool start;
    int mode;    //0 : Manual    1 : Auto    2 : Calibration
    bool obstacle_detected;
    
    //Motors feedback variables
    float currentAngle;
    float actualSpeed;  

    //Manual Mode variables (with joystick control)
    bool reverse;
    float requestedThrottle;
    float requestedSteerAngle;

    //Control variables
    uint8_t leftRearPwmCmd;
    uint8_t rightRearPwmCmd;
    uint8_t steeringPwmCmd;

    //Us Sensors variables  
    int16_t front_left ;
    int16_t front_center ;
    int16_t front_right ;
    int16_t rear_left;
    int16_t rear_center ;
    int16_t rear_right ;

    //Publishers
    rclcpp::Publisher<interfaces::msg::MotorsOrder>::SharedPtr publisher_can_;
    rclcpp::Publisher<interfaces::msg::SteeringCalibration>::SharedPtr publisher_steeringCalibration_;

    rclcpp::Publisher<interfaces::msg::VehicleSpeed>::SharedPtr publisher_vehicle_speed_;

    rclcpp::Publisher<interfaces::msg::ObstacleInfo>::SharedPtr publisher_obstacle_info_;



    //Subscribers
    rclcpp::Subscription<interfaces::msg::JoystickOrder>::SharedPtr subscription_joystick_order_;
    rclcpp::Subscription<interfaces::msg::MotorsFeedback>::SharedPtr subscription_motors_feedback_;
    rclcpp::Subscription<interfaces::msg::SteeringCalibration>::SharedPtr subscription_steering_calibration_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_obstacle_detected_ ;
    rclcpp::Subscription<interfaces::msg::Ultrasonic>::SharedPtr subscription_ultrasonic_;

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
