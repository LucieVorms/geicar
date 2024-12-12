#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>

#include "interfaces/msg/motors_order.hpp"
#include "interfaces/msg/motors_feedback.hpp"
#include "interfaces/msg/steering_calibration.hpp"
#include "interfaces/msg/joystick_order.hpp"
#include "interfaces/msg/gnss_status.hpp"
#include "interfaces/msg/vehicle_speed.hpp"
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
        // Initialize variables
        start = false;
        mode = 0;
        requestedThrottle = 0;
        requestedSteerAngle = 0;
        reversing = false;
        rear_obstacle = false;
    
        // Create publishers
        publisher_can_= this->create_publisher<interfaces::msg::MotorsOrder>("motors_order", 10);
        publisher_steeringCalibration_ = this->create_publisher<interfaces::msg::SteeringCalibration>("steering_calibration", 10);
        publisher_vehicle_speed_ = this->create_publisher<interfaces::msg::VehicleSpeed>("vehicle_speed", 10);



        // Create subscriptions
        subscription_joystick_order_ = this->create_subscription<interfaces::msg::JoystickOrder>(
            "joystick_order", 10, std::bind(&car_control::joystickOrderCallback, this, _1));

        subscription_gnss_status_ = this->create_subscription<interfaces::msg::GnssStatus>(
            "gnss_status", 10, std::bind(&car_control::GnssStatusCallback, this, _1));

        subscription_motors_feedback_ = this->create_subscription<interfaces::msg::MotorsFeedback>(
            "motors_feedback", 10, std::bind(&car_control::motorsFeedbackCallback, this, _1));

        subscription_steering_calibration_ = this->create_subscription<interfaces::msg::SteeringCalibration>(
            "steering_calibration", 10, std::bind(&car_control::steeringCalibrationCallback, this, _1));
        
        // Create service
        server_calibration_ = this->create_service<std_srvs::srv::Empty>(
            "steering_calibration", std::bind(&car_control::steeringCalibration, this, std::placeholders::_1, std::placeholders::_2));
       
 
        subscription_obstacle_info_ = this->create_subscription<interfaces::msg::ObstacleInfo>(
            "obstacle_info", 10, std::bind(&car_control::ObstacleInfoCallback, this, _1));
  
       

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

        if (start && reversing && (this->now() - reverse_timer).nanoseconds() / 1e6 < REVERSE_DURATION) {
        
            if (!rear_obstacle) {
                leftRearPwmCmd = REVERSE_PWM;
                rightRearPwmCmd = REVERSE_PWM;
                steeringPwmCmd = STOP;
            } else {
                RCLCPP_WARN(this->get_logger(), "Obstacle detected at the Rear, Stopping the car");
                leftRearPwmCmd = STOP;
                rightRearPwmCmd = STOP;
                reversing = false; 
            } 
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

                if (abs(turn_angle) > 5){
	            //RCLCPP_WARN(this->get_logger(), "Je tourne, %2f",turn_angle);
                    steeringCmd(turn_angle/30 ,currentAngle, steeringPwmCmd);
                    leftRearPwmCmd = 65;
                    rightRearPwmCmd = 65;
                }else {
                    //RCLCPP_WARN(this->get_logger(), "Je tourne pas %2f",turn_angle);
                    steeringCmd(0 ,currentAngle, steeringPwmCmd);
                    leftRearPwmCmd = 75;
                    rightRearPwmCmd = 75;
                }

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

    bool rear_obstacle;
    bool reversing;  // Indique si la voiture est en marche arrière
    rclcpp::Time reverse_timer;  // Timer pour la marche arrière


    //Manual Mode variables (with joystick control)
    bool reverse;
    float requestedThrottle;
    float requestedSteerAngle;

    //Control variables
    uint8_t leftRearPwmCmd;
    uint8_t rightRearPwmCmd;
    uint8_t steeringPwmCmd;


    float turn_angle;

    //Publishers
    rclcpp::Publisher<interfaces::msg::MotorsOrder>::SharedPtr publisher_can_;
    rclcpp::Publisher<interfaces::msg::SteeringCalibration>::SharedPtr publisher_steeringCalibration_;
    rclcpp::Publisher<interfaces::msg::VehicleSpeed>::SharedPtr publisher_vehicle_speed_;

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
