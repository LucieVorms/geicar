#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>

#include "interfaces/msg/motors_order.hpp"
#include "interfaces/msg/motors_feedback.hpp"
#include "interfaces/msg/steering_calibration.hpp"
#include "interfaces/msg/joystick_order.hpp"
#include "interfaces/msg/ultrasonic.hpp"
#include "interfaces/msg/obstacle_info.hpp"
#include "std_msgs/msg/bool.hpp"  


#include "std_srvs/srv/empty.hpp"

#include "../include/car_control/steeringCmd.h"
#include "../include/car_control/propulsionCmd.h"
#include "../include/car_control/car_control_node.h"

using namespace std;
using placeholders::_1;
#define OBSTACLE_THRESHOLD 50  

class car_control : public rclcpp::Node {

public:
    car_control()
    : Node("car_control_node")
    {
        start = false;
        mode = 0;
        requestedThrottle = 0;
        requestedSteerAngle = 0;
        obstacle_detected = false;
    

        publisher_can_= this->create_publisher<interfaces::msg::MotorsOrder>("motors_order", 10);

        publisher_steeringCalibration_ = this->create_publisher<interfaces::msg::SteeringCalibration>("steering_calibration", 10);

        
        publisher_obstacle_info_ = this->create_publisher<interfaces::msg::ObstacleInfo>("ObstacleInfo", 10);

        subscription_joystick_order_ = this->create_subscription<interfaces::msg::JoystickOrder>(
        "joystick_order", 10, std::bind(&car_control::joystickOrderCallback, this, _1));

        subscription_motors_feedback_ = this->create_subscription<interfaces::msg::MotorsFeedback>(
        "motors_feedback", 10, std::bind(&car_control::motorsFeedbackCallback, this, _1));

        subscription_steering_calibration_ = this->create_subscription<interfaces::msg::SteeringCalibration>(
        "steering_calibration", 10, std::bind(&car_control::steeringCalibrationCallback, this, _1));

        subscription_obstacle_detected_ = this->create_subscription<std_msgs::msg::Bool>(
            "obstacle_detected", 10, std::bind(&car_control::obstacleDetectedCallback, this, _1));

        // Abonnement pour les distances des capteurs à ultrasons
        subscription_ultrasonic_ = this->create_subscription<interfaces::msg::Ultrasonic>(
            "us_data", 10, std::bind(&car_control::ultrasonicCallback, this, _1));    
        

        server_calibration_ = this->create_service<std_srvs::srv::Empty>(
                            "steering_calibration", std::bind(&car_control::steeringCalibration, this, std::placeholders::_1, std::placeholders::_2));

        timer_ = this->create_wall_timer(PERIOD_UPDATE_CMD, std::bind(&car_control::updateCmd, this));

        
        RCLCPP_INFO(this->get_logger(), "car_control_node READY");
    }

    
private:

     /* Update obstacledetection from obstacledetected topic [callback function]  :
    *
    * This function is called when a message is published on the "/obstacle_detected" topic
    * 
    */
     void obstacleDetectedCallback(const std_msgs::msg::Bool & msg) {

        auto obstacle_info_msg = interfaces::msg::ObstacleInfo();

        obstacle_info_msg.obstacle_detected = msg.data;

        if (msg.data) {
            std::string detected_sides; 
            

            if (front_left < OBSTACLE_THRESHOLD) detected_sides += "Avant Gauche, ";
            if (front_center < OBSTACLE_THRESHOLD) detected_sides += "Avant Centre, ";
            if (front_right < OBSTACLE_THRESHOLD) detected_sides += "Avant Droit, ";
            if (rear_left < OBSTACLE_THRESHOLD) detected_sides += "Arrière Gauche, ";
            if (rear_center < OBSTACLE_THRESHOLD) detected_sides += "Arrière Centre, ";
            if (rear_right < OBSTACLE_THRESHOLD) detected_sides += "Arrière Droit, ";

            // Supprimer la dernière virgule et espace, si nécessaire
            if (!detected_sides.empty()) {
                detected_sides = detected_sides.substr(0, detected_sides.size() - 2);
            } else {
                detected_sides = "Aucun obstacle";
            }

            obstacle_info_msg.sides_detected = detected_sides;
        } else {
            obstacle_info_msg.sides_detected = "Aucun obstacle";
        }

    
        publisher_obstacle_info_->publish(obstacle_info_msg);


        obstacle_detected = msg.data;  
        /*
        if (obstacle_detected) {
            RCLCPP_INFO(this->get_logger(), "Obstacle detected! Stopping the car.");
        } else {
            RCLCPP_INFO(this->get_logger(), "No obstacle detected. Resuming the car.");
        }*/
    }

     void ultrasonicCallback(const interfaces::msg::Ultrasonic & ultrasonicMsg) {

        // Accéder aux distances des capteurs
        front_left = ultrasonicMsg.front_left;
        front_center = ultrasonicMsg.front_center;
        front_right = ultrasonicMsg.front_right;
        rear_left = ultrasonicMsg.rear_left;
        rear_center = ultrasonicMsg.rear_center;
        rear_right = ultrasonicMsg.rear_right;

        /*
        if(obstacle_detected){
            if (front_left < OBSTACLE_THRESHOLD) {
                RCLCPP_INFO(this->get_logger(), "Obstacle détecté à %d cm - Avant Gauche", front_left);
            }
            if (front_center < OBSTACLE_THRESHOLD) {
                RCLCPP_INFO(this->get_logger(), "Obstacle détecté à %d cm - Avant Centre", front_center);
            }
            if (front_right < OBSTACLE_THRESHOLD) {
                RCLCPP_INFO(this->get_logger(), "Obstacle détecté à %d cm - Avant Droit", front_right);
            }
            if (rear_left < OBSTACLE_THRESHOLD) {
                RCLCPP_INFO(this->get_logger(), "Obstacle détecté à %d cm - Arrière Gauche", rear_left);
            }
            if (rear_center < OBSTACLE_THRESHOLD) {
                RCLCPP_INFO(this->get_logger(), "Obstacle détecté à %d cm - Arrière Centre", rear_center);
            }
            if (rear_right < OBSTACLE_THRESHOLD) {
                RCLCPP_INFO(this->get_logger(), "Obstacle détecté à %d cm - Arrière Droit", rear_right);
            }
        }*/
    }




    /* Update start, mode, requestedThrottle, requestedSteerAngle and reverse from joystick order [callback function]  :
    *
    * This function is called when a message is published on the "/joystick_order" topic
    * 
    */
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
    * This function is called when a message is published on the "/motors_feedback" topic
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
    void updateCmd(){

        auto motorsOrder = interfaces::msg::MotorsOrder();

        if (!start || obstacle_detected){    //Car stopped 
            leftRearPwmCmd = STOP;
            rightRearPwmCmd = STOP;
            steeringPwmCmd = STOP;


        }else{ //Car started

            //Manual Mode
            if (mode==0){
                
                manualPropulsionCmd(requestedThrottle, reverse, leftRearPwmCmd,rightRearPwmCmd);

                steeringCmd(requestedSteerAngle,currentAngle, steeringPwmCmd);


            //Autonomous Mode
            } else if (mode==1){
                //...
            }
        }


        //Send order to motors
        motorsOrder.left_rear_pwm = leftRearPwmCmd;
        motorsOrder.right_rear_pwm = rightRearPwmCmd;
        motorsOrder.steering_pwm = steeringPwmCmd;

        publisher_can_->publish(motorsOrder);
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
    int mode;    //0 : Manual    1 : Auto    2 : Calibration
    bool obstacle_detected;
    
    //Motors feedback variables
    float currentAngle;

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