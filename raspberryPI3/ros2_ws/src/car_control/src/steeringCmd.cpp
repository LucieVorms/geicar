#include "../include/car_control/steeringCmd.h"

// Define the maximum angle for normalization
#define MAX_ANGLE 30.0

// Return the Pwm command to reach the angle passed in argument
int steeringCmd(float requestedSteerAngle, float currentSteerAngle, uint8_t & steeringPwmCmd){

    float errorAngle = requestedSteerAngle - currentSteerAngle;

    // Command's calculation
    if (abs(errorAngle) < TOLERANCE_ANGLE){
        steeringPwmCmd = 50; // Neutral position
    }
    else { 

        // Ensure the normalized error is within the range [-1, 1]
        if (errorAngle > 1.0) errorAngle = 1.0;
        if (errorAngle < -1.0) errorAngle = -1.0;

        // Calculate the PWM command based on the normalized error
        steeringPwmCmd = 50 + (normalizedError * 50); // 50 is the neutral position

        if (steeringPwmCmd > 100.0) steeringPwmCmd = 100.0;
        if (steeringPwmCmd < 0) steeringPwmCmd = 0;

    }

    return errorAngle;
}




/*
// Define the maximum angle for normalization
#define MAX_ANGLE 30.0

// Define the ranges and corresponding PWM values
#define PWM_NEUTRAL 50
#define PWM_LEFT_MAX 0
#define PWM_RIGHT_MAX 100

// Define angle ranges
#define ANGLE_TOLERANCE 0.08
#define ANGLE_SMALL 0.2
#define ANGLE_MEDIUM 0.5
#define ANGLE_MEDIUM 0.7


// Return the Pwm command to reach the angle passed in argument
int steeringCmd(float requestedSteerAngle, float currentSteerAngle, uint8_t & steeringPwmCmd){

    float errorAngle = requestedSteerAngle - currentSteerAngle;

    // Command's calculation based on ranges
    if (abs(errorAngle) < ANGLE_TOLERANCE){
        steeringPwmCmd = PWM_NEUTRAL; // Neutral position
    }
    else if (errorAngle >= ANGLE_TOLERANCE && errorAngle < ANGLE_SMALL){
        steeringPwmCmd = PWM_NEUTRAL + 10; // Small right turn
    }
    else if (errorAngle >= ANGLE_SMALL && errorAngle < ANGLE_MEDIUM){
        steeringPwmCmd = PWM_NEUTRAL + 25; // Medium right turn
    }
    else if (errorAngle >= ANGLE_MEDIUM && errorAngle < ANGLE_LARGE){
        steeringPwmCmd = PWM_NEUTRAL + 35; // Large right turn
    }
    else if (errorAngle >= ANGLE_LARGE){
        steeringPwmCmd = PWM_RIGHT_MAX; // Maximum right turn
    }
    else if (errorAngle <= -ANGLE_TOLERANCE && errorAngle > -ANGLE_SMALL){
        steeringPwmCmd = PWM_NEUTRAL - 10; // Small left turn
    }
    else if (errorAngle <= -ANGLE_SMALL && errorAngle > -ANGLE_MEDIUM){
        steeringPwmCmd = PWM_NEUTRAL - 25; // Medium left turn
    }
    else if (errorAngle <= -ANGLE_MEDIUM && errorAngle > -ANGLE_LARGE){
        steeringPwmCmd = PWM_NEUTRAL - 35; // Large left turn
    }
    else if (errorAngle <= -ANGLE_LARGE){
        steeringPwmCmd = PWM_LEFT_MAX; // Maximum left turn
    }

    return errorAngle;
}





*/