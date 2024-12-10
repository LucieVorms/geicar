#include "../include/car_control/steeringCmd.h"

#define MAX_ANGLE_ERROR 1.0 // Maximum error angle in normalized range [-1, 1]
#define TOLERANCE_ANGLE 0.08 // Tolerance angle in normalized range [-1, 1]
#define STOP 50
#define DEADBAND 0.13 // Deadband around zero to handle sensor imprecision

// Return the Pwm command to reach the angle passed in argument
int steeringCmd(float requestedSteerAngle, float currentSteerAngle, uint8_t & steeringPwmCmd) {
    // Convert requestedSteerAngle from degrees to the normalized range [-1, 1]
    float normalizedRequestedAngle = requestedSteerAngle / 30.0;
    
    // Apply deadband to currentSteerAngle to handle small variations around zero
    if (abs(currentSteerAngle) < DEADBAND) {
        currentSteerAngle = 0.0;
    }

    // Compute the error angle
    float errorAngle = currentSteerAngle - normalizedRequestedAngle;

    // Command's calculation
    if (abs(errorAngle) < TOLERANCE_ANGLE) {
        steeringPwmCmd = STOP;
    } else {
        // Compute a proportional PWM value based on the error angle
        float proportionalPwm = (abs(errorAngle) / MAX_ANGLE_ERROR) * 50; // 50 is the middle value for left and right
        if (errorAngle > 0) {
            steeringPwmCmd = 50 - proportionalPwm; // Turn left
        } else {
            steeringPwmCmd = 50 + proportionalPwm; // Turn right
        }
    }

    return errorAngle;
}