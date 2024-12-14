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
        steeringPwmCmd = 50 + (errorAngle * 50); // 50 is the neutral position

        if (steeringPwmCmd > 100.0) steeringPwmCmd = 100.0;
        if (steeringPwmCmd < 0) steeringPwmCmd = 0;

    }

    return errorAngle;
}

//return the Pwm command to reach the angle passed in argument
int steeringCmdZero(float requestedSteerAngle, float currentSteerAngle, uint8_t & steeringPwmCmd){

	float errorAngle = currentSteerAngle - requestedSteerAngle;

    //Command's calculation
	if (abs(errorAngle)<TOLERANCE_ANGLE){
		steeringPwmCmd = STOP;
	}
	else {
		if (errorAngle>0) {
			steeringPwmCmd = MAX_PWM_LEFT;
		}
		else {
			steeringPwmCmd = MAX_PWM_RIGHT;
		}
	}

    return errorAngle;
}

