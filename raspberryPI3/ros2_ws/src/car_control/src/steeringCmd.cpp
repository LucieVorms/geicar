#include "../include/car_control/steeringCmd.h"
#include "../include/car_control/steeringCmd.h"

/* Angles are specified in radians */
#define STEERING_ANGLE_MAX     (+1.0f) // TBV
#define STEERING_ANGLE_MIN     (-1.0f)
#define STEERING_ANGLE_NUL     ( 0.0f) 

#define STEERING_DELTA_ANGLE_MAX ((STEERING_ANGLE_MAX) - (STEERING_ANGLE_NUL))
#define STEERING_DELTA_ANGLE_MIN ((STEERING_ANGLE_MIN) - (STEERING_ANGLE_NUL))

#define PWM_NUL_STEERING_ANGLE (50)
#define MAX_DELTA_PWM          (50)

//return the Pwm command to reach the angle passed in argument
int steeringCmd(float requestedSteerAngle, float currentSteerAngle, uint8_t & steeringPwmCmd){
        float errorAngle = requestedSteerAngle - currentSteerAngle;

        if (abs(errorAngle)<TOLERANCE_ANGLE)  // Or set flush to zero ?  
                steeringPwmCmd = PWM_NUL_STEERING_ANGLE;
        else   
                steeringPwmCmd = PWM_NUL_STEERING_ANGLE - (int)((errorAngle * MAX_DELTA_PWM)/STEERING_DELTA_ANGLE_MAX);

        return errorAngle;
}


#if 0
//return the Pwm command to reach the angle passed in argument
int steeringCmd(float requestedSteerAngle, float currentSteerAngle, uint8_t & steeringPwmCmd){

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
#endif

