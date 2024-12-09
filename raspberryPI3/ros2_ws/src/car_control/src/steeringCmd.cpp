#include "../include/car_control/steeringCmd.h"
/* Angles are specified in radians */
#define STEERING_ANGLE_MAX     (+1.0f) // TBV
#define STEERING_ANGLE_MIN     (-1.0f)
#define STEERING_ANGLE_NUL     ( 0.0f) 

#define STEERING_ANGLE_MAX_DEGREES (30.0f)

#define STEERING_DELTA_ANGLE_MAX ((STEERING_ANGLE_MAX) - (STEERING_ANGLE_NUL))
#define STEERING_DELTA_ANGLE_MIN ((STEERING_ANGLE_MIN) - (STEERING_ANGLE_NUL))

#define PWM_NUL_STEERING_ANGLE (50)
#define MAX_DELTA_PWM          (50)

//return the Pwm command to reach the angle passed in argument
int steeringCmd(float requestedSteerAngle, float currentSteerAngle, uint8_t &steeringPwmCmd) {

    float requestedSteerNormalized = requestedSteerAngle / STEERING_ANGLE_MAX_DEGREES;

	
	if (requestedSteerNormalized < STEERING_ANGLE_MIN) {
		requestedSteerNormalized = STEERING_ANGLE_MIN;
	} else if (requestedSteerNormalized > STEERING_ANGLE_MAX) {
		requestedSteerNormalized = STEERING_ANGLE_MAX;
	}

    // Calcul de l'erreur
    float errorAngle = requestedSteerNormalized - currentSteerAngle;

    // Calcul de la commande PWM
    if (abs(errorAngle) < TOLERANCE_ANGLE) {
        steeringPwmCmd = PWM_NUL_STEERING_ANGLE;
    } else {
        float pwmAdjustment = (errorAngle * MAX_DELTA_PWM) / STEERING_DELTA_ANGLE_MAX;
        steeringPwmCmd = PWM_NUL_STEERING_ANGLE - static_cast<int>(pwmAdjustment);

    }

    return steeringPwmCmd;
}
