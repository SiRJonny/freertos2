/*
 * Controllers.h
 *
 *  Created on: 2015. okt. 27.
 *      Author: Csabi
 */

#ifndef INCLUDE_CONTROLLERS_H_
#define INCLUDE_CONTROLLERS_H_

#include "stm32f4xx_hal.h"


typedef struct
{
	float dState;       // Last position input
	float iState;       // Integrator state
	float iMax, iMin;   // Maximum and minimum allowable integrator state
	float iGain,     // integral gain
	pGain,     // proportional gain
	dGain;      // derivative gain
} PID_struct;

float UpdatePID1(PID_struct * pid, float error, float position);










#endif /* INCLUDE_CONTROLLERS_H_ */
