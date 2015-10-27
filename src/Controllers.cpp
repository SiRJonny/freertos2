/*
 * Controllers.c
 *
 *  Created on: 2015. okt. 27.
 *      Author: Csabi
 */

#include "Controllers.h"




float UpdatePID1(PID_struct * pid, float error, float position)
{
	double pTerm, dTerm, iTerm;

	pTerm = pid->pGain * error;   // calculate the proportional term
	// calculate the integral state with appropriate limiting
	pid->iState += error;
	if (pid->iState > pid->iMax) pid->iState = pid->iMax;
	else if (pid->iState < pid->iMin) pid->iState = pid->iMin;
	iTerm = pid->iGain * pid->iState;  // calculate the integral term
	dTerm = pid->dGain * (pid->dState - position);
	pid->dState = position;
	return pTerm + dTerm + iTerm;
}
