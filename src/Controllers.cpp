/*
 * Controllers.c
 *
 *  Created on: 2015. okt. 27.
 *      Author: Csabi
 */

#include "Controllers.h"



float d5, t5, T, w0, Re_s, kp, kd;
float kszi = 0.9;
float L = 0.285;//0.335; // szenzorsor távolsága hátsó tengelytõl

float UpdatePID1(PID_struct * pid, float error, float position)
{
	float pTerm, dTerm, iTerm;

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

// L = 0,335
//
float UpdateStateSpace(float A, float B, float v, float p, float angle)
{
	d5 = A*v + B;
	t5 = d5/v;
	T = (kszi*t5)/3;
	w0 = 1/T;
	Re_s = kszi*w0;

	kp = (L*w0*w0)/(v*v);
	kd = (L/v)*( (Re_s+Re_s) - (v*kp) );


	return kp*p + kd*angle;
}
