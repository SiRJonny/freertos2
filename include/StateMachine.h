/*
 * StateMachine.h
 *
 *  Created on: 2015. dec. 9.
 *      Author: Csabi
 */

#ifndef INCLUDE_STATEMACHINE_H_
#define INCLUDE_STATEMACHINE_H_

#include "ProcessSensors.h"

extern float SLOW;
extern float FAST;
extern float SET_SPEED;

typedef struct
{
	int state;       // state
	int TargetEncoderPos;
	int nextState;

} state_machine_struct;

void StateMachine(state_machine_struct * State, bool * stable3lines, int encoderPos);

#endif /* INCLUDE_STATEMACHINE_H_ */
