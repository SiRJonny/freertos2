/*
 * StateMachine.cpp
 *
 *  Created on: 2015. dec. 9.
 *      Author: Csabi
 */

#include "StateMachine.h"

extern void SetServo_motor(int pos);
extern float B;

void StateMachine(state_machine_struct * State, LineState * Lines, int encoderPos)
{
	switch (State->state)
	{
	case -1:
		//SET_SPEED = 0;
		SetServo_motor(0);
		break;
	case 0:		// start state
		//B = 0.85;
		SET_SPEED = SLOW;
		if(Lines->numLines1 == 3 && Lines->numLines2 == 3)
		{
			State->TargetEncoderPos = encoderPos - 1250;	// kb 0,5 méter
			State->nextState = 1;
			State->state = 3;
			//B = 0.85;
			SET_SPEED = SLOW;
		}
		break;
	case 1:		// kanyar végetért state
		State->TargetEncoderPos = encoderPos - 5000;	// kb 2 méter
		State->nextState = 2;
		State->state = 3;
		//B = 1.25;
		SET_SPEED = FAST;
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		break;
	case 2:		// gyors state
		//SET_SPEED = FAST;
		if(Lines->numLines1 == 3 && Lines->numLines2 == 3)
		{
			State->TargetEncoderPos = encoderPos - 10000;	// kb 4 méter
			State->nextState = 4;
			State->state = 3;
			//B = 0.85;
			SET_SPEED = SLOW;
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		}
		break;
	case 3:		// várakozó state (lefele számol az encoder..)
		if(encoderPos < State->TargetEncoderPos)
		{
			State->state = State->nextState;
		}
		break;
	case 4:		// kanyar state
		//SET_SPEED = SLOW;
		if(Lines->numLines1 == 3 && Lines->numLines2 == 3)
		{
			State->TargetEncoderPos = encoderPos - 1250;	// kb 0,5 méter
			State->nextState = 1;
			State->state = 3;
			//B = 0.85;
			SET_SPEED = SLOW;
		}
		break;
	}
}
