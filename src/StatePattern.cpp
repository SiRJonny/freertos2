/*
 * StatePattern.cpp
 *
 *  Created on: 2016. jan. 6.
 *      Author: Gabor
 */

#include "StatePattern.hpp"
#include "config.hpp"


extern void SetServo_motor(int pos);


KanyarState BaseState::kanyar;
GyorsitoState BaseState::gyorsito;
GyorsState BaseState::gyors;
LassitoState BaseState::lassito;
StopState BaseState::stopped;
StartState BaseState::started;

//Kanyar közben
KanyarState::KanyarState() {
	stateId = 1;
	steeringPD = false;
	targetSpeed = SLOW;
	encoderPosDifference = 3000;
}

void KanyarState::handleEvent(StateContext& context, Event event) {
	if (event == GYORSITO) {
		context.setState(&BaseState::gyorsito);
	}
}


//Gyorsításkor
GyorsitoState::GyorsitoState() {
	stateId = 2;
	steeringPD = false;
	targetSpeed = SLOW;
	encoderPosDifference = 1200;
}

void GyorsitoState::handleEvent(StateContext& context, Event event) {
	context.setState(&BaseState::gyors);
}

//Gyors szakaszon
GyorsState::GyorsState() {
	stateId = 3;
	steeringPD = true;
	targetSpeed = FAST;
	encoderPosDifference = 2500;
}

void GyorsState::handleEvent(StateContext& context, Event event) {
	if (event == GYORSITO) {
		context.setState(&BaseState::lassito);
	}
}

//Lassításkor
LassitoState::LassitoState() {
	stateId = 4;
	steeringPD = true;
	targetSpeed = SLOW;
	encoderPosDifference = 6000;
}

void LassitoState::handleEvent(StateContext& context, Event event) {
	if (event == SIMA) {
		context.setState(&BaseState::kanyar);
		PIDm.iState = 0;
	}
}

//Stop state
StopState::StopState() {
	stateId = -1;
	steeringPD = false;
	targetSpeed = 0;
}

void StopState::handleEvent(StateContext& context, Event event) {
	if (event == START) {
		context.setState(&BaseState::started);
	}
}

//Start state
StartState::StartState() {
	stateId = 0;
	steeringPD = false;
	targetSpeed = SLOW;
	encoderPosDifference = 0;
}

void StartState::handleEvent(StateContext& context, Event event) {
	if (event == GYORSITO) {
		context.setState(&BaseState::gyorsito);
	}
}

//BaseState
void BaseState::stop(StateContext& context) {
	context.setState(&BaseState::stopped);
}

//BaseState::~BaseState() {};

//StateContext
StateContext::StateContext(){
	setState(&BaseState::stopped);
}

void StateContext::handleEvent(Event event) {
	state->handleEvent(*this, event);
}

void StateContext::setState(BaseState* newState){
	state = newState;
	state->targetEncoderPos = currEncoderPos - state->encoderPosDifference;
}


void StateContext::update(bool stable3lines, int encoderPos){
	currEncoderPos = encoderPos;

	if(currEncoderPos <= state->targetEncoderPos)
		{

			if (stable3lines) {
				handleEvent(GYORSITO);
			} else {
				handleEvent(SIMA);
			}
		}
}

float StateContext::getTargetSpeed(){
	return state->targetSpeed;
}

bool StateContext::isSteeringPD(){
	return state->steeringPD;
}

void StateContext::stop(){
	state->stop(*this);
	//SetServo_motor(0);
}

void StateContext::start(int encoderPos) {
	currEncoderPos = encoderPos;
	setState(&BaseState::started);
}

int StateContext::getStateId() {
	return state->stateId;
}


