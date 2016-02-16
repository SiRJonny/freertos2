/*
 * StatePattern.cpp
 *
 *  Created on: 2016. jan. 6.
 *      Author: Gabor
 */

#include "StatePattern.hpp"
#include "config.hpp"


extern void SetServo_sensor(int pos);

KanyarState BaseState::kanyar;
GyorsitoState BaseState::gyorsito;
GyorsState BaseState::gyors;
LassitoState BaseState::lassito;
StopState BaseState::stopped;
StartState BaseState::started;

SafetyState BaseState::safetySlow(&BaseState::safetyFast, SAFETYSLOW, 4000, true, GYORSITO);
SafetyState BaseState::safetyFast(&BaseState::safetySlow, SAFETYFAST, 2000, false, GYORSITO);

int sensorAngle  = 180;

void moveSensor() {
	if(safety_car) {
			if (control > 0) {
				SetServo_sensor(-sensorAngle);
			} else {
				SetServo_sensor(sensorAngle);
			}
		}
}

void resetSensor() {
	SetServo_sensor(0);
}

//Kanyar közben
KanyarState::KanyarState() {
	stateId = 1;
	steeringPD = false;
	targetSpeed = SLOW;
	distanceToMove = 1000; //encoderPosDifference = 3000;
}

void KanyarState::handleEvent(StateContext& context, SpeedEvent event) {
	moveSensor();

	if (event == GYORSITO) {
		resetSensor();
		context.setState(&BaseState::gyorsito);
	}
}


//Gyorsításkor
GyorsitoState::GyorsitoState() {
	stateId = 2;
	steeringPD = false;
	targetSpeed = SLOW;
	distanceToMove = 500; //encoderPosDifference = 1200;
}

void GyorsitoState::handleEvent(StateContext& context, SpeedEvent event) {
	if (event == SIMA || event == GYORSITO) {
		context.setState(&BaseState::gyors);
	}
}

//Gyors szakaszon
GyorsState::GyorsState() {
	stateId = 3;
	steeringPD = true;
	targetSpeed = FAST;
	distanceToMove = 2000; //encoderPosDifference = 2500;
}

void GyorsState::handleEvent(StateContext& context, SpeedEvent event) {
	if (event == GYORSITO) {
		context.setState(&BaseState::lassito);
	}
}

//Lassításkor
LassitoState::LassitoState() {
	stateId = 4;
	steeringPD = true;
	targetSpeed = SLOW;
	distanceToMove = 3000; //6000;
}

void LassitoState::handleEvent(StateContext& context, SpeedEvent event) {
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
	distanceToMove = 0;
}

void StopState::handleEvent(StateContext& context, SpeedEvent event) {
	if (event == START) {
		resetSensor();
		context.setState(&BaseState::started);
	}
}

//Start state
StartState::StartState() {
	stateId = 0;
	steeringPD = false;
	targetSpeed = SLOW;
	distanceToMove = 0;
}

void StartState::handleEvent(StateContext& context, SpeedEvent event) {
	moveSensor();

	if (event == GYORSITO) {
		resetSensor();
		context.setState(&BaseState::gyorsito);
	}
}


//SafetyState
SafetyState::SafetyState(BaseState* nState, float maxSpeed,int howMuchToMove, bool moveSensor, SpeedEvent triggerSpeedEvent) {
	stateId = 10;
	steeringPD = false;
	targetSpeed = maxSpeed;
	distanceToMove = howMuchToMove;
	isSafety = true;
	isSensorMoved = moveSensor;
	triggerEvent = triggerSpeedEvent;
	nextState = nState;
}

void SafetyState::handleEvent(StateContext& context, SpeedEvent event) {
	if (isSensorMoved) {
		moveSensor();
	}

	if (event == triggerEvent) {
		resetSensor();
		context.setState(&BaseState::gyorsito);
	} else if (event == SPEEDUP) {
		//todo context.setState(&BaseState::gyorsito);
	}
}




//BaseState
void BaseState::stop(StateContext& context) {
	context.setState(&BaseState::stopped);
}

//StateContext
StateContext::StateContext(){
	setState(&BaseState::stopped);
}

void StateContext::handleEvent(SpeedEvent event) {
	state->handleEvent(*this, event);
}

void StateContext::setState(BaseState* newState){
	state = newState;
	state->triggerGlobalDistance = globalDistance + state->distanceToMove;
}


void StateContext::update(bool stable3lines, int encoderPos){
	currEncoderPos = encoderPos;

	if (globalDistance >= state->triggerGlobalDistance) {
			if (stable3lines) {
				handleEvent(GYORSITO);
			} else {
				handleEvent(SIMA);
			}
		}
	else {
		handleEvent(SEMMI);
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


