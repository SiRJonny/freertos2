/*
 * StatePattern.cpp
 *
 *  Created on: 2016. jan. 6.
 *      Author: Gabor
 */

#include "StatePattern.hpp"
#include "config.hpp"


extern void SetServo_sensor(int pos);

KanyarState BaseState::kanyar("kany", &BaseState::gyorsito, SLOW, 1000, GYORSITO);
GyorsitoState BaseState::gyorsito("gy1", &BaseState::gyors, SLOW, 1000, GYORSITO);
GyorsState BaseState::gyors("gy2", &BaseState::lassito, FAST, 2000, GYORSITO);
LassitoState BaseState::lassito("lass", &BaseState::kanyar, SLOW, 3000, GYORSITO);

StopState BaseState::stopped;
StartState BaseState::started("start", &BaseState::gyorsito, SLOW, 0, GYORSITO);



SafetyState BaseState::safetyLassit(10, &BaseState::safetyKanyar, SAFETYSLOW, 2000, false, SIMA);
SafetyState BaseState::safetyKanyar(11,&BaseState::safetyFast, SAFETYSLOW, 1000, true, GYORSITO);
SafetyState BaseState::safetyFast(12, &BaseState::safetyLassit, SAFETYFAST, 2000, false, GYORSITO);

GyorsState BaseState::gyors1("gyors1", &BaseState::lassito, FAST, 2000, GYORSITO);
LassitoState BaseState::lassito1("las1", &BaseState::kanyar, SLOW, 3000, GYORSITO);
KanyarState BaseState::kanyar1("kany1", &BaseState::gyorsito, SLOW, 1000, GYORSITO);
GyorsitoState BaseState::gaz1("gaz1", &BaseState::gyors, SLOW, 1000, GYORSITO);

GyorsState BaseState::gyors2("gyors2", &BaseState::lassito, FAST, 2000, GYORSITO);
LassitoState BaseState::lassito2("las2", &BaseState::kanyar, SLOW, 3000, GYORSITO);
KanyarState BaseState::kanyar2("kany2", &BaseState::gyorsito, SLOW, 1000, GYORSITO);
GyorsitoState BaseState::gaz2("gaz2", &BaseState::gyors, SLOW, 1000, GYORSITO);

TavState BaseState::gy3tav("gy3tav", &BaseState::gyors3, FAST, 2000);
GyorsState BaseState::gyors3("gyors3", &BaseState::lassito, FAST, 2000, GYORSITO);
LassitoState BaseState::lassito3("las3", &BaseState::kanyar, SLOW, 3000, GYORSITO);
KanyarState BaseState::kanyar3("kany3", &BaseState::gyorsito, SLOW, 1000, GYORSITO);
GyorsitoState BaseState::gaz3("gaz3", &BaseState::gyors, SLOW, 1000, GYORSITO);

GyorsState BaseState::gyors4("gyors4", &BaseState::lassito, FAST, 2000, GYORSITO);
LassitoState BaseState::lassito4("las4", &BaseState::kanyar, SLOW, 3000, GYORSITO);
KanyarState BaseState::kanyar4("kany4", &BaseState::gyorsito, SLOW, 1000, GYORSITO);
GyorsitoState BaseState::gaz4("gaz4", &BaseState::gyors, SLOW, 1000, GYORSITO);




int sensorAngle  = 200;

int stateCounter = 0;
int statemax = 20;

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
KanyarState::KanyarState(string stateName,
		BaseState* nState,
		int minWaitDistance,
		float tSpeed,
		SpeedEvent tEvent) {

	stateId = 1;
	steeringPD = false;
	targetSpeed = tSpeed;

	isSafety = false;
	distanceToMove = minWaitDistance; //encoderPosDifference = 3000;
	nextState = nState;
	targetEvent = tEvent;
	name = stateName;
}

void KanyarState::handleEvent(StateContext& context, SpeedEvent event) {
	moveSensor();

	if (event == targetEvent) {
		resetSensor();
		context.setState(&BaseState::gyorsito);
	}
}


//Gyorsításkor
GyorsitoState::GyorsitoState(string stateName,
		BaseState* nState,
		int minWaitDistance,
		float tSpeed,
		SpeedEvent tEvent) {
	name = stateName;
	nextState = nState;
	stateId = 2;
	steeringPD = false;
	targetSpeed = tSpeed;
	distanceToMove = minWaitDistance; //encoderPosDifference = 1200;
	isSafety = false;
	targetEvent = tEvent;
}

void GyorsitoState::handleEvent(StateContext& context, SpeedEvent event) {
	if (event == SIMA || event == GYORSITO) {
		context.setState(nextState);
	}
}

//Gyors szakaszon
GyorsState::GyorsState(string stateName,
		BaseState* nState,
		int minWaitDistance,
		float tSpeed,
		SpeedEvent tEvent) {
	stateId = 3;
	steeringPD = true;
	targetSpeed = tSpeed;
	distanceToMove = minWaitDistance; //encoderPosDifference = 2500;
	isSafety = false;

	nextState = nState;
	targetEvent = tEvent;
	name = stateName;
}

void GyorsState::handleEvent(StateContext& context, SpeedEvent event) {
	if (event == targetEvent) {
		PIDm.iState = 0;
		context.setState(nextState);
	}
}


//Gyors hosszu szakaszon
TavState::TavState(string stateName,
		BaseState* nState,
		int minWaitDistance,
		float tSpeed) {
	stateId = 6;
	steeringPD = true;
	targetSpeed = tSpeed;
	distanceToMove = minWaitDistance; //encoderPosDifference = 2500;
	isSafety = false;

	nextState = nState;
	name = stateName;
}

void TavState::handleEvent(StateContext& context, SpeedEvent event) {
	if (event != SEMMI) {
		context.setState(nextState);
	}
}

//Lassításkor
LassitoState::LassitoState(string stateName,
		BaseState* nState,
		int minWaitDistance,
		float tSpeed,
		SpeedEvent tEvent) {
	stateId = 4;
	steeringPD = true;
	targetSpeed = tSpeed;
	distanceToMove = minWaitDistance; //6000;
	isSafety = false;

	nextState = nState;
	targetEvent = tEvent;
	name = stateName;
}

void LassitoState::handleEvent(StateContext& context, SpeedEvent event) {
	if (event == targetEvent) {
		context.setState(nextState);
		PIDm.iState = 0;
	}
}

//Stop state
StopState::StopState() {
	stateId = -1;
	steeringPD = false;
	targetSpeed = 0;
	distanceToMove = 0;
	isSafety = false;
}

void StopState::handleEvent(StateContext& context, SpeedEvent event) {
	if (event == START) {
		resetSensor();
		context.setState(&BaseState::started);
	}
}

//Start state
StartState::StartState(string stateName,
		BaseState* nState,
		int minWaitDistance,
		float tSpeed,
		SpeedEvent tEvent) {
	stateId = 0;
	steeringPD = false;
	targetSpeed = tSpeed;
	distanceToMove = minWaitDistance;
	isSafety = false;

	nextState = nState;
	targetEvent = tEvent;
	name = stateName;
}

void StartState::handleEvent(StateContext& context, SpeedEvent event) {
	moveSensor();

	if (event == GYORSITO) {
		resetSensor();
		context.setState(&BaseState::gyorsito);
	}
}


//SafetyState
SafetyState::SafetyState(int st_id, BaseState* nState, float maxSpeed,int howMuchToMove, bool moveSensor, SpeedEvent triggerSpeedEvent) {
	stateId = st_id;
	steeringPD = false;
	targetSpeed = maxSpeed;
	distanceToMove = howMuchToMove;
	isSafety = true;
	isSensorMoved = moveSensor;
	triggerEvent = triggerSpeedEvent;
	nextState = nState;
	isSafety = true;
}

void SafetyState::handleEvent(StateContext& context, SpeedEvent event) {
	if (isSensorMoved) {
		moveSensor();
	}

	if (event == triggerEvent) {
		resetSensor();
		context.setState(nextState);
	} else if (event == SPEEDUP) {
		safety_car = false;
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
	stateCounter++;
	state = newState;
	state->triggerGlobalDistance = globalDistance + state->distanceToMove;
}


void StateContext::update(bool stable3lines, int encoderPos){
	currEncoderPos = encoderPos;

	if (stateCounter >= statemax) {
		setState(&BaseState::stopped);
	}

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


void StateContext::stop(){
	state->stop(*this);
	//SetServo_motor(0);
}

void StateContext::start(int encoderPos) {
	currEncoderPos = encoderPos;
	setState(&BaseState::safetyFast);
}



