/*
 * StatePattern.cpp
 *
 *  Created on: 2016. jan. 6.
 *      Author: Gabor
 */

#include "StatePattern.hpp"
#include "config.hpp"


extern void SetServo_sensor(int pos);
/*
KanyarState BaseState::kanyar(1, "kany", &BaseState::gyorsito, SLOW, 1000, GYORSITO);
GyorsitoState BaseState::gyorsito("gy1", &BaseState::gyors, SLOW, 1000, GYORSITO);
GyorsState BaseState::gyors("gy2", &BaseState::lassito, FAST, 2000, GYORSITO);
LassitoState BaseState::lassito("lass", &BaseState::kanyar, SLOW, 3000, GYORSITO);
*/
StopState BaseState::stopped;
KanyarState BaseState::started(0, "start1", &BaseState::lap, &SLOW, 0, GYORSITO);



SafetyState BaseState::safetyLassit(10, &BaseState::safetyKanyar, &SAFETYSLOW, 2000, false, SIMA);
SafetyState BaseState::safetyKanyar(11,&BaseState::safetyFast, &SAFETYSLOW, 1000, true, GYORSITO);
SafetyState BaseState::safetyFast(12, &BaseState::safetyLassit, &SAFETYFAST, 2000, false, GYORSITO);


SafetyState BaseState::safetyFast1(21, &BaseState::safetyLassit1, &SAFETYFAST, 2000, false, GYORSITO);
SafetyState BaseState::safetyLassit1(22, &BaseState::safetyKanyar1, &SAFETYSLOW, 2000, false, SIMA);
SafetyState BaseState::safetyKanyar1(23,&BaseState::safetyFast2, &SAFETYSLOW, 3000, true, GYORSITO);
SafetyState BaseState::safetyFast2(24, &BaseState::safetyLassit2, &SAFETYFAST, 4000, false, GYORSITO);
SafetyState BaseState::safetyLassit2(25, &BaseState::safetyKanyar2, &SAFETYSLOW, 2000, false, SIMA);
SafetyState BaseState::safetyKanyar2(26,&BaseState::gaz4, &SAFETYSLOW, 1000, true, GYORSITO);

int tavFekDist = 1700;

GyorsState BaseState::gyors1("gyors1", &BaseState::tav1, &FAST, 2000, GYORSITO);
TavState BaseState::tav1("tav1", &BaseState::lassito1, &MEDIUM, tavFekDist, false );

LassitoState BaseState::lassito1("las1", &BaseState::kanyar1, &SLOW, 0, SIMA);
KanyarTavState BaseState::kanyarTav1("kanyt1", &BaseState::kanyar1, &SLOW, 1010);
KanyarState BaseState::kanyar1(110, "kany1", &BaseState::gaz1, &SLOW, 1010, GYORSITO);
GyorsitoState BaseState::gaz1("gaz1", &BaseState::gyors2, &SLOW, 1000, GYORSITO);

GyorsState BaseState::gyors2("gyors2", &BaseState::tav2, &FAST, 2000, GYORSITO);
TavState BaseState::tav2("tav2", &BaseState::lassito2, &MEDIUM, tavFekDist, false );

LassitoState BaseState::lassito2("las2", &BaseState::kanyar2, &SLOW, 0, SIMA);
KanyarTavState BaseState::kanyarTav2("kanyt2", &BaseState::kanyar2, &SLOW, 1010);
KanyarState BaseState::kanyar2(111, "kany2", &BaseState::gaz2, &KANYAR2, 4000, GYORSITO);
GyorsitoState BaseState::gaz2("gaz2", &BaseState::gyors3, &SLOW, 1000, GYORSITO);

TavState BaseState::gy3tav("gy3tav", &BaseState::gyors3, &FAST, 2000, false);
GyorsState BaseState::gyors3("gyors3", &BaseState::tav3, &FAST, 2000, GYORSITO);

TavState BaseState::tav3("tav3", &BaseState::lassito3, &MEDIUM, tavFekDist, false );
LassitoState BaseState::lassito3("las3", &BaseState::kanyar3, &SLOW, 0, SIMA);
KanyarTavState BaseState::kanyarTav3("kanyt3", &BaseState::kanyar3, &SLOW, 1010);
KanyarState BaseState::kanyar3(112, "kany3", &BaseState::gaz3, &KANYAR3, 4000, GYORSITO);
GyorsitoState BaseState::gaz3("gaz3", &BaseState::gyors4, &SLOW, 1000, GYORSITO);

GyorsState BaseState::gyors4("gyors4", &BaseState::tav4, &FAST, 2000, GYORSITO);

TavState BaseState::tav4("tav4", &BaseState::lassito4, &MEDIUM, tavFekDist, false );
LassitoState BaseState::lassito4("las4", &BaseState::kanyar4, &SLOW, 0, SIMA);
KanyarTavState BaseState::kanyarTav4("kanyt4", &BaseState::kanyar4, &SLOW, 1010);
KanyarState BaseState::kanyar4(13, "kany4", &BaseState::lap, &SLOW, 1010, GYORSITO);
GyorsitoState BaseState::gaz4("gaz4", &BaseState::gyors1, &SLOW, 1000, GYORSITO);

TavState BaseState::lap("lap", &BaseState::gaz4, &SLOW, 0, true);




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
KanyarState::KanyarState(int id,
		string stateName,
		BaseState* nState,
		float *tSpeed,
		int minWaitDistance,
		SpeedEvent tEvent) {

	stateId = id;
	steeringPD = false;
	targetSpeed = tSpeed;

	isSafety = false;
	distanceToMove = minWaitDistance; //encoderPosDifference = 3000;
	nextState = nState;
	targetEvent = tEvent;
	name = stateName;
	isSensorMoved = true;
}

void KanyarState::handleEvent(StateContext& context, SpeedEvent event) {
	if (event == targetEvent) {
		resetSensor();
		context.setState(nextState);
	} else if (triggerGlobalDistance + distanceToMove){
		//context.setState(nextState);
	}
}


//Gyorsításkor
GyorsitoState::GyorsitoState(string stateName,
		BaseState* nState,
		float *tSpeed,
		int minWaitDistance,
		SpeedEvent tEvent) {
	name = stateName;
	nextState = nState;
	stateId = 2;
	steeringPD = true;
	targetSpeed = tSpeed;
	distanceToMove = minWaitDistance; //encoderPosDifference = 1200;
	isSafety = false;
	targetEvent = tEvent;
	isSensorMoved = false;
}

void GyorsitoState::handleEvent(StateContext& context, SpeedEvent event) {
	if (event == SIMA || event == GYORSITO) {
		context.setState(nextState);
	}
}

//Gyors szakaszon
GyorsState::GyorsState(string stateName,
		BaseState* nState,
		float* tSpeed,
		int minWaitDistance,
		SpeedEvent tEvent) {
	stateId = 3;
	steeringPD = true;
	targetSpeed = tSpeed;
	distanceToMove = minWaitDistance; //encoderPosDifference = 2500;
	isSafety = false;

	nextState = nState;
	targetEvent = tEvent;
	name = stateName;
	isSensorMoved = false;
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
		float *tSpeed,
		int minWaitDistance,
		bool lap) {
	stateId = 6;
	steeringPD = true;
	targetSpeed = tSpeed;
	distanceToMove = minWaitDistance; //encoderPosDifference = 2500;
	isSafety = false;

	nextState = nState;
	name = stateName;
	lapState = lap;
	isSensorMoved = false;
}

void TavState::handleEvent(StateContext& context, SpeedEvent event) {

	if (event != SEMMI) {
		if (lapState) {
			lapCounter++;
		}
		context.setState(nextState);
	}
}

//Tav alapu kanyar
KanyarTavState::KanyarTavState(string stateName,
		BaseState* nState,
		float *tSpeed,
		int minWaitDistance) {
	stateId = 66;
	steeringPD = true;
	targetSpeed = tSpeed;
	distanceToMove = minWaitDistance; //encoderPosDifference = 2500;
	isSafety = false;

	nextState = nState;
	name = stateName;
	isSensorMoved = false;
}

void KanyarTavState::handleEvent(StateContext& context, SpeedEvent event) {
	if (event != SEMMI) {
		context.setState(nextState);
	}
}

//Lassításkor
LassitoState::LassitoState(string stateName,
		BaseState* nState,
		float *tSpeed,
		int minWaitDistance,
		SpeedEvent tEvent) {
	stateId = 4;
	steeringPD = true;
	targetSpeed = tSpeed;
	distanceToMove = minWaitDistance; //6000;
	isSafety = false;

	nextState = nState;
	targetEvent = tEvent;
	name = stateName;
	isSensorMoved = false;
}

void LassitoState::handleEvent(StateContext& context, SpeedEvent event) {
	if (lapCounter >= lapMax+1 ) {
		context.setState(&BaseState::stopped);
	}

	if (event == targetEvent) {
		context.setState(nextState);

	}
}

//Stop state
StopState::StopState() {
	stateId = -1;
	steeringPD = true;
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



//SafetyState
SafetyState::SafetyState(int st_id, BaseState* nState, float *maxSpeed,int howMuchToMove, bool moveSensor, SpeedEvent triggerSpeedEvent) {
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

extern float szorzo;
void StateContext::update(bool stable3lines, int encoderPos){
	currEncoderPos = encoderPos;

	if (lapCounter == 1) {
		//szorzo = 0.8;
	} else if (lapCounter == 2) {
		//szorzo = 0.9;
	} else if (lapCounter == 3) {
		//szorzo = 1;
	}

	if (state->isSensorMoved) {
		moveSensor();
	} else {
		resetSensor();
	}

	if (globalDistance >= state->triggerGlobalDistance) {
		if (stable3lines) {
			handleEvent(GYORSITO);
		} else {
			handleEvent(SIMA);
		}
	}
}


void StateContext::stop(){
	state->stop(*this);
	//SetServo_motor(0);
}

void StateContext::start(int encoderPos) {
	currEncoderPos = encoderPos;
	setState(&BaseState::safetyFast1);
}



