/*
 * StatePatternSkill.cpp
 *
 *  Created on: 2016. febr. 2.
 *      Author: Gabor
 */

#include <StatePatternSkill.hpp>

KoztesState SkillBaseState::koztes;
SkillStopState SkillBaseState::skillStopped;
SkillStartState SkillBaseState::skillStarted;


//MovingState(string stateName,SkillBaseState* nextState, int howMuchToMove, float angle, float tSpeed) {
MovingState SkillBaseState::parkolasTolat1("park1", &SkillBaseState::skillStopped, 5000, 0, 0.3);
MovingState SkillBaseState::parkolasTolat2("park2", &SkillBaseState::skillStopped, 5000, 0, 1);





//Koztes
KoztesState::KoztesState() {
	name = "koztes";
	stateId = 1;
	targetSpeed = SKILLSLOW;
	distanceToMove = 0;
}

void KoztesState::update(SkillStateContext& context, StateData data) {
	switch (data.event) {
		case PARKOLASSTART:
			context.setState(&SkillBaseState::parkolasTolat1);
			break;
	}
}

//Moving
MovingState::MovingState(string stateName,SkillBaseState* nState, int howMuchToMove, float angle, float tSpeed) {
	name = stateName;
	stateId = 2;
	targetSpeed = tSpeed;
	distanceToMove = howMuchToMove;
	steeringAngle = angle;
	steeringControlled = false;
	nextState = nState;
}

void MovingState::update(SkillStateContext& context, StateData data) {
	if (targetSpeed > 0) {
		if (globalDistance >= triggerGlobalDistance) {
			context.setState(this->nextState);
		}
	} else {
		if (globalDistance <= triggerGlobalDistance) {
			context.setState(this->nextState);
		}
	}
}

//EventBased
EventBasedState::EventBasedState(string stateName, SkillBaseState* nState, int waitDistance, SkillTrackEvent targetEvent, float tSpeed) {
	name = stateName;
	stateId = 3;
	targetSpeed = tSpeed;
	distanceToMove = waitDistance;
	steeringControlled = true;
	nextState = nState;
	triggerEvent = targetEvent;
}

void EventBasedState::update(SkillStateContext& context, StateData data) {
	if (targetSpeed > 0) {
		if (globalDistance >= triggerGlobalDistance) {
			if(data.event == triggerEvent) {
				context.setState(this->nextState);
			}
		}
	} else {
		if (globalDistance <= triggerGlobalDistance) {
			if(data.event == triggerEvent) {
				context.setState(this->nextState);
			}
		}
	}
}


//Stop state
SkillStopState::SkillStopState() {
	name = "stop";
	stateId = -1;
	targetSpeed = 0;
}

void SkillStopState::update(SkillStateContext& context, StateData data){

}

//Start state
SkillStartState::SkillStartState() {
	name = "start";
	stateId = 0;
	targetSpeed = 0;
}

void SkillStartState::update(SkillStateContext& context, StateData data){
	switch (data.event) {
			case RADIOSTART:
				context.setState(&SkillBaseState::parkolasTolat2);
				break;
		}
}

//SkillStateContext


void SkillStateContext::setState(SkillBaseState* newState) {
	state = newState;
	state->triggerGlobalDistance = globalDistance + state->distanceToMove;
}

SkillStateContext::SkillStateContext() {
	setState(&SkillBaseState::parkolasTolat1);
}

void SkillBaseState::stop(SkillStateContext& context) {
	context.setState(&SkillBaseState::skillStopped);
}


