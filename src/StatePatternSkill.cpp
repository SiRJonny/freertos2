/*
 * StatePatternSkill.cpp
 *
 *  Created on: 2016. febr. 2.
 *      Author: Gabor
 */

#include "StatePatternSkill.hpp"

KoztesState SkillBaseState::koztes;
SkillStopState SkillBaseState::skillStopped;
SkillStartState SkillBaseState::skillStarted;

//Koztes
KoztesState::KoztesState() {
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
	stateId = -1;
	targetSpeed = 0;
}

void SkillStopState::update(SkillStateContext& context, StateData data){

}

//Start state
SkillStartState::SkillStartState() {
	stateId = 0;
	targetSpeed = SKILLSLOW;
}

void SkillStartState::update(SkillStateContext& context, StateData data){
	switch (data.event) {
			case RADIOSTART:
				context.setState(&SkillBaseState::koztes);
				break;
		}
}

//SkillStateContext


void SkillStateContext::setState(SkillBaseState* newState) {
	state = newState;
	state->triggerGlobalDistance = globalDistance + state->distanceToMove;
}

SkillStateContext::SkillStateContext() {
	setState(&SkillBaseState::skillStarted);
}

void SkillBaseState::stop(SkillStateContext& context) {
	context.state = &SkillBaseState::skillStopped;
}


/*
KoztesState::~KoztesState() {}
EventBasedState::~EventBasedState() {}
MovingState::~MovingState() {}
SkillStartState::~SkillStartState() {}
SkillStopState::~SkillStopState() {}
*/
