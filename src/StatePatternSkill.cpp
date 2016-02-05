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

//EventBasedState(string stateName, SkillBaseState* nState, int waitDistance,
		//SkillTrackEvent targetEvent, float tSpeed, float angle, bool controlSteer);

//MovingState(string stateName,SkillBaseState* nextState, int howMuchToMove, float angle, float tSpeed) {
MovingState SkillBaseState::parkolasTolat1("park1", &SkillBaseState::parkolasTolat2, 3000, 0, 2);
MovingState SkillBaseState::parkolasTolat2("park2", &SkillBaseState::skillStopped, 3000, 0, 2);

//Torkolat
// 45 fok = 0.78 rad
EventBasedState SkillBaseState::TorkFalakKozt("Tork1", &SkillBaseState::TorkFalakElhagyva, 100, NOWALLS, SKILLSLOW, 0, false);
MovingState SkillBaseState::TorkFalakElhagyva("Tork2", &SkillBaseState::parkolasTolat2, 100, 0.78, SKILLSLOW);
EventBasedState SkillBaseState::TorkVonalKereses("Tork3", &SkillBaseState::koztes, 0, NEWLINE, SKILLSLOW, 0, false);

SkillTrackEvent checkWalls() {
	static bool wall = false;
	SkillTrackEvent event = NONE;
	if(fal_bal && fal_jobb) {
		if (bordas_jobb) {
			event = TWOWALL_BORDAS_RIGHT;
			direction = RIGHT;
		} else if (bordas_bal) {
			event = TWOWALL_BORDAS_LEFT;
			direction = LEFT;
		} else {
			event = TWOWALL;
		}
	} else if (bordas_jobb) {
		event = WALL_BORDAS_RIGHT;
		direction = RIGHT;
	} else if(bordas_bal) {
		event = WALL_BORDAS_LEFT;
		direction = LEFT;
	} else if (wall) {
		event = NOWALLS;
	}
	if (event != NONE && event != NOWALLS) {
		wall = true;
	}
	return event;
}

SkillTrackEvent checkStateStart() {
	SkillTrackEvent event = NONE;
	//TODO stableLine?
	if (globalLines.numLines1 == -1 && last_active_line_pos1 > 13 && last_active_line_pos1 < 18) {
		event = TORKOLAT;
	}

	return event;
}

SkillTrackEvent checkNewLine() {
	SkillTrackEvent event = NONE;
	if(activeLine1 > 13 && activeLine1 < 18) {
		event = NEWLINE;
	}
	return event;
}

SkillTrackEvent SkillBaseState::calculateEvent() {
	static SkillTrackEvent lastEvent;
	static int cntr = 0;
	SkillTrackEvent event = NONE;

	switch (currentState) {
		case NONE:
			direction = UNDEFINED;
			event = checkStateStart();
			break;
		case PARKOLAS:
			break;
		case TORKOLAT:
			event = checkWalls();
			if (event == NONE) {
				event = checkNewLine();
			}
			break;
		case TELEPHELY:
			break;
		case HATAR:
			break;
		case BILLENO:
			break;
		case FORDITO:
			break;
		case CEL:
			break;
		default:
			break;
	}

	if (event == lastEvent){
		if (cntr > 5) {
			cntr = 0;
			return event;
		} else {
			cntr++;
			return NONE;
		}
	} else {
		cntr = 0;
		lastEvent = event;
		event = NONE;
	}

	return event;
}



//Koztes
KoztesState::KoztesState() {
	name = "koztes";
	stateId = 1;
	targetSpeed = SKILLSLOW;
	distanceToMove = 0;
}

void KoztesState::update() {
	switch (stateData.event) {
		case PARKOLAS:
			skillStateContext.setState(&SkillBaseState::parkolasTolat1);
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

void MovingState::update() {
	if (targetSpeed > 0) {
		if (globalDistance >= triggerGlobalDistance) {
			skillStateContext.setState(this->nextState);
		}
	} else {
		if (globalDistance <= triggerGlobalDistance) {
			skillStateContext.setState(this->nextState);
		}
	}
}

//EventBased
EventBasedState::EventBasedState(string stateName,
		SkillBaseState* nState,
		int waitDistance,
		SkillTrackEvent targetEvent,
		float tSpeed,
		float angle,
		bool controlSteer) {
	name = stateName;
	stateId = 3;
	targetSpeed = tSpeed;
	distanceToMove = waitDistance;
	steeringAngle = angle;
	steeringControlled = controlSteer;
	nextState = nState;
	triggerEvent = targetEvent;
}

void EventBasedState::update() {
	if (targetSpeed > 0) {
		if (globalDistance >= triggerGlobalDistance) {
			if(stateData.event == triggerEvent) {
				skillStateContext.setState(this->nextState);
			}
		}
	} else {
		if (globalDistance <= triggerGlobalDistance) {
			if(stateData.event == triggerEvent) {
				skillStateContext.setState(this->nextState);
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

void SkillStopState::update(){

}

//Start state
SkillStartState::SkillStartState() {
	name = "start";
	stateId = 0;
	targetSpeed = 0;
}

void SkillStartState::update(){
	switch (stateData.event) {
			case RADIOSTART:
				skillStateContext.setState(&SkillBaseState::parkolasTolat2);
				break;
		}
}

//SkillStateContext


void SkillStateContext::setState(SkillBaseState* newState) {
	state = newState;
	state->triggerGlobalDistance = globalDistance + state->distanceToMove;
}

SkillStateContext::SkillStateContext() {
	setState(&SkillBaseState::koztes);
}

void SkillBaseState::stop(SkillStateContext& context) {
	context.setState(&SkillBaseState::skillStopped);
}


