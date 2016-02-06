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

/*
MovingState(string stateName,
			SkillBaseState* nState,
			int howMuchToMove,
			float tSpeed,
			float angle,
			bool controlSteer);

EventBasedState(string stateName,
			SkillBaseState* nState,
			int waitDistance,
			float tSpeed,
			float angle,
			bool controlSteer,
			SkillTrackEvent targetEvent);
			*/

MovingState SkillBaseState::parkolasTolat1("park1", &SkillBaseState::parkolasTolat2, 3000, 0, 2, false);
MovingState SkillBaseState::parkolasTolat2("park2", &SkillBaseState::skillStopped, 3000, 0, 2, false);


//Torkolat
EventBasedState SkillBaseState::TorkFalakKozt("Tork1", &SkillBaseState::TorkFalakElhagyva, 300, SKILLSLOW, 0, false, NOLINE_NOWALLS);
MovingState SkillBaseState::TorkFalakElhagyva("Tork2", &SkillBaseState::TorkVonalKereses, 500, 500, SKILLSLOW, false);
EventBasedState SkillBaseState::TorkVonalKereses("Tork3", &SkillBaseState::koztes, 0, SKILLSLOW, 0, false, NONE);




SkillTrackEvent SkillBaseState::calculateEvent() {
	static SkillTrackEvent lastEvent;
	static int cntr = 0;
	SkillTrackEvent event = NONE;
//todo kirakni
	bool line = false;
	bool keresztvonal = false;
	bool ketvonal = false;
	bool haromvonal = false;

	if (keresztvonal) {
		return KERESZT;
	}

	if(fal_bal && fal_jobb) {
		event = TWOWALL;
	} else if (!line) {
		event = NOLINE_NOWALLS;
	} else {
		//hany vonal van todo
		event = NONE;
		if (ketvonal) {
			if (fal_jobb || fal_bal) {
				event = STABIL2VONAL;
			} else {
				event = SZAGGATOTT2VONAL;
			}
		}
		if (haromvonal) {
			event = HAROMVONAL;
		}

	}

	if (event == lastEvent){
		if (cntr > 5) {
			cntr = 0;
			return event;
		} else {
			cntr++;
			return UNSTABLE;
		}
	} else {
		cntr = 0;
		lastEvent = event;
		event = UNSTABLE;
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

//TODO switch caset feltölteni, melyik eventre melyi kakadály állapotba lépjen
void KoztesState::update() {
	switch (stateData.event) {
		case NOLINE_NOWALLS:
			skillStateContext.setState(&SkillBaseState::TorkFalakKozt);
			break;
	}
}

//Moving
MovingState::MovingState(string stateName,
		SkillBaseState* nState,
		int howMuchToMove,
		float tSpeed,
		float angle,
		bool controlSteer) {
	name = stateName;
	stateId = 2;
	targetSpeed = tSpeed;
	distanceToMove = howMuchToMove;
	steeringAngle = angle;
	steeringControlled = controlSteer;
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
		float tSpeed,
		float angle,
		bool controlSteer,
		SkillTrackEvent targetEvent) {
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
	stateData.event = skillStateContext.state->calculateEvent();
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


GiroState::GiroState() {
	name = "giro";
	stateId = 4;
	targetSpeed = 0;
	distanceToMove = 0;
	//nextState =
}

//ebbe kell, hogy mikor lépjen a következõ eventbe a girostate
void GiroState::update() {

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


