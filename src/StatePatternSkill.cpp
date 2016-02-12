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
MovingState SkillBaseState::TorkFalakElhagyva("Tork2", &SkillBaseState::TorkVonalKereses, 750, SKILLSLOW, 500, false);
EventBasedState SkillBaseState::TorkVonalKereses("Tork3", &SkillBaseState::TorkVonalKereses2, 0, SKILLSLOW, 0, false, NONE);
MovingState SkillBaseState::TorkVonalKereses2("Tork4", &SkillBaseState::koztes, 1000, SKILLSLOW, 0, true);

//Parkol
EventBasedState SkillBaseState::ParkEloremegy1("P1Elore", &SkillBaseState::ParkEloremegy2, 1500, SKILLSLOW, 0, true, TWOWALL);
MovingState SkillBaseState::ParkEloremegy2("P2Elore", &SkillBaseState::ParkTolatKanyar1, 300, SKILLSLOW, 0, true);
MovingState SkillBaseState::ParkTolatKanyar1("P3Tolat", &SkillBaseState::ParkTolatKanyar2, -700, -SKILLSLOW, -500, false);
MovingState SkillBaseState::ParkTolatAtlo("P4Tolat", &SkillBaseState::ParkTolatKanyar2, -10, -SKILLSLOW, 0, false);
MovingState SkillBaseState::ParkTolatKanyar2("P5Tolat", &SkillBaseState::ParkTolatEgyenesen, -700, -SKILLSLOW, 500, false);
MovingState SkillBaseState::ParkTolatEgyenesen("P6Tolat", &SkillBaseState::ParkVar, 750, -SKILLSLOW, 0, false);

TimeState SkillBaseState::ParkVar("PVar", &SkillBaseState::ParkKiKanyar1, 100);

MovingState SkillBaseState::ParkKiKanyar1("P7Elore", &SkillBaseState::ParkKiAtlo, 750, SKILLSLOW, -500, false);
EventBasedState SkillBaseState::ParkKiAtlo("P8Elore", &SkillBaseState::ParkKiTeljesen, 0, SKILLSLOW, 0, false, NONE);
EventBasedState SkillBaseState::ParkKiTeljesen("P9Ki", &SkillBaseState::koztes, 2000, SKILLSLOW, 0, true, NOLINE_NOWALLS);

//giro
EventBasedState SkillBaseState::giroStart("giroStart", &SkillBaseState::giroFel, 0, SKILLSLOW, 0, true, KERESZT);
EventBasedState SkillBaseState::giroFel("giroFel", &SkillBaseState::giroPark, 200, SKILLSLOW, 0, true, KERESZT);
MovingState SkillBaseState::giroPark("giroPark", &SkillBaseState::giro, 100, SKILLSLOW, 0, true);

GiroState SkillBaseState::giro("Giro", &SkillBaseState::giroLejon, true);
MovingState SkillBaseState::giroLejon("giroLejon", &SkillBaseState::skillStopped, 2000, SKILLSLOW, 0, true);


//libikoka
GiroState SkillBaseState::libikoka("libikoka", &SkillBaseState::libiLassu, false);
MovingState SkillBaseState::libiLassu("libiLassu", &SkillBaseState::skillStopped, 100, SKILLSLOW, 0, true);

// határ
MovingState SkillBaseState::hatarStart("hatarStart", &SkillBaseState::hatarWait, 250, SKILLSLOW, 0, true);
HatarState SkillBaseState::hatarWait("hatarWait", &SkillBaseState::hatarMove);
MovingState SkillBaseState::hatarMove("hatarMove", &SkillBaseState::skillStopped, 2500, SKILLSLOW, 0, true);


SkillTrackEvent SkillBaseState::calculateEvent() {
	static SkillTrackEvent lastEvent;
	static int cntr = 0;
	SkillTrackEvent event = NONE;
//todo kirakni


	if (keresztvonal) {
		return KERESZT;
	}

	if(fal_bal && fal_jobb) {
		event = TWOWALL;
	} else if (stable0lines) {
		event = NOLINE_NOWALLS;
	} else if (stable1linesForBoth || stable2lines || stable3lines){
		//hany vonal van todo
		if (stable1linesForBoth) {
			event = NONE;
		} else if (stable2lines) {
			if (fal_jobb || fal_bal) {
				event = STABIL2VONAL;
			} else {
				event = SZAGGATOTT2VONAL;
			}
		} else if (stable3lines) {
			event = HAROMVONAL;
		}

	} else {
		event = UNSTABLE;
	}

	if (event == lastEvent){
		if (cntr > 10) {
			//cntr = 0;
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
	steeringAngle = 1;
	steeringControlled = true;
}

//TODO switch caset feltölteni, melyik eventre melyi kakadály állapotba lépjen
void KoztesState::update() {
	stateData.event = skillStateContext.state->calculateEvent();

	switch (stateData.event) {
		case NOLINE_NOWALLS:
			skillStateContext.setState(&SkillBaseState::TorkFalakKozt);
			break;
		case KERESZT:
			skillStateContext.setState(&SkillBaseState::skillStopped);
			break;
		case SZAGGATOTT2VONAL:
			skillStateContext.setState(&SkillBaseState::giroStart);
			break;
		case HAROMVONAL:
			skillStateContext.setState(&SkillBaseState::hatarStart);
			break;
		case TWOWALL:
			skillStateContext.setState(&SkillBaseState::ParkEloremegy1);
			break;
	}
}

//Moving
MovingState::MovingState(string stateName,
		SkillBaseState* nState,
		int howMuchToMove,
		float tSpeed,
		int angle,
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
		int angle,
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


GiroState::GiroState(string stateName, SkillBaseState* nState, bool Z) {
	name = stateName;
	stateId = 4;
	targetSpeed = 0;
	distanceToMove = 0;
	nextState = nState;
	startAngle = 0;
	started = false;
	zAxis = Z;
}

extern bool giro_stopped;
extern bool giro_fall;
extern bool giro_downAngle;
//ebbe kell, hogy mikor lépjen a következõ eventbe a girostate
void GiroState::update() {
	if (!started) {
		giro_start_measurement();
		started = true;
	} else {
		if (zAxis) {

			float angle = giro_get_angle_Z();
			if ((angle < 135 && angle > 45) || (angle < -225 && angle > -315) ) {
				if (giro_stopped) {
					started = false;
					skillStateContext.setState(this->nextState);
				}

			}
		} else {
			float angle = giro_get_angle_Y();
			if (angle < -10) {
				started = false;
				skillStateContext.setState(this->nextState);
			}

		}
	}
}

//Idore varo state
TimeState::TimeState(string stateName,
		SkillBaseState* nState,
		int wait) {
	name = stateName;
	stateId = 5;
	targetSpeed = 0;
	distanceToMove = wait;
	nextState = nState;
	started = false;
	triggerTime = 100000000;
	startTime = 0;
}

//ebbe kell, hogy mikor lépjen a következõ eventbe a
void TimeState::update() {
	if (!started) {
		startTime = timerCounter;
		triggerTime = startTime + distanceToMove;
		started = true;
	} else {
		if (timerCounter > triggerTime) {
			skillStateContext.setState(this->nextState);
		}
	}

}

//hataratkelo
HatarState::HatarState(string stateName,
		SkillBaseState* nState) {
	name = stateName;
	stateId = 6;
	targetSpeed = 0;
	distanceToMove = 0;
	nextState = nState;
}

//ebbe kell, hogy mikor lépjen a következõ eventbe a
void HatarState::update() {
	if(ReadFrontLeft() < 40)
	{
		direction = LEFT;
		skillStateContext.setState(this->nextState);
	}else if(ReadFrontRight() < 40)
	{
		direction = RIGHT;
		skillStateContext.setState(this->nextState);
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
				skillStateContext.setState(&SkillBaseState::koztes);
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


