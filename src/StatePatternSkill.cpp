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
			bool controlSteer,
			bool checkDir);

EventBasedState(string stateName,
			SkillBaseState* nState,
			int waitDistance,
			float tSpeed,
			float angle,
			bool controlSteer,
			SkillTrackEvent targetEvent,
			bool checkDir);
			*/

//Teszt
MovingState SkillBaseState::tolatTeszt("tolat", &SkillBaseState::eloreTeszt, -750, -SKILLSLOW, 0, false, true);
MovingState SkillBaseState::eloreTeszt("elore", &SkillBaseState::tolatTeszt, 750, SKILLSLOW, 0, false, true);


//Torkolat
EventBasedState SkillBaseState::TorkFalakKozt("Tork1", &SkillBaseState::TorkFalakElhagyva, 300, SKILLSLOW, 0, false, NOLINE_NOWALLS, true);
MovingState SkillBaseState::TorkFalakElhagyva("Tork2", &SkillBaseState::TorkVonalKereses, 750, SKILLSLOW, 500, false, true);
EventBasedState SkillBaseState::TorkVonalKereses("Tork3", &SkillBaseState::TorkVonalKereses2, 0, SKILLSLOW, 0, false, NONE, true);
MovingState SkillBaseState::TorkVonalKereses2("Tork4", &SkillBaseState::koztes, 1000, SKILLSLOW, 0, true, true);

//Parkol
EventBasedState SkillBaseState::ParkEloremegy1("P1Elore", &SkillBaseState::ParkEloremegy2, 1500, SKILLSLOW, 0, true, TWOWALL, false);
MovingState SkillBaseState::ParkEloremegy2("P2Elore", &SkillBaseState::ParkTolatKanyar1, 420, SKILLSLOW, 0, true, false);
MovingState SkillBaseState::ParkTolatKanyar1("P3Tolat", &SkillBaseState::ParkTolatKanyar2, -710, -SKILLSLOW, 500, false, false);
MovingState SkillBaseState::ParkTolatAtlo("P4Tolat", &SkillBaseState::ParkTolatKanyar2, -50, -SKILLSLOW, 0, false, false);
MovingState SkillBaseState::ParkTolatKanyar2("P5Tolat", &SkillBaseState::ParkTolatEgyenesen, -710, -SKILLSLOW, -500, false, false);
MovingState SkillBaseState::ParkTolatEgyenesen("P6Tolat", &SkillBaseState::ParkVar, -60, -SKILLSLOW, 0, false, false);

TimeState SkillBaseState::ParkVar("PVar", &SkillBaseState::ParkKiKanyar1, 200);

MovingState SkillBaseState::ParkKiKanyar1("P7Elore", &SkillBaseState::ParkKiAtlo, 500, SKILLSLOW, -500, false, false);
EventBasedState SkillBaseState::ParkKiAtlo("P8Elore", &SkillBaseState::ParkKiTeljesen, 0, SKILLSLOW, 0, false, NONE, false);
EventBasedState SkillBaseState::ParkKiTeljesen("P9Ki", &SkillBaseState::koztes, 300, SKILLSLOW, 0, true, NONE, false);

//giro
EventBasedState SkillBaseState::giroStart("giroStart", &SkillBaseState::giroFel, 0, SKILLSLOW, 0, true, KERESZT, true);
EventBasedState SkillBaseState::giroFel("giroFel", &SkillBaseState::giroPark, 200, SKILLSLOW, 0, true, KERESZT, true);
MovingState SkillBaseState::giroPark("giroPark", &SkillBaseState::giro, 100, SKILLSLOW, 0, true, true);

GiroState SkillBaseState::giro("Giro", &SkillBaseState::giroLejon, true, 0);
MovingState SkillBaseState::giroLejon("giroLejon", &SkillBaseState::skillStopped, 2000, SKILLSLOW, 0, true, true);



// határ
MovingState SkillBaseState::hatarStart("hatarStart", &SkillBaseState::hatarWait, 200, SKILLSLOW, 0, true, true);
HatarState SkillBaseState::hatarWait("hatarWait", &SkillBaseState::hatarMove);
MovingState SkillBaseState::hatarMove("hatarMove", &SkillBaseState::emelkedo_elott, 2500, SKILLSLOW, 0, true, true);

//libikoka

EventBasedState SkillBaseState::emelkedo_elott("emelElott", &SkillBaseState::emelkedo, 100, SKILLSLOW, 0, true, EMELKEDO, true);

EventBasedState SkillBaseState::emelkedo("libEmel", &SkillBaseState::libiStop, 200, SKILLSLOW, 0, true, NONE, true);
TimeState SkillBaseState::libiStop("libiStop", &SkillBaseState::lejto, 300);

EventBasedState SkillBaseState::lejto("libLejt", &SkillBaseState::libiLassu, 200, SKILLSLOW, 0, true, SZAGGATOTT2VONAL, true);
MovingState SkillBaseState::libiLassu("libiLassu", &SkillBaseState::koztes, 500, SKILLSLOW, 0, true, true);


MovingState SkillBaseState::libiStart("libiStart", &SkillBaseState::emelkedo, 1000, SKILLSLOW, 0, true, true);
GiroState SkillBaseState::libikoka("libikoka", &SkillBaseState::libiStop, false, SKILLSLOW);


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
	chkDir = true;
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
		bool controlSteer,
		bool checkDir) {
	name = stateName;
	stateId = 2;
	targetSpeed = tSpeed;
	distanceToMove = howMuchToMove;
	steeringAngle = angle;
	steeringControlled = controlSteer;
	nextState = nState;
	chkDir = checkDir;
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
		SkillTrackEvent targetEvent,
		bool checkDir) {
	name = stateName;
	stateId = 3;
	targetSpeed = tSpeed;
	distanceToMove = waitDistance;
	steeringAngle = angle;
	steeringControlled = controlSteer;
	nextState = nState;
	triggerEvent = targetEvent;
	chkDir = checkDir;
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


GiroState::GiroState(string stateName, SkillBaseState* nState, bool Z, float tarSpeed) {
	name = stateName;
	stateId = 4;
	targetSpeed = tarSpeed;
	distanceToMove = 0;
	nextState = nState;
	startAngle = 0;
	started = false;
	zAxis = Z;
	chkDir = true;
}


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
			if (giro_fall) {
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
	chkDir = false;
}


extern int timeCounter;
//ebbe kell, hogy mikor lépjen a következõ eventbe a
void TimeState::update() {
	if (!started) {
		startTime = timeCounter;
		triggerTime = startTime + distanceToMove;
		started = true;
	} else {
		if (timeCounter > triggerTime) {
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
	chkDir = true;
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
	chkDir = true;
}

void SkillStopState::update(){

}

//Start state
SkillStartState::SkillStartState() {
	name = "start";
	stateId = 0;
	targetSpeed = 0;
	chkDir = true;
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


