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


//start
EventBasedState SkillBaseState::startWait("sWait", &SkillBaseState::startMove, 0, 0, 0, true, RADIOSTART, true);
MovingState SkillBaseState::startMove("sMove", &SkillBaseState::koztes, 1000, SKILLSLOW, 0, true, true);


//Torkolat
EventBasedState SkillBaseState::TorkFalakKozt("Tork1", &SkillBaseState::TorkFalakElhagyva, 300, SKILLSLOW, 0, false, NOLINE_NOWALLS, true);
MovingState SkillBaseState::TorkFalakElhagyva("Tork2", &SkillBaseState::TorkVonalKereses, 750, SKILLSLOW, 500, false, true);
EventBasedState SkillBaseState::TorkVonalKereses("Tork3", &SkillBaseState::TorkVonalKereses2 , 0, SKILLSLOW, 0, false, NONE, true);
MovingState SkillBaseState::TorkVonalKereses2("Tork4", &SkillBaseState::koztes, 500, SKILLSLOW, 0, true, true);

//Parkol
EventBasedState SkillBaseState::ParkEloremegy1("P1Elore", &SkillBaseState::ParkEloremegy2, 1500, SKILLSLOW, 0, true, TWOWALL, false);
MovingState SkillBaseState::ParkEloremegy2("P2Elore", &SkillBaseState::parkElolVar, 420, SKILLPARK, 0, true, false);

EventBasedState SkillBaseState::ParkElolTolat("P251T", &SkillBaseState::ParkTolatKanyar1, 1500, -0.1, 0, true, TWOWALL, false);



TimeState SkillBaseState::parkElolVar("PEVar", &SkillBaseState::ParkTolatKanyar1, 200, 0);

MovingState SkillBaseState::ParkTolatKanyar1("P3Tolat", &SkillBaseState::ParkTolatKanyar2, -720, -SKILLPARK, 500, false, false);
MovingState SkillBaseState::ParkTolatAtlo("P4Tolat", &SkillBaseState::ParkTolatKanyar2, -50, -SKILLPARK, 0, false, false);
MovingState SkillBaseState::ParkTolatKanyar2("P5Tolat", &SkillBaseState::ParkVar, -710, -0.1, -500, false, false);
MovingState SkillBaseState::ParkTolatEgyenesen("P6Tolat", &SkillBaseState::ParkVar, -60, -SKILLPARK, 0, false, false);

TimeState SkillBaseState::ParkVar("PVar", &SkillBaseState::ParkKiKanyar1, 125, -0.1);

MovingState SkillBaseState::ParkKiKanyar1("P7Elore", &SkillBaseState::ParkKiAtlo, 500, SKILLPARK, -500, false, false);
EventBasedState SkillBaseState::ParkKiAtlo("P8Elore", &SkillBaseState::ParkKiTeljesen, 0, SKILLPARK, 0, false, NONE, false);
EventBasedState SkillBaseState::ParkKiTeljesen("P9Ki", &SkillBaseState::koztes, 1000, SKILLPARK, 0, true, NONE, false);

//giro
EventBasedState SkillBaseState::giroStart("giroSt", &SkillBaseState::giroFel, 0, SKILLSLOW, 0, true, KERESZT, true);
EventBasedState SkillBaseState::giroFel("giroFel", &SkillBaseState::giroPark, 350, SKILLSLOW, 0, true, KERESZT, true);
MovingState SkillBaseState::giroPark("giroPark", &SkillBaseState::giro, 100, SKILLSLOW, 0, true, true);

GiroState SkillBaseState::giro("Giro", &SkillBaseState::giroLejon, true, 0);
MovingState SkillBaseState::giroLejon("giroLe", &SkillBaseState::koztes, 1000, SKILLSLOW, 0, true, true);

// határ
TimeState SkillBaseState::hTime("hTime", &SkillBaseState::hatarWait, 300, 0);

MovingState SkillBaseState::hatarStart("hStart", &SkillBaseState::hatarWait, 2000, SKILLSLOW, 0, true, true);
HatarState SkillBaseState::hatarWait("hWait", &SkillBaseState::hatarMove);
MovingState SkillBaseState::hatarMove("hMove", &SkillBaseState::libStart, 2000, SKILLSLOW, 0, true, true);

//libikoka

LibiState SkillBaseState::libStart("lStart", &SkillBaseState::libPos, &SkillBaseState::libVeg);

MovingState SkillBaseState::libPos("libPos", &SkillBaseState::libiStop, 1250, SKILLSLOW, 0, true, true);

GiroState SkillBaseState::libikoka("lkoka", &SkillBaseState::libiStop, false, SKILLSLOW);
TimeState SkillBaseState::libiStop("lStop", &SkillBaseState::lejto, 300, 0.01);
EventBasedState SkillBaseState::lejto("lLejt", &SkillBaseState::libVeg, 200, SKILLSLOW, 0, true, SZAGGATOTT2VONAL, true);

MovingState SkillBaseState::libVeg("libKi", &SkillBaseState::koztes, 500, SKILLSLOW, 0, true, true);


//lerako
EventBasedState SkillBaseState::lerakoStart("leStart", &SkillBaseState::lerakoVeg, 1500, SKILLSLOW, 0, true, SZAGGATOTT2VONAL, true);
MovingState SkillBaseState::lerakoVeg("leVeg", &SkillBaseState::koztes, 1000, SKILLSLOW, 0, true, true);


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
	} else if (stable0linesForBoth) {
		event = NOLINE_NOWALLS;
	} else if (stable1linesForBoth || stable2lines || stable3lines){

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
		if (cntr > 0) {
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
	targetSpeed = SKILLMEDIUM;
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
		case STABIL2VONAL:
			skillStateContext.setState(&SkillBaseState::lerakoStart);
			break;
		case HAROMVONAL:
			skillStateContext.setState(&SkillBaseState::hTime);
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
		int wait,
		float tarSpeed) {
	name = stateName;
	stateId = 5;
	targetSpeed = tarSpeed;
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
			started = false;
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
	giro_start_measurement();
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

//libikoka elejehez
LibiState::LibiState(string stateName,
		SkillBaseState* nState1,
		SkillBaseState* nState2) {
	name = stateName;
	stateId = 7;
	targetSpeed = SKILLSLOW;
	distanceToMove = 0;
	nextState = nState1;
	nextState2 = nState2;
	steeringControlled = true;
	chkDir = true;
}

//ebbe kell, hogy mikor lépjen a következõ eventbe a
void LibiState::update() {
	stateData.event = skillStateContext.state->calculateEvent();

	if (giro_emelkedo) {
		skillStateContext.setState(this->nextState);
	} else if (stateData.event == SZAGGATOTT2VONAL) {
		skillStateContext.setState(this->nextState2);
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
	steeringControlled = true;
}

void SkillStartState::update(){
	switch (stateData.event) {
			case RADIOSTART:
				skillStateContext.setState(&SkillBaseState::startMove);
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
	context.setState(&SkillBaseState::skillStopped);
}


