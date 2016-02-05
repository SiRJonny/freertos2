/*
 * StatePatternSkill.hpp
 *
 *  Created on: 2016. febr. 2.
 *      Author: Gabor
 */

#ifndef STATEPATTERNSKILL_HPP_
#define STATEPATTERNSKILL_HPP_

#include <string>
#include "ProcessSensors.h"


using namespace std;

extern int globalDistance;
extern float SKILLSLOW;



class SkillStateContext;
class StateData;

class KoztesState;
class SkillStopState;
class SkillStartState;

class MovingState;
class EventBasedState;
class SkillBaseState;

extern StateData stateData;
extern SkillStateContext skillStateContext;

extern bool fal_bal;
extern bool fal_jobb;
extern bool bordas_bal;
extern bool bordas_jobb;


extern float activeLine1;
extern float last_active_line_pos1;
extern LineState globalLines;

enum Direction {
	UNDEFINED,
	RIGHT,
	LEFT
};
extern Direction direction;

enum SkillTrackEvent {
	NONE,
	RADIOSTART,
	PARKOLAS,
	TORKOLAT,
	TELEPHELY,
	HATAR,
	BILLENO,
	FORDITO,
	CEL,
	TWOWALL,
	TWOWALL_BORDAS_RIGHT,
	TWOWALL_BORDAS_LEFT,
	WALL_BORDAS_LEFT,
	WALL_BORDAS_RIGHT,
	NOWALLS,
	NEWLINE

};
extern SkillTrackEvent currentState;



class StateData {
public:
	int lines;
	float sensorLeft;
	float sensorRight;
	float sensorFront;
	float speed;
	SkillTrackEvent event;
};



class SkillBaseState {

public:
	static SkillStopState skillStopped;
	static SkillStartState skillStarted;
	static KoztesState koztes;

	static MovingState parkolasTolat1;
	static MovingState parkolasTolat2;

	//Torkolat
	static EventBasedState TorkFalakKozt;
	static MovingState TorkFalakElhagyva;
	static EventBasedState TorkVonalKereses;

	string name;
	int stateId;
	float targetSpeed;

	bool steeringControlled;

	float steeringAngle;
	int distanceToMove;
	SkillTrackEvent triggerEvent;

	int triggerGlobalDistance;

	SkillBaseState* nextState;

	void stop(SkillStateContext& context);
	SkillTrackEvent calculateEvent();
	virtual void update() {}
	virtual ~SkillBaseState(){}

};

class SkillStopState : public SkillBaseState {
public:
	SkillStopState();
	virtual void update();
	//~SkillStopState() {}
};

class SkillStartState : public SkillBaseState {
public:
	SkillStartState();
	virtual void update();
	//~SkillStartState() {}
};

class KoztesState : public SkillBaseState {
public:
	KoztesState();
	virtual void update();
	//~KoztesState() {}
};

class MovingState : public SkillBaseState {
public:
	MovingState(string stateName, SkillBaseState* nState, int howMuchToMove, float angle, float tSpeed);
	virtual void update();
	//~MovingState() {}
};

class EventBasedState : public SkillBaseState {
public:
	EventBasedState(string stateName, SkillBaseState* nState, int waitDistance, SkillTrackEvent targetEvent, float tSpeed, float angle, bool controlSteer);
	virtual void update();
	//~EventBasedState() {}
};


class SkillStateContext {

public:
	SkillBaseState* state;

	SkillStateContext();
	void setState(SkillBaseState *newState);
	//void stop();
	//void start();

};





#endif /* STATEPATTERNSKILL_HPP_ */
