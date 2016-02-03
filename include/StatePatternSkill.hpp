/*
 * StatePatternSkill.hpp
 *
 *  Created on: 2016. febr. 2.
 *      Author: Gabor
 */

#ifndef STATEPATTERNSKILL_HPP_
#define STATEPATTERNSKILL_HPP_

#include <string>
//#include <StateParkolas.hpp>

using namespace std;

extern int globalDistance;
extern float SKILLSLOW;

class SkillStateContext;

class KoztesState;
class SkillStopState;
class SkillStartState;

class MovingState;
class SkillBaseState;

enum SkillTrackEvent {
	STATEEND,
	PARKOLASSTART,
	TORKOLAT,
	TELEPHELY,
	HATAR,
	BILLENO,
	FORDITO,
	CEL,
	TWOWALL,
	RADIOSTART
};




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
	virtual void update(SkillStateContext& context, StateData data) {}
	virtual ~SkillBaseState(){}

};

class SkillStopState : public SkillBaseState {
public:
	SkillStopState();
	virtual void update(SkillStateContext& context, StateData data);
	//~SkillStopState() {}
};

class SkillStartState : public SkillBaseState {
public:
	SkillStartState();
	virtual void update(SkillStateContext& context, StateData data);
	//~SkillStartState() {}
};

class KoztesState : public SkillBaseState {
public:
	KoztesState();
	virtual void update(SkillStateContext& context, StateData data);
	//~KoztesState() {}
};

class MovingState : public SkillBaseState {
public:
	MovingState(string stateName, SkillBaseState* nState, int howMuchToMove, float angle, float tSpeed);
	virtual void update(SkillStateContext& context, StateData data);
	//~MovingState() {}
};

class EventBasedState : public SkillBaseState {
public:
	EventBasedState(string stateName, SkillBaseState* nState, int waitDistance, SkillTrackEvent targetEvent, float tSpeed);
	virtual void update(SkillStateContext& context, StateData data);
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
