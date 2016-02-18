/*
 * StatePattern.hpp
 *
 *  Created on: 2016. jan. 6.
 *      Author: Gabor
 */

#ifndef STATEPATTERN_HPP_
#define STATEPATTERN_HPP_

#include <string>
using namespace std;

class StateContext;

class KanyarState;
class GyorsitoState;
class GyorsState;
class LassitoState;

class TavState;

class StopState;
class StartState;

class SafetyState;

extern volatile float SLOW;
extern volatile float FAST;
extern float SET_SPEED;

enum SpeedEvent {
	GYORSITO,
	LASSITO,
	SIMA,
	START,
	SEMMI,
	SPEEDUP
};

class BaseState {

public:
	static KanyarState kanyar;
	static GyorsitoState gyorsito;
	static GyorsState gyors;
	static LassitoState lassito;
	static StopState stopped;
	static StartState started;

	static SafetyState safetyLassit;
	static SafetyState safetyKanyar;
	static SafetyState safetyFast;


	static GyorsState gyors1;
	static LassitoState lassito1;
	static KanyarState kanyar1;
	static GyorsitoState gaz1;

	static GyorsState gyors2;
	static LassitoState lassito2;
	static KanyarState kanyar2;
	static GyorsitoState gaz2;

	static TavState gy3tav;
	static GyorsState gyors3;
	static LassitoState lassito3;
	static KanyarState kanyar3;
	static GyorsitoState gaz3;

	static GyorsState gyors4;
	static LassitoState lassito4;
	static KanyarState kanyar4;
	static GyorsitoState gaz4;

	string name;
	int stateId;
	bool steeringPD;
	float targetSpeed;

	bool isSafety;
	bool isSensorMoved;

	int distanceToMove;
	int triggerGlobalDistance;

	BaseState* nextState;
	SpeedEvent targetEvent;

	void stop(StateContext& context);
	virtual void handleEvent(StateContext& context, SpeedEvent event) {}
	virtual ~BaseState() {}

};

class KanyarState : public BaseState {
public:
	KanyarState(string stateName,
			BaseState* nState,
			int minWaitDistance,
			float tSpeed,
			SpeedEvent tEvent);
	virtual void handleEvent(StateContext& context, SpeedEvent event);
};



class GyorsitoState : public BaseState {
public:
	GyorsitoState(string stateName,
			BaseState* nState,
			int minWaitDistance,
			float tSpeed,
			SpeedEvent tEvent);
	virtual void handleEvent(StateContext& context, SpeedEvent event);
};

class GyorsState : public BaseState {
public:
	GyorsState(string stateName,
			BaseState* nState,
			int minWaitDistance,
			float tSpeed,
			SpeedEvent tEvent);
	virtual void handleEvent(StateContext& context, SpeedEvent event);
};

class TavState : public BaseState {
public:
	TavState(string stateName,
			BaseState* nState,
			int waitDistance,
			float tSpeed);
	virtual void handleEvent(StateContext& context, SpeedEvent event);
};

class LassitoState : public BaseState {
public:
	LassitoState(string stateName,
			BaseState* nState,
			int minWaitDistance,
			float tSpeed,
			SpeedEvent tEvent);
	virtual void handleEvent(StateContext& context, SpeedEvent event);
};


class StopState : public BaseState {
public:
	StopState();
	virtual void handleEvent(StateContext& context, SpeedEvent event);
};

class StartState : public BaseState {
public:
	StartState(string stateName,
			BaseState* nState,
			int minWaitDistance,
			float tSpeed,
			SpeedEvent tEvent);
	virtual void handleEvent(StateContext& context, SpeedEvent event);
};



class SafetyState : public BaseState {
public:


	SpeedEvent triggerEvent;


	SafetyState(int st_id, BaseState* nState, float maxSpeed, int howMuchToMove, bool moveSensor, SpeedEvent triggerSpeedEvent);
	virtual void handleEvent(StateContext& context, SpeedEvent event);
};


class StateContext {
private:

public:
	BaseState* state;
	int currEncoderPos;

	StateContext();

	void handleEvent(SpeedEvent event);
	void setState(BaseState *newState);
	void setTargetEncoderPos(int target);
	void stop();
	void start(int encoderPos);
	void update(bool stable3lines, int encoderPos);



};

#endif /* STATEPATTERN_HPP_ */
