/*
 * StatePattern.hpp
 *
 *  Created on: 2016. jan. 6.
 *      Author: Gabor
 */

#ifndef STATEPATTERN_HPP_
#define STATEPATTERN_HPP_

using namespace std;

class StateContext;

class KanyarState;
class GyorsitoState;
class GyorsState;
class LassitoState;
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

	static SafetyState safetySlow;
	static SafetyState safetyFast;

	int stateId;
	bool steeringPD;
	float targetSpeed;

	int distanceToMove;
	int triggerGlobalDistance;

	void stop(StateContext& context);
	virtual void handleEvent(StateContext& context, SpeedEvent event) {}
	virtual ~BaseState() {}

};

class KanyarState : public BaseState {
public:
	KanyarState();
	virtual void handleEvent(StateContext& context, SpeedEvent event);
};

class GyorsitoState : public BaseState {
public:
	GyorsitoState();
	virtual void handleEvent(StateContext& context, SpeedEvent event);
};

class GyorsState : public BaseState {
public:
	GyorsState();
	virtual void handleEvent(StateContext& context, SpeedEvent event);
};

class LassitoState : public BaseState {
public:
	LassitoState();
	virtual void handleEvent(StateContext& context, SpeedEvent event);
};


class StopState : public BaseState {
public:
	StopState();
	virtual void handleEvent(StateContext& context, SpeedEvent event);
};

class StartState : public BaseState {
public:
	StartState();
	virtual void handleEvent(StateContext& context, SpeedEvent event);
};

class SafetyState : public BaseState {
public:
	bool isSafety;
	bool isSensorMoved;
	BaseState* nextState;
	SpeedEvent triggerEvent;


	SafetyState(BaseState* nState, float maxSpeed, int howMuchToMove, bool moveSensor, SpeedEvent triggerSpeedEvent);
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

	float getTargetSpeed();
	bool isSteeringPD();
	int getStateId();


};

#endif /* STATEPATTERN_HPP_ */
