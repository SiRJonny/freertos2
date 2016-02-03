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

extern float SLOW;
extern float FAST;
extern float SET_SPEED;

enum Event {
	GYORSITO,
	LASSITO,
	SIMA,
	START
};

class BaseState {

public:
	static KanyarState kanyar;
	static GyorsitoState gyorsito;
	static GyorsState gyors;
	static LassitoState lassito;
	static StopState stopped;
	static StartState started;

	int stateId;
	bool steeringPD;
	float targetSpeed;
	int encoderPosDifference;
	int targetEncoderPos;

	void stop(StateContext& context);
	virtual void handleEvent(StateContext& context, Event event);
	virtual ~BaseState() {}

};

class KanyarState : public BaseState {
public:
	KanyarState();
	virtual void handleEvent(StateContext& context, Event event);
};

class GyorsitoState : public BaseState {
public:
	GyorsitoState();
	virtual void handleEvent(StateContext& context, Event event);
};

class GyorsState : public BaseState {
public:
	GyorsState();
	virtual void handleEvent(StateContext& context, Event event);
};

class LassitoState : public BaseState {
public:
	LassitoState();
	virtual void handleEvent(StateContext& context, Event event);
};


class StopState : public BaseState {
public:
	StopState();
	virtual void handleEvent(StateContext& context, Event event);
};

class StartState : public BaseState {
public:
	StartState();
	virtual void handleEvent(StateContext& context, Event event);
};


class StateContext {
private:

public:
	BaseState* state;
	int currEncoderPos;

	StateContext();

	void handleEvent(Event event);
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
