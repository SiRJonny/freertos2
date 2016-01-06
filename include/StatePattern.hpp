/*
 * StatePattern.hpp
 *
 *  Created on: 2016. jan. 6.
 *      Author: Gabor
 */

#ifndef STATEPATTERN_HPP_
#define STATEPATTERN_HPP_

class StateContext;
class KanyarState;
class GyorsitoState;
class GyorsState;
class LassitoState;

extern float SLOW;
extern float FAST;
extern float SET_SPEED;

enum Event {
	GYORSITO,
	LASSITO,
	SIMA
};

class BaseState {

public:
	static KanyarState kanyar;
	static GyorsitoState gyorsito;
	static GyorsState gyors;
	static LassitoState lassito;

	bool steeringPD;
	float targetSpeed;

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



class StateContext {
private:
	BaseState* state;
public:
	void handleEvent(Event event);
	void setState(BaseState *newState);
	void update(bool stable3lines);

	float getTargetSpeed();
	bool isSteeringPD();


};

#endif /* STATEPATTERN_HPP_ */
