/*
 * StatePatternSkill.hpp
 *
 *  Created on: 2016. febr. 2.
 *      Author: Gabor
 */

#ifndef STATEPATTERNSKILL_HPP_
#define STATEPATTERNSKILL_HPP_

#include <config.hpp>
//#include <StateParkolas.hpp>

using namespace std;

class SkillStateContext;

class KoztesState;
class SkillStopState;
class SkillStartState;

class ParkolasStart;
class Tolat1State;
class Tolat2State;
class Vissza1State;
class Vissza2State;


class SkillBaseState {

public:
	static SkillStopState skillStopped;
	static SkillStartState skillStarted;
	static KoztesState koztes;

	static ParkolasStart parkolasStart;
	static Tolat1State tolat1;
	static Tolat2State tolat2;
	static Vissza1State vissza1;
	static Vissza2State vissza2;

	int stateId;
	float targetSpeed;

	bool steeringController;
	float steering;

	int targetDistance;
	int targetDifference;

	void stop(SkillStateContext& context);
	virtual void update(SkillStateContext& context, StateData data);
	virtual ~SkillBaseState() {}

};

class SkillStopState : public SkillBaseState {
public:
	SkillStopState();
	void update(SkillStateContext& context, StateData data);
};

class SkillStartState : public SkillBaseState {
public:
	SkillStartState();
	void update(SkillStateContext& context, StateData data);
};

class KoztesState : public SkillBaseState {
public:
	KoztesState();
	void update(SkillStateContext& context, StateData data);
};

class ParkolasStart : public SkillBaseState {
public:
	ParkolasStart();
	virtual void update(SkillStateContext& context, StateData data);
};

class Tolat1State : public SkillBaseState {
public:
	Tolat1State();
	virtual void update(SkillStateContext& context, StateData data);
};

class Tolat2State : public SkillBaseState {
public:
	Tolat2State();
	virtual void update(SkillStateContext& context, StateData data);
};

class Vissza1State : public SkillBaseState {
public:
	Vissza1State();
	virtual void update(SkillStateContext& context, StateData data);
};

class Vissza2State : public SkillBaseState {
public:
	Vissza2State();
	virtual void update(SkillStateContext& context, StateData data);
};



class SkillStateContext {

public:
	SkillBaseState* state;

	SkillStateContext();
	void setState(SkillBaseState *newState);
	void stop();
	void start();

};





#endif /* STATEPATTERNSKILL_HPP_ */
