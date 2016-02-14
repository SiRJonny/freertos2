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
#include "ReadSensors.h"
#include "EnumsStructs.hpp"
#include "giro.hpp"



using namespace std;

extern int globalDistance;
extern float SKILLSLOW;
extern float SKILLMEDIUM;
extern float SKILLPARK;

class SkillStateContext;
class StateData;

class KoztesState;
class SkillStopState;
class SkillStartState;
class TimeState;

class MovingState;
class EventBasedState;
class SkillBaseState;
class GiroState;
class HatarState;
class LibiState;

extern StateData stateData;
extern SkillStateContext skillStateContext;

extern bool fal_bal;
extern bool fal_jobb;
extern bool bordas_bal;
extern bool bordas_jobb;


extern float activeLine1;
extern float last_active_line_pos1;
extern LineState globalLines;

extern bool stable0lines;
extern bool stable1lines;
extern bool stable2lines;
extern bool stable3lines;

extern bool stable1linesForBoth;
extern bool stable0linesForBoth;


extern bool keresztvonal;
extern bool checkDirection;

extern SkillTrackEvent currentState;

extern bool giro_stopped;
extern bool giro_lejto;
extern bool giro_fall;
extern bool giro_emelkedo;


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

	static MovingState tolatTeszt;
	static MovingState eloreTeszt;

	//start
	static EventBasedState startWait;
	static MovingState startMove;


	//Torkolat
	static EventBasedState TorkFalakKozt;
	static MovingState TorkFalakElhagyva;
	static EventBasedState TorkVonalKereses;
	static MovingState TorkVonalKereses2;

	//Parkolas
	static EventBasedState ParkEloremegy1; 	//masodik 2 falig
	static MovingState ParkEloremegy2;		//kicsit még elõrébb?
	static TimeState parkElolVar;
	static EventBasedState ParkElolTolat;

	static MovingState ParkTolatKanyar1;
	static MovingState ParkTolatAtlo;
	static MovingState ParkTolatKanyar2;
	static MovingState ParkTolatEgyenesen;

	static TimeState ParkVar;

	static MovingState ParkKiKanyar1;
	static EventBasedState ParkKiAtlo;
	static EventBasedState ParkKiTeljesen;

	//giro
	static EventBasedState giroStart;
	static EventBasedState giroFel;
	static MovingState giroPark;

	static GiroState giro;
	static MovingState giroLejon;


	//határ
	static TimeState hTime;

	static MovingState hatarStart;
	static HatarState hatarWait;
	static MovingState hatarMove;

	//libikoka

	static LibiState libStart;
	static GiroState libikoka;

	static MovingState libPos;

	static TimeState libiStop;
	static EventBasedState lejto;
	static MovingState libVeg;

	//lerak
	static EventBasedState lerakoStart;

	static MovingState lerakoVeg;

	string name;
	int stateId;
	float targetSpeed;
	bool chkDir;

	bool steeringControlled;

	int steeringAngle;
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
	MovingState(string stateName,
			SkillBaseState* nState,
			int howMuchToMove,
			float tSpeed,
			int angle,
			bool controlSteer,
			bool checkDir);
	virtual void update();
	//~MovingState() {}
};

class EventBasedState : public SkillBaseState {
public:
	EventBasedState(string stateName,
			SkillBaseState* nState,
			int waitDistance,
			float tSpeed,
			int angle,
			bool controlSteer,
			SkillTrackEvent targetEvent,
			bool checkDir);
	virtual void update();
	//~EventBasedState() {}
};

class GiroState : public SkillBaseState {
public:
	bool started;
	float startAngle; //kell?
	bool zAxis;

	GiroState(string stateName, SkillBaseState* nState, bool Z, float tarSpeed);
	virtual void update();
	//~EventBasedState() {}
};

class TimeState : public SkillBaseState {
public:
	bool started;
	int startTime;
	int triggerTime;
	TimeState(string stateName,
			SkillBaseState* nState,
			int wait,
			float tarSpeed);
	virtual void update();
};

class HatarState : public SkillBaseState {
public:
	HatarState(string stateName,
			SkillBaseState* nState);
	virtual void update();
};

class LibiState : public SkillBaseState {
public:
	SkillBaseState* nextState2;
	LibiState(string stateName,
			SkillBaseState* nState1,
			SkillBaseState* nState2);
	virtual void update();
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
