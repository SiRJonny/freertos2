/*
 * StatePatternSkill.cpp
 *
 *  Created on: 2016. febr. 2.
 *      Author: Gabor
 */

#include "StatePatternSkill.hpp"

KoztesState SkillBaseState::koztes;
SkillStopState SkillBaseState::skillStopped;
SkillStartState SkillBaseState::skillStarted;

//Koztes
KoztesState::KoztesState() {
	stateId = 1;
	targetSpeed = SKILLSLOW;
}

void KoztesState::update(SkillStateContext& context, StateData data) {
	switch (data.event) {
		case PARKOLAS:
			context.setState(&SkillBaseState::parkolasStart);
			break;
	}
}


//Stop state
SkillStopState::SkillStopState() {
	stateId = -1;
	targetSpeed = 0;
}

void SkillStopState::update(SkillStateContext& context, StateData data){

}

//Start state
SkillStartState::SkillStartState() {
	stateId = 0;
	targetSpeed = SLOW;
	targetDifference = 0;
}

void SkillStartState::update(SkillStateContext& context, StateData data){

}

//SkillStateContext

void SkillStateContext::setState(SkillBaseState* newState) {
	state = newState;
}

SkillStateContext::SkillStateContext() {
	setState(&SkillBaseState::skillStarted);
}



