/*
 * StateParkolas.cpp
 *
 *  Created on: 2016. febr. 2.
 *      Author: Gabor
 */

#include <StatePatternSkill.hpp>


ParkolasStart SkillBaseState::parkolasStart;
Tolat1State SkillBaseState::tolat1;
Tolat2State SkillBaseState::tolat2;
Vissza1State SkillBaseState::vissza1;
Vissza2State SkillBaseState::vissza2;

//Tolatas kezdes
ParkolasStart::ParkolasStart() {
	stateId = 1;
	targetDistance = 10;
	targetSpeed = -1.0;
	steering = 10;
}

void ParkolasStart::update(SkillStateContext& context, StateData data){

}

Tolat1State::Tolat1State() {
	stateId = 1;
	targetDistance = 10;
	targetSpeed = -1.0;
	steering = 10;
}

void Tolat1State::update(SkillStateContext& context, StateData data){

}

Tolat2State::Tolat2State() {
	stateId = 1;
	targetDistance = 10;
	targetSpeed = -1.0;
	steering = 10;
}

void Tolat2State::update(SkillStateContext& context, StateData data){

}

Vissza1State::Vissza1State() {
	stateId = 1;
	targetDistance = 10;
	targetSpeed = -1.0;
	steering = 10;
}

void Vissza1State::update(SkillStateContext& context, StateData data){

}

Vissza2State::Vissza2State() {
	stateId = 1;
	targetDistance = 10;
	targetSpeed = -1.0;
	steering = 10;
}

void Vissza2State::update(SkillStateContext& context, StateData data){

}
