/*
 * StatePattern.cpp
 *
 *  Created on: 2016. jan. 6.
 *      Author: Gabor
 */

#include "StatePattern.hpp"

KanyarState BaseState::kanyar;
GyorsitoState BaseState::gyorsito;
GyorsState BaseState::gyors;
LassitoState BaseState::lassito;

//Kanyar közben
KanyarState::KanyarState() {
	steeringPD = false;
	targetSpeed = SLOW;
}

void KanyarState::handleEvent(StateContext& context, Event event) {
	if (event == GYORSITO) {
		context.setState(&BaseState::gyorsito);
	}
}


//Gyorsításkor
GyorsitoState::GyorsitoState() {
	steeringPD = true;
	targetSpeed = FAST;
}

void GyorsitoState::handleEvent(StateContext& context, Event event) {
	if (event == SIMA) {
		context.setState(&BaseState::gyors);
	}
}

//Gyors szakaszon
GyorsState::GyorsState() {
	steeringPD = true;
	targetSpeed = FAST;
}

void GyorsState::handleEvent(StateContext& context, Event event) {
	if (event == GYORSITO) {
		context.setState(&BaseState::lassito);
	}
}

//Lassításkor
LassitoState::LassitoState() {
	steeringPD = true;
	targetSpeed = SLOW;
}

void LassitoState::handleEvent(StateContext& context, Event event) {
	if (event == SIMA) {
		context.setState(&BaseState::kanyar);
	}
}

//StateContext
void StateContext::handleEvent(Event event) {
	state->handleEvent(*this, event);
}

void StateContext::setState(BaseState* newState){
	state = newState;
}

void StateContext::update(bool stable3lines){
	if (stable3lines) {
		handleEvent(GYORSITO);
	} else {
		handleEvent(SIMA);
	}
}

float StateContext::getTargetSpeed(){
	return state->targetSpeed;
}

bool StateContext::isSteeringPD(){
	return state->steeringPD;
}


