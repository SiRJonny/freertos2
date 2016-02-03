/*
 * StateParkolas.cpp
 *
 *  Created on: 2016. febr. 2.
 *      Author: Gabor
 */

#include <StatePatternSkill.hpp>


//MovingState(string stateName,SkillBaseState* nextState, int howMuchToMove, float angle, float tSpeed) {
MovingState SkillBaseState::parkolasTolat1("park1", &SkillBaseState::parkolasTolat2, -10, 0, -SKILLSLOW);
MovingState SkillBaseState::parkolasTolat2("park2", &SkillBaseState::parkolasTolat1, 10, 0, SKILLSLOW);

