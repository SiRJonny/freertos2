/*
 * ProcessSensors.h
 *
 *  Created on: 2015. okt. 24.
 *      Author: Csabi
 */

#ifndef INCLUDE_PROCESSSENSORS_H_
#define INCLUDE_PROCESSSENSORS_H_


#include "stm32f4xx_hal.h"
#include <cmath>

float getLinePos();
float calculateAngle(float pos1, float pos2);
float calculateAngle(void);
int calculateAverage(uint32_t * data, int datacount = 32);
void subtractFromAll(uint32_t * data, int amount, int datacount = 32);
int findMaxPos(uint32_t * data, int datacount = 32);
float refineMaxPos(uint32_t * data, int pos, int radius);


#endif /* INCLUDE_PROCESSSENSORS_H_ */
