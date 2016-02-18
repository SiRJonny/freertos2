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
#include <string>
#include <stdlib.h>
#include "EnumsStructs.hpp"

typedef struct
{
	int numLines1, numLines2;		// vonalak száma
	float pos1[3];		// vonalak pozíciója
	float pos2[3];
}LineState;

extern void BT_send_msg(int*msg,std::string nev);

int average_difference(int * array, int N);
float median_filter(float val);
int compare (const void * a, const void * b);
int count_between_values(int * array, int N, int min, int max);
void wall_detection();
void wall_borda_detection();
LineState getLinePos(int treshold);
float calculateAngle(float pos1, float pos2);
float calculateAngle(void);
int calculateAverage(uint32_t * data, int datacount = 32);
float calculateMovingAverage(float data); // 5
void subtractFromAll(uint32_t * data, int amount, int datacount = 32);
void subtractAllFrom(uint32_t * data, int amount, int datacount = 32);
int findMaxPos(uint32_t * data, int datacount = 32);
int findMinPos(uint32_t * data, int datacount = 32);
int find3peaks(uint32_t * data, int * peaks, int treshold);
int findPeakMinPos(int * peakValue);
void swap(int * a, int * b);
float refineMaxPos(uint32_t * data, int pos, int radius);


#endif /* INCLUDE_PROCESSSENSORS_H_ */
