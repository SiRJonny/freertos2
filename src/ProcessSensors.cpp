/*
 * ProcessSensors.c
 *
 *  Created on: 2015. okt. 24.
 *      Author: Csabi
 */

#include "ProcessSensors.h"

#define REFINE_RADIUS 2

extern uint32_t szenzorsor_1[32];
extern uint32_t szenzorsor_2[32];


float getLinePos()
{
	int average = calculateAverage(szenzorsor_1,32);
	int max = findMaxPos(szenzorsor_1, 32);
	float refined_max = refineMaxPos(szenzorsor_1,max,REFINE_RADIUS);

	return refined_max;
}



int calculateAverage(uint32_t * data, int datacount)
{
	int average = 0;
	for(int i = 0; i < datacount; i++)
	{
		average += data[i];
	}
	average /= datacount;
	return average;
}


void subtractFromAll(uint32_t * data, int amount, int datacount)
{
	for (int i = 0; i < datacount; i++)
	{
		if(data[i] < datacount)
		{
			data[i] = 0;
		}else{
			data[i] -= amount;
		}
	}
}

int findMaxPos(uint32_t * data, int datacount)
{
	int max = 0;
	int max_pos = 0;

	for(int i = 0; i < datacount; i++)
	{
		if(data[i] > max)
		{
			max = data[i];
			max_pos = i;
		}
	}
	return max_pos;
}

// az adott pont körüliekbõl
float refineMaxPos(uint32_t * data, int pos, int radius)
{
	int base = pos - radius;
	float weighed_sum = 0;
	int value_sum = 0;
	int sat_i;

	// radius-nyival balra kezdve, radiusnyival a jobboldalra is elmenve
	// súlyozott átlagot számolunk a pontosított pozícióért
	for(int i = 0; i<(2*radius + 1); i++)
	{
		if(i<0)
		{
			sat_i = -i;		// szenzorsor alul/túlindexelés
		}else if(i > 32){
			sat_i = i-32;
		}else{
			sat_i = i;
		}
		weighed_sum += data[base+sat_i]*(base+sat_i);
		value_sum += data[base+sat_i];
	}
	weighed_sum /= value_sum;

	return weighed_sum;
}











