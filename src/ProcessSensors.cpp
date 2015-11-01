/*
 * ProcessSensors.c
 *
 *  Created on: 2015. okt. 24.
 *      Author: Csabi
 */

#include "ProcessSensors.h"

#define REFINE_RADIUS 2
#define NO_LINE_MULTIPLIER 2

extern TIM_HandleTypeDef htim5;
extern char buffer[10];	//bt msg hez
extern int timer; // id�m�r�shez
extern uint32_t szenzorsor_1[32];
extern uint32_t szenzorsor_2[32];

float refined_max;
float refined_max2;
struct LineState Lines; // TODO: angle nem, m�sik sor pos igen


// vonal poz�ci�, sz�m, sz�g sz�m�t�sa, treshold = h�ny %-al kisebb cs�cs �rv�nyes m�g
struct LineState getLinePos(int treshold)
{
	__HAL_TIM_SET_COUNTER(&htim5,0);

	int peaks1[3];
	int peaks2[3];
	//int average = calculateAverage(szenzorsor_1,32);
	subtractAllFrom(szenzorsor_1, 255);	// a kicsi �rt�k jelenti a vonalat, konvert�ljuk
	subtractAllFrom(szenzorsor_2, 255);

	Lines.numLines1 = find3peaks(szenzorsor_1, peaks1, treshold);
	Lines.numLines2 = find3peaks(szenzorsor_2, peaks2, treshold);

	for(int i = 0; i < Lines.numLines1; i++)
	{
		Lines.pos1[i] = refineMaxPos(szenzorsor_1,peaks1[i],REFINE_RADIUS);
	}
	for(int i = 0; i < Lines.numLines2; i++)
	{
		Lines.pos2[i] = refineMaxPos(szenzorsor_2,peaks2[i],REFINE_RADIUS);
	}

	timer = __HAL_TIM_GET_COUNTER(&htim5);
	BT_send_msg(&timer, "lines:" + std::string(itoa(timer,buffer,10)) + "\n");

	return Lines;
}

float calculateAngle()
{
	float angle = atan( ((refined_max-refined_max2)*5.917) / 56.725 );
	return angle;
}

float calculateAngle(float pos1, float pos2)
{
	float angle = atan( ((pos1-pos2)*5.917) / 56.725 );
	return angle;
}

// �tlag sz�m�t�s
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

// uint32_t t�mb minden elem�b�l ugyanazt kivonni,   < 0-ra szatur�l
void subtractFromAll(uint32_t * data, int amount, int datacount)
{
	for (int i = 0; i < datacount; i++)
	{
		if(data[i] < amount)
		{
			data[i] = 0;
		}else{
			data[i] -= amount;
		}
	}
}

// uint32_t t�mb minden elem�t ugyanabb�l kivonni
void subtractAllFrom(uint32_t * data, int amount, int datacount)
{
	for (int i = 0; i < datacount; i++)
	{
		data[i] = amount - data[i];
	}
}

// uint32_t t�mb-ben a legnagyobb �rt�k megkeres�se
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

// uint32_t t�mb-ben a legkisebb �rt�k megkeres�se
int findMinPos(uint32_t * data, int datacount)
{
	int min = 10000;	// adc max 12bit lehet, -> 4095
	int min_pos = 0;

	for(int i = 0; i < datacount; i++)
	{
		if(data[i] < min)
		{
			min = data[i];
			min_pos = i;
		}
	}
	return min_pos;
}

// 3 legnagyobb cs�cs megkeres�se, return numPeaks
int find3peaks(uint32_t * data, int * peaks, int treshold)
{
	int peakValue[3];
	int peakMinPos = 0;
	int peakMaxValue = 0;
	int numPeaks = 3;

	for(int i = 0; i<3; i++)
	{
		peakValue[i] = 0; // init
	}

	for(int i = 1; i < 31; i++)	// els�t �s utols�t nem n�zz�k
	{
		peakMinPos = findPeakMinPos(peakValue);		// a megl�v� cs�csok k�z�tt a legkisebb megkeres�se
		if(data[i-1] <= data[i] && data[i+1] <= data[i] && data[i] > peakValue[peakMinPos])
		{
			peaks[peakMinPos] = i;
			peakValue[peakMinPos] = data[i];
			if (data[i] > peakMaxValue)
			{
				peakMaxValue = data[i];
			}
		}
	}

	treshold = (float)peakMaxValue/100.0*(100-treshold); // treshold %-b�l �rt�k

	// treshold alattiak poz�ci�j�t 35-re, sorba rendez�sn�l el�l lesznek a t�nylegesek
	for(int i = 0; i < 3; i++)
	{
		if(peakValue[i] < treshold)
		{
			peaks[i] = 35;
			numPeaks--;
		}
	}

	// sorba rendez�s poz�ci� szerint
	if(peaks[0] > peaks[1])
	{
		swap(&peaks[0], &peaks[1]);
		swap(&peakValue[0], &peakValue[1]);
	}

	if(peaks[0] > peaks[2])
	{
		swap(&peaks[0], &peaks[2]);
		swap(&peakValue[0], &peakValue[2]);
	}

	if(peaks[1] > peaks[2])
	{
		swap(&peaks[1], &peaks[2]);
		swap(&peakValue[1], &peakValue[2]);
	}

	if(peakMaxValue > calculateAverage(data)*NO_LINE_MULTIPLIER) // ha a legnagyobb cs�cs nincs az �tlag ennyiszerese -> nincs vonal
	{
		return numPeaks;
	}else{
		return -1;
	}
}

int findPeakMinPos(int * peakValue)
{
	if(peakValue[0] < peakValue[1] && peakValue[0] < peakValue[2])
	{
		return 0;
	}else if (peakValue[1] < peakValue[2])
	{
		return 1;
	}else
	{
		return 2;
	}
}

void swap(int * a, int * b)
{
	int temp = *a;
	*a = *b;
	*b = temp;
}

// az adott pont k�r�liekb�l
float refineMaxPos(uint32_t * data, int pos, int radius)
{
	int base = pos - radius;
	float weighed_sum = 0;
	int value_sum = 0;
	int sat_i;

	// radius-nyival balra kezdve, radiusnyival a jobboldalra is elmenve
	// s�lyozott �tlagot sz�molunk a pontos�tott poz�ci��rt
	for(int i = 0; i<(2*radius + 1); i++)
	{
		if(i<0)
		{
			sat_i = -i;		// szenzorsor alul/t�lindexel�s
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











