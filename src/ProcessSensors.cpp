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

float refined_max;
float refined_max2;
struct LineState Lines; // TODO: angle nem, másik sor pos igen


// vonal pozíció, szám, szög számítása, treshold = hány %-al kisebb csúcs érvényes még
struct LineState getLinePos(int treshold)
{
	int peaks1[3];
	//int average = calculateAverage(szenzorsor_1,32);
	subtractAllFrom(szenzorsor_1, 255);	// a kicsi érték jelenti a vonalat, konvertáljuk
	subtractAllFrom(szenzorsor_2, 255);

	int max = findMaxPos(szenzorsor_1, 32);
	refined_max = refineMaxPos(szenzorsor_1,max,REFINE_RADIUS);

	int max2 = findMaxPos(szenzorsor_2, 32);
	refined_max2 = refineMaxPos(szenzorsor_2,max2,REFINE_RADIUS);

	//float angle = calculateAngle(refined_max,refined_max2);

	Lines.numLines = find3peaks(szenzorsor_1, peaks1, treshold);


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

// uint32_t tömb minden elemébõl ugyanazt kivonni,   < 0-ra szaturál
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

// uint32_t tömb minden elemét ugyanabból kivonni
void subtractAllFrom(uint32_t * data, int amount, int datacount)
{
	for (int i = 0; i < datacount; i++)
	{
		data[i] = amount - data[i];
	}
}

// uint32_t tömb-ben a legnagyobb érték megkeresése
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

// uint32_t tömb-ben a legkisebb érték megkeresése
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

// 3 legnagyobb csúcs megkeresése, return numPeaks
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

	for(int i = 1; i < 31; i++)	// elsõt és utolsót nem nézzük
	{
		peakMinPos = findPeakMinPos(peakValue);		// a meglévõ csúcsok között a legkisebb megkeresése
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

	treshold = (float)peakMaxValue/100.0*(100-treshold);

	// treshold alattiak pozícióját 35-re, sorba rendezésnél elöl lesznek a ténylegesek
	for(int i = 0; i < 3; i++)
	{
		if(peakValue[i] < treshold)
		{
			peaks[i] = 35;
			numPeaks--;
		}
	}

	// sorba rendezés pozíció szerint
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

	return numPeaks;
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











