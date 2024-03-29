/*
 * ProcessSensors.c
 *
 *  Created on: 2015. okt. 24.
 *      Author: Csabi
 */

#include "ProcessSensors.h"
#include <stdlib.h>

#define REFINE_RADIUS 2
#define NO_LINE_MULTIPLIER 2

extern TIM_HandleTypeDef htim5;
extern char buffer[10];	//bt msg hez
extern int timer; // id�m�r�shez
extern uint32_t szenzorsor_1[32];
extern uint32_t szenzorsor_2[32];
extern int Distance_sensors[5];

extern bool fal_jobb;
extern bool fal_bal;
extern bool bordas_jobb;
extern bool bordas_bal;

float refined_max;
float refined_max2;
LineState Lines; // TODO: angle nem, m�sik sor pos igen


// �tlagt�l val� �tlagos elt�r�s
int average_difference(int * array, int N)
{
	int ret = 0;
	int average = calculateAverage(array, N);
	for(int i=0; i<N; i++)
	{
		ret = ret + abs(average-array[i]);
	}
	return ret/N;
}

// t�mb elemeinek sz�ma, amik a min-max k�z�tt vannak
float median_filter(float val)
{
	static float array[5];
	static int index = 0;

	array[index] = val;
	index++;
	if(index >=5)
	{
		index = 0;
	}

	qsort(array,5,4,compare);

	return array[2];
}

int compare (const void * a, const void * b)
{
  float fa = *(const float*) a;
  float fb = *(const float*) b;
  return (fa > fb) - (fa < fb);
}

int count_between_values(int * array, int N, int min, int max)
{
	int count = 0;
	for(int i=0; i<N; i++)
	{
		if( array[i] > min && array[i] < max )
		{
			count++;
		}
	}
	return count;
}

// van_fal: utols� 5 m�r�sb�l legal�bb 4 40-80 k�z�tt
// bordas_fal: utols� 20 m�r�sb�l legal�bb 8 k�zeli �s 8 t�voli falr�sz, TRUE marad, am�g van fal
void wall_detection()
{
	static int num_samples_average = 5;
	static int num_samples_borda = 20;
	static int array_j[5];
	static int array_b[5];
	static int borda_array_j[20];
	static int borda_array_b[20];
	static int average_cntr = 0;
	static int borda_cntr_j = 0;
	static int borda_cntr_b = 0;
	static int jArray_counter = 0;
	static int bArray_counter = 0;

	array_j[average_cntr] = Distance_sensors[3];
	array_b[average_cntr] = Distance_sensors[2];

	if(average_cntr > num_samples_average-2){
		average_cntr = 0;
	}else{
		average_cntr++;
	}

	// van-e fal (b�rmilyen)
	if(count_between_values(array_j,num_samples_average,40,250) >= 4)
	{
		fal_jobb = true;
		borda_cntr_j++;		// csak akkor n�zz�k a bord�ss�got, ha van fal
		if (jArray_counter < num_samples_borda) {
			jArray_counter++;
		}

	}else{
		fal_jobb = false;
		borda_cntr_j = 0;
	}
	if(count_between_values(array_b,num_samples_average,40,250) >= 4)
	{
		fal_bal = true;
		borda_cntr_b++;
		if (bArray_counter < num_samples_borda) {
			bArray_counter++;
		}
	}else{
		fal_bal = false;
		borda_cntr_b = 0;
	}

	// borda sz�ml�l�kat null�zzuk ha tele (hossz� fal)
	if(borda_cntr_j > num_samples_borda-2){
		borda_cntr_j = 0;
	}
	if(borda_cntr_b > num_samples_borda-2){
		borda_cntr_b = 0;
	}


	borda_array_j[borda_cntr_j] = Distance_sensors[3];
	borda_array_b[borda_cntr_b] = Distance_sensors[2];

	//ha m�r 10 mint�t vett�nk falb�l
	if(jArray_counter > 9)
	{
		// borda_cntr elemig megn�zz�k az �tlagt�l val� elt�r�st
		int avDiff = average_difference(borda_array_j,jArray_counter);
		//BT_send_msg(&avDiff, "speed");
		if(avDiff > 5)
		{
			bordas_jobb = true;
		}
	}
	if (!fal_jobb) {
		bordas_jobb = false;	// amint elt�nik a fal, ez lesz
	}


	//ha m�r 10 mint�t vett�nk falb�l
	if(bArray_counter > 9)
	{
		// borda_cntr elemig megn�zz�k az �tlagt�l val� elt�r�st
		if(average_difference(borda_array_b,bArray_counter) > 5)
		{
			bordas_bal = true;
		}
	}
	if (!fal_bal) {
		bordas_bal = false;	// amint elt�nik a fal, ez lesz
	}
}

extern int globalDistance;
void wall_borda_detection() {
	static int index = 0;

	static int prevPos = globalDistance;
	static int borda_array_j[20];
	static int borda_array_b[20];
	static bool checking = true;
	static int arraySize = 20;

	int distanceRight = 0;
	int distanceLeft = 0;

	if ((fal_jobb || fal_bal) && checking) {

		int difference = globalDistance - prevPos;
		int diffTreshold = 10;

		if (difference > diffTreshold ){
			distanceRight = Distance_sensors[3];
			distanceLeft = Distance_sensors[2];
			if (distanceRight < 200 && distanceLeft < 200) {
				borda_array_j[index] = distanceRight;
				borda_array_b[index] = distanceLeft;
				prevPos = globalDistance;
				index++;
			}
		}


	} else if (!(fal_jobb || fal_bal)){
		index = 0;
		checking = true;
		//direction = UNDEFINED;
	}

	if (index == arraySize) {
		//osThreadSuspend(SendRemoteVar_TaskHandle);
		//int dirInt = -5;
		int bordaTresholdMin = 5;
		int bordaTresholdMax = 30;

		int aveDiffRight = average_difference(borda_array_j, arraySize);
		int aveDiffLeft = average_difference(borda_array_b, arraySize);

		if ( aveDiffRight > bordaTresholdMin && aveDiffRight < bordaTresholdMax) {
			direction = LEFT;
		} else if ( aveDiffLeft > bordaTresholdMin && aveDiffLeft < bordaTresholdMax) {
			direction = RIGHT;
		} else {
			direction = UNDEFINED;
		}

		/*
		for (int i = 1; i < 20; i++) {

			int differentJobb = borda_array_j[i] - borda_array_j[i-1];
			int differentBal = borda_array_b[i] - borda_array_b[i-1];

			if ( differentJobb > bordaTresholdMin && differentJobb < bordaTresholdMax) {
				direction = LEFT;
			} else if ( differentBal > 8 && differentBal < bordaTresholdMax) {
				direction = RIGHT;
			} else {
				direction = UNDEFINED;
			}
			//BT_send_msg(&speed_global, "speed");
			//BT_send_msg(&borda_array_b[i], "contBordArr");
		}
		//dirInt = direction;
*/
		//BT_send_msg(&dirInt, "dirInt");

		checking = false;
	}

}



// vonal poz�ci�, sz�m, sz�g sz�m�t�sa, treshold = h�ny %-al kisebb cs�cs �rv�nyes m�g
LineState getLinePos(int treshold)
{
	//__HAL_TIM_SET_COUNTER(&htim5,0);

	int peaks1[3];
	int peaks2[3];
	//int average = calculateAverage(szenzorsor_1,32);
	//subtractAllFrom(szenzorsor_1, 255);	// a kicsi �rt�k jelenti a vonalat, konvert�ljuk
	//subtractAllFrom(szenzorsor_2, 255);

	Lines.numLines1 = find3peaks(szenzorsor_1, peaks1, treshold);
	Lines.numLines2 = find3peaks(szenzorsor_2, peaks2, treshold);

	/*if(peaks1[0] > 32 || peaks1[0] < 0)
	{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);

	}
	if(peaks2[0] > 32 || peaks2[0] < 0)
	{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);

	}*/


	for(int i = 0; i < Lines.numLines1; i++)
	{
		Lines.pos1[i] = refineMaxPos(szenzorsor_1,peaks1[i],REFINE_RADIUS);
	}
	for(int i = 0; i < Lines.numLines2; i++)
	{
		Lines.pos2[i] = refineMaxPos(szenzorsor_2,peaks2[i],REFINE_RADIUS);
	}



	//timer = __HAL_TIM_GET_COUNTER(&htim5);
	//BT_send_msg(&timer, "lines:" + std::string(itoa(timer,buffer,10)) + "\n");

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

// �tlag sz�m�t�s
int calculateAverage(int * data, int datacount)
{
	int average = 0;
	for(int i = 0; i < datacount; i++)
	{
		average += data[i];
	}
	average /= datacount;
	return average;
}
float calculateMovingAverage(float data)
{
	static float average = 0;
	static float array[10];
	static int index = 0;
	array[index] = data;
	index++;
	if(index >= 10)
	{
		index = 0;
	}

	average = 0;
	for(int i = 0; i < 10; i++)
	{
		average += array[i];
	}
	average /= 10.0f;
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
	static int average;

	for(int i = 0; i<3; i++)
	{
		peakValue[i] = 0; // init
	}

	for(int i = 1; i < 31; i++)	// els�t �s utols�t nem n�zz�k
	{
		peakMinPos = findPeakMinPos(peakValue);		// a megl�v� cs�csok k�z�tt a legkisebb megkeres�se
		if(data[i-1] <= data[i] && data[i+1] < data[i] && data[i] > peakValue[peakMinPos])
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
		if(peakValue[i] < treshold && peakValue[i] < 130)
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

	//keresztvonal
	average = calculateAverage(data);
	if (average > 130)
	{
		return -2;
	}

	if(peakMaxValue > average*NO_LINE_MULTIPLIER && peakMaxValue > 130) // ha a legnagyobb cs�cs nincs az �tlag ennyiszerese -> nincs vonal
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
	int index;

	// radius-nyival balra kezdve, radiusnyival a jobboldalra is elmenve
	// s�lyozott �tlagot sz�molunk a pontos�tott poz�ci��rt
	for(int i = 0; i<(2*radius + 1); i++)
	{
		index = base + i;
		if(index < 0){
			index = 1;
		}else if(index > 31){
			index = 30;
		}

		weighed_sum += data[index]*(index);
		value_sum += data[index];
	}
	weighed_sum /= value_sum;

	return weighed_sum;
}











