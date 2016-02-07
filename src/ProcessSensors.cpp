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
extern int timer; // idõméréshez
extern uint32_t szenzorsor_1[32];
extern uint32_t szenzorsor_2[32];
extern int Distance_sensors[5];

extern bool fal_jobb;
extern bool fal_bal;
extern bool bordas_jobb;
extern bool bordas_bal;

float refined_max;
float refined_max2;
LineState Lines; // TODO: angle nem, másik sor pos igen


// átlagtól való átlagos eltérés
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

// tömb elemeinek száma, amik a min-max között vannak
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

// van_fal: utolsó 5 mérésbõl legalább 4 40-80 között
// bordas_fal: utolsó 20 mérésbõl legalább 8 közeli és 8 távoli falrész, TRUE marad, amíg van fal
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

	// van-e fal (bármilyen)
	if(count_between_values(array_j,num_samples_average,40,100) >= 4)
	{
		fal_jobb = true;
		borda_cntr_j++;		// csak akkor nézzük a bordásságot, ha van fal
		if (jArray_counter < num_samples_borda) {
			jArray_counter++;
		}

	}else{
		fal_jobb = false;
		borda_cntr_j = 0;
	}
	if(count_between_values(array_b,num_samples_average,40,100) >= 4)
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

	// borda számlálókat nullázzuk ha tele (hosszú fal)
	if(borda_cntr_j > num_samples_borda-2){
		borda_cntr_j = 0;
	}
	if(borda_cntr_b > num_samples_borda-2){
		borda_cntr_b = 0;
	}


	borda_array_j[borda_cntr_j] = Distance_sensors[3];
	borda_array_b[borda_cntr_b] = Distance_sensors[2];

	//ha már 10 mintát vettünk falból
	if(jArray_counter > 9)
	{
		// borda_cntr elemig megnézzük az átlagtól való eltérést
		int avDiff = average_difference(borda_array_j,jArray_counter);
		//BT_send_msg(&avDiff, "speed");
		if(avDiff > 5)
		{
			bordas_jobb = true;
		}
	}else if (!fal_jobb) {
		bordas_jobb = false;	// amint eltûnik a fal, ez lesz
	}


	//ha már 10 mintát vettünk falból
		if(bArray_counter > 9)
		{
			// borda_cntr elemig megnézzük az átlagtól való eltérést
			if(average_difference(borda_array_b,bArray_counter) > 5)
			{
				bordas_bal = true;
			}
		}else if (!fal_bal) {
			bordas_bal = false;	// amint eltûnik a fal, ez lesz
		}
}



// vonal pozíció, szám, szög számítása, treshold = hány %-al kisebb csúcs érvényes még
LineState getLinePos(int treshold)
{
	//__HAL_TIM_SET_COUNTER(&htim5,0);

	int peaks1[3];
	int peaks2[3];
	//int average = calculateAverage(szenzorsor_1,32);
	//subtractAllFrom(szenzorsor_1, 255);	// a kicsi érték jelenti a vonalat, konvertáljuk
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

// átlag számítás
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

// átlag számítás
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
	static int average;

	for(int i = 0; i<3; i++)
	{
		peakValue[i] = 0; // init
	}

	for(int i = 1; i < 31; i++)	// elsõt és utolsót nem nézzük
	{
		peakMinPos = findPeakMinPos(peakValue);		// a meglévõ csúcsok között a legkisebb megkeresése
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

	treshold = (float)peakMaxValue/100.0*(100-treshold); // treshold %-ból érték

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


	average = calculateAverage(data);
	if (average > 130)
	{
		return -2;
	}

	if(peakMaxValue > average*NO_LINE_MULTIPLIER && peakMaxValue > 130) // ha a legnagyobb csúcs nincs az átlag ennyiszerese -> nincs vonal
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

// az adott pont körüliekbõl
float refineMaxPos(uint32_t * data, int pos, int radius)
{
	int base = pos - radius;
	float weighed_sum = 0;
	int value_sum = 0;
	int index;

	// radius-nyival balra kezdve, radiusnyival a jobboldalra is elmenve
	// súlyozott átlagot számolunk a pontosított pozícióért
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











