/*
 * ReadSensors.c
 *
 *  Created on: 2015. okt. 8.
 *      Author: Csabi
 */

#include "ReadSensors.h"

extern int ADC1_BUFFER[4];
extern int szenzorsor_1[32];
extern int szenzorsor_2[32];


void ReadSensors()
{
	uint16_t pattern = 0x0101;

	EnableMUX();

	for (int i = 0; i<8; i++)
	{


		SetLeds(pattern);	// ez felf�ggeszt�s legyen? vagy id�beoszt�st m�shogy
		LATCHLeds();
		pattern <<= 1;

		// TODO: v�rni a szenzorra
		SetMUX((uint8_t)i);
		//ADC1_read();

		// 16on bel�l az els�
		szenzorsor_1[i] = ADC1_BUFFER[1];		// PA2 -> bal els� csoport
		szenzorsor_1[i+16] = ADC1_BUFFER[0];	// PA1 -> jobb els� csoport
		szenzorsor_2[i] = ADC1_BUFFER[3];		// PA4 -> bal h�ts�
		szenzorsor_2[i+16] = ADC1_BUFFER[2];	// bal h�ts�

		SetMUX((uint8_t)i+8);

		// 16on bel�l a m�sik
		szenzorsor_1[i+8] = ADC1_BUFFER[1];		// PA2 -> bal els� csoport
		szenzorsor_1[i+16+8] = ADC1_BUFFER[0];	// PA1 -> jobb els� csoport
		szenzorsor_2[i+8] = ADC1_BUFFER[3];		// PA4 -> bal h�ts�
		szenzorsor_2[i+16+8] = ADC1_BUFFER[2];	// bal h�ts�

	}

	DisableMUX();

}

void EnableDrivers()
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_RESET); // BLANK neg�lt jel
}
void DisableDrivers()
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_SET);
}
void SetLeds(uint16_t pattern)
{
	//TODO: set leds
	// send SPI
	// LATCH
}

// nincs 1bit k�ld�s SPI-on, ez lehet kimarad
void ShiftLeds(uint8_t amount)
{

}

void LATCHLeds()
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,GPIO_PIN_SET);
	// TODO: ez mennyi id� lesz?
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,GPIO_PIN_RESET);
}

void EnableMUX()
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET); // negált az EN jel
}
void DisableMUX()
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
}
void SetMUX(uint8_t ch)
{
	if (ch & (uint8_t)1){
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,GPIO_PIN_RESET);
	}
	if (ch & (uint8_t)2){
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_RESET);
	}
	if (ch & (uint8_t)4){
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
	}
	if (ch & (uint8_t)8){
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_RESET);
	}
	
}


uint16_t ReadADC()
{
	return 1;
}

















