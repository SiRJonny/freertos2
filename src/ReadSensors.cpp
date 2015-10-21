/*
 * ReadSensors.c
 *
 *  Created on: 2015. okt. 8.
 *      Author: Csabi
 */

#include "ReadSensors.h"

void ReadSensors()
{

}

void EnableDrivers()
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_RESET); // BLANK negált jel
}
void DisableDrivers()
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_SET);
}
void SetLeds(uint16_t pattern)
{

}

// nincs 1bit küldés SPI-on, ez lehet kimarad
void ShiftLeds(uint8_t amount)
{

}

void EnableMUX()
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET); // negÃ¡lt az EN jel
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

















