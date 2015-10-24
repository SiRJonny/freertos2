/*
 * ReadSensors.c
 *
 *  Created on: 2015. okt. 8.
 *      Author: Csabi
 */

#include "ReadSensors.h"
#include "cmsis_os.h"

extern int ADC1_BUFFER[4];
extern int szenzorsor_1[32];
extern int szenzorsor_2[32];

extern ADC_HandleTypeDef hadc1;
extern osSemaphoreId ADC1_complete;
extern SPI_HandleTypeDef hspi3;
HAL_StatusTypeDef status;

// teljes szenzorsor beolvasás
void ReadSensors()
{
	uint16_t pattern = 0x0101;

	EnableMUX();

	for (int i = 0; i<8; i++)
	{


		SetLeds(pattern);	// ez felfüggesztõs legyen? vagy idõbeosztást máshogy
		LATCHLeds();
		pattern <<= 1;

		// TODO: várni a szenzorra
		SetMUX((uint8_t)i);
		ADC1_read();

		// 16on belül az elsõ
		szenzorsor_1[i] = ADC1_BUFFER[1];		// PA2 -> bal elsõ csoport
		szenzorsor_1[i+16] = ADC1_BUFFER[0];	// PA1 -> jobb elsõ csoport
		szenzorsor_2[i] = ADC1_BUFFER[3];		// PA4 -> bal hátsó
		szenzorsor_2[i+16] = ADC1_BUFFER[2];	// bal hátsó

		SetMUX((uint8_t)i+8);

		// 16on belül a másik
		szenzorsor_1[i+8] = ADC1_BUFFER[1];		// PA2 -> bal elsõ csoport
		szenzorsor_1[i+16+8] = ADC1_BUFFER[0];	// PA1 -> jobb elsõ csoport
		szenzorsor_2[i+8] = ADC1_BUFFER[3];		// PA4 -> bal hátsó
		szenzorsor_2[i+16+8] = ADC1_BUFFER[2];	// bal hátsó

	}

	DisableMUX();

}


// szenzorsor 4 jelének beolvasása
// TODO: lehet, hogy nem is kell visszaadni, ott az ADC1_BUFFER?
// TODO: idõmérés?
void ADC1_read()
{
	HAL_ADC_Start_DMA(&hadc1, ADC1_BUFFER, 4);
	osSemaphoreWait(ADC1_complete,osWaitForever);
	HAL_ADC_Stop_DMA(&hadc1);
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
	status = HAL_SPI_Transmit(&hspi3,(uint8_t*)&pattern,1,100);
	if(status != HAL_OK)
	{
		while(1); // TODO: ez csak debughoz
	}
}

// nincs 1bit küldés SPI-on, ez lehet kimarad
void ShiftLeds(uint8_t amount)
{

}

void LATCHLeds()
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,GPIO_PIN_SET);
	// TODO: ez mennyi idõ lesz?
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,GPIO_PIN_RESET);
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
















