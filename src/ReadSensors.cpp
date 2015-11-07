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
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
HAL_StatusTypeDef status;


extern void BT_send_msg(int*msg,std::string nev);



// teljes szenzorsor beolvas�s
void ReadSensors()
{
	uint16_t pattern = 0x0101;


	EnableMUX();

	for (int i = 0; i<8; i++)
	{


		//__HAL_TIM_SET_COUNTER(&htim5,0);



		SetLeds(0x0000);
		SetLeds(pattern);	// ez felf�ggeszt�s legyen? vagy id�beoszt�st m�shogy
		LATCHLeds();
		pattern <<= 1;

		SetMUX((uint8_t)i);

		//ControlTaskDelay(200);	// szenzor felfut�sra v�rakoz�s
		osDelay(10);



		// TODO: v�rni muxra? 1us-t csak blokkolva lehet
		//ControlTaskDelay(20);

		ADC1_read();

		// 16on bel�l az els�
		szenzorsor_1[i] = ADC1_BUFFER[1];		// PA2 -> bal els� csoport
		szenzorsor_1[i+16] = ADC1_BUFFER[0];	// PA1 -> jobb els� csoport
		szenzorsor_2[i] = ADC1_BUFFER[3];		// PA4 -> bal h�ts�
		szenzorsor_2[i+16] = ADC1_BUFFER[2];	// bal h�ts�

		SetMUX((uint8_t)i+8);
		//ControlTaskDelay(20);
		osDelay(10);

		ADC1_read();

		// 16on bel�l a m�sik
		szenzorsor_1[i+8] = ADC1_BUFFER[1];		// PA2 -> bal els� csoport
		szenzorsor_1[i+16+8] = ADC1_BUFFER[0];	// PA1 -> jobb els� csoport
		szenzorsor_2[i+8] = ADC1_BUFFER[3];		// PA4 -> bal h�ts�
		szenzorsor_2[i+16+8] = ADC1_BUFFER[2];	// bal h�ts�

		//timer = __HAL_TIM_GET_COUNTER(&htim5);
		//BT_send_msg(&timer, "1/8sens" + std::string(itoa(timer,buffer,10)) + "\n");


	}



	HAL_TIM_Base_Stop_IT(&htim6);
	DisableMUX();

}


// szenzorsor 4 jel�nek beolvas�sa
void ADC1_read()
{
	HAL_ADC_Start_DMA(&hadc1, ADC1_BUFFER, 4);

	osSemaphoreWait(ADC1_complete,osWaitForever);
	HAL_ADC_Stop_DMA(&hadc1);

	/*HAL_ADC_Start(&hadc1);
	for(int i = 0; i < 4; i++)
	{
		HAL_ADC_PollForConversion(&hadc1,1000);
		ADC1_BUFFER[i] = HAL_ADC_GetValue(&hadc1);

	}
	HAL_ADC_Stop(&hadc1);*/
}

// control task delay microsec (min 20)
void ControlTaskDelay(int us)
{
	HAL_TIM_Base_Start_IT(&htim6);
	__HAL_TIM_SET_COUNTER(&htim6,60000-us+11); // tim6 nem tud lefele sz�molni?? // +11 a taskv�lt�s miatt (n�ha 1us-el hosszabbat v�r �gy)
	osSignalWait(0x0001,osWaitForever);
	HAL_TIM_Base_Stop_IT(&htim6);
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
	status = HAL_SPI_Transmit(&hspi3,(uint8_t*)&pattern,1,100);
	if(status != HAL_OK)
	{
		while(1); // TODO: ez csak debughoz
	}
}

// nincs 1bit k�ld�s SPI-on, ez lehet kimarad
void ShiftLeds(uint8_t amount)
{

}

void LATCHLeds()
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,GPIO_PIN_SET);
	// TODO: ez mennyi id� lesz?
	ControlTaskDelay(40);
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
















