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
extern int Distance_sensors[5];

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern osSemaphoreId ADC1_complete;
extern SPI_HandleTypeDef hspi3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
HAL_StatusTypeDef status;


extern void BT_send_msg(int*msg,std::string nev);
extern void SetServo_sensor_jobb();
extern void SetServo_sensor_bal();
extern void SetServo_sensor(int pos);
extern int timer;
extern char buffer[10];

extern int testLinePos;




//elsõ távolságszenzor
int ReadFrontLeft()
{
	SetServo_sensor_bal();
	osDelay(300);
	ADC2_read();
	return (int)Distance_sensors[1];
}

int ReadFrontRight()
{
	SetServo_sensor_jobb();
	osDelay(300);
	ADC2_read();
	return (int)Distance_sensors[1];
}

int ReadFrontMiddle()
{
	SetServo_sensor(0);
	osDelay(300);
	ADC2_read();
	return (int)Distance_sensors[1];
}

// teljes szenzorsor beolvasás
void ReadSensorsDummy()
{



	for (int i = 0; i < 32; i++) {

		if (i == testLinePos) {
			szenzorsor_1[i] = 50;
			szenzorsor_2[i] = 50;
		} else if(i == testLinePos-1){
			szenzorsor_1[i] = 50;
			szenzorsor_2[i] = 50;
		} else {
			szenzorsor_1[i] = 251;
			szenzorsor_2[i] = 251;
		}


	}

}

// teljes szenzorsor beolvasás
void ReadSensors()
{
	uint16_t pattern = 0x0101;
	int amount = 255;

	EnableMUX();

	// TODO: elsõ szenzor kiugró érték ellen, de nem biztos, hogy jó így
	SetLeds(0x0000);
	SetLeds(pattern);
	LATCHLeds();
	ADC1_read();
	ControlTaskDelay(200);

	for (int i = 0; i<8; i++)
	{






		SetLeds(0x0000);
		SetLeds(pattern);	// ez felfüggesztõs legyen? vagy idõbeosztást máshogy
		LATCHLeds();



		pattern <<= 1;

		SetMUX((uint8_t)i);

		ControlTaskDelay(40);	// szenzor felfutásra várakozás
		//osDelay(10);



		// TODO: várni muxra? 1us-t csak blokkolva lehet
		//ControlTaskDelay(20);
		//__HAL_TIM_SET_COUNTER(&htim5,0);

		ADC1_read();

		/*timer = __HAL_TIM_GET_COUNTER(&htim5);
		if(i==3){
			BT_send_msg(&timer, "setL:" + std::string(itoa(timer,buffer,10)) + "\n");
		}*/

		// 16on belül az elsõ
		szenzorsor_1[i] = amount - ADC1_BUFFER[1];		// PA2 -> bal elsõ csoport
		szenzorsor_1[i+16] = amount - ADC1_BUFFER[0];	// PA1 -> jobb elsõ csoport
		szenzorsor_2[i] = amount - ADC1_BUFFER[3];		// PA4 -> bal hátsó
		szenzorsor_2[i+16] = amount - ADC1_BUFFER[2];	// jobb hátsó

		SetMUX((uint8_t)i+8);
		ControlTaskDelay(40);
		//osDelay(10);

		ADC1_read();

		// 16on belül a másik
		szenzorsor_1[i+8] = amount - ADC1_BUFFER[1];		// PA2 -> bal elsõ csoport
		szenzorsor_1[i+16+8] = amount - ADC1_BUFFER[0];	// PA1 -> jobb elsõ csoport
		szenzorsor_2[i+8] = amount - ADC1_BUFFER[3];		// PA4 -> bal hátsó
		szenzorsor_2[i+16+8] = amount - ADC1_BUFFER[2];	// jobb hátsó

		//osDelay(500);



	}

	SetLeds(0x0000);
	LATCHLeds();

	HAL_TIM_Base_Stop_IT(&htim6);
	DisableMUX();

	szenzorsor_1[7] = ((szenzorsor_1[6] + szenzorsor_1[8])/2.0f)*1.5f;
}


// szenzorsor 4 jelének beolvasása
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

void ADC2_read()
{
	HAL_ADC_Start(&hadc2);
	for(int i = 0; i<5; i++)
	{
		HAL_ADC_PollForConversion(&hadc2,1000);
		Distance_sensors[i] = HAL_ADC_GetValue(&hadc2);
	}
	HAL_ADC_Stop(&hadc2);
}

// control task delay microsec (min 20)
void ControlTaskDelay(int us)
{
	HAL_TIM_Base_Start_IT(&htim6);
	__HAL_TIM_SET_COUNTER(&htim6,60000-us+11); // tim6 nem tud lefele számolni?? // +11 a taskváltás miatt (néha 1us-el hosszabbat vár így)
	osSignalWait(0x0001,osWaitForever);
	HAL_TIM_Base_Stop_IT(&htim6);
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
	//ControlTaskDelay(40);
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
















