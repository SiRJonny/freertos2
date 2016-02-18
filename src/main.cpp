
/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/

extern "C"
{
#include <stm32f4xx_hal.h>
#include "cmsis_os.h"
}

#include <string>
#include <stdlib.h>

#include "GPIO_setup.h"
//#include "stm32f4xx_hal_uart.h"
#include "BT_MSG.h"
#include "ReadSensors.h"
#include "ProcessSensors.h"
#include "Controllers.h"
#include "StateMachine.h"
#include "StatePattern.hpp"
#include <StatePatternSkill.hpp>
#include <config.hpp>
#include "giro.hpp"
using namespace std;

int stAngle = 0;




/* Private variables ---------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId BT_TaskHandle;
osThreadId SendRemoteVar_TaskHandle;
osThreadId SteerControl_TaskHandle;
osThreadId BT_Receive_TaskHandle;
osThreadId Turn_On_TaskHandle;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

osSemaphoreId xSem_USART_rdy_to_send;
osSemaphoreDef(xSem_USART_rdy_to_send);

osSemaphoreId ADC1_complete;
osSemaphoreDef(ADC1_complete);

QueueHandle_t xQueue_BT;


float distance_error = 0;
float fr_distance = 0;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void StartDefaultTask();
void StartButtonTask();
void SendBluetoothTask();
void SendRemoteVarTask();
void SteerControlTask();
void BTReceiveTask();
void TurnOnTask();
bool START_PIN();
void SetSkillTrack();
bool get_switch();

void USART3_UART_Init();
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART1_UART_Init(void);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);


/*void BT_send_msg(int msg);
void BT_send_msg(int msg, char* nev);*/
void BT_send_msg(int * msg, string nev);
//void BT_send_msg(float * msg, string nev);
void SetServo_motor(int pos); // -SERVO_RANGE_MOTOR +SERVO_RANGE_MOTOR
void SetServo_sensor(int pos);
void SetServo_steering(int pos); // -SERVO_RANGE_STEERING +SERVO_RANGE_STEERING
void SetServo_steering(float angle);  // kormányzás, szöggel
void getActiveLinePos(LineState * Lines, float *last_pos1, float *last_pos2, float * active1, float * active2);
void is_speed_under_X(float speed, float limit);
void get_stable_line_count(int numlines);
void get_stable_line_count2(int numlines1, int numlines2);

void update_direction();
bool DIP(int num);
bool START_PIN();
uint8_t Radio_get_char();
void SetServo_sensor_jobb();
void SetServo_sensor_bal();
int systick_count();

void sendSensors();
void sendDebugVars();
void sendStateData();
void sendPIDs();

int main(void)
{


  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */

  /* USER CODE BEGIN 2 */
  GPIO_Init();
  MX_DMA_Init();
  USART3_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();

  /* USER CODE END 2 */

  xSem_USART_rdy_to_send = osSemaphoreCreate(osSemaphore(xSem_USART_rdy_to_send),1);
  ADC1_complete = osSemaphoreCreate(osSemaphore(ADC1_complete),1);
  osSemaphoreWait(ADC1_complete,osWaitForever);

	// itt, hogy BT tasknak mÃ¡r kÃ©sz legyen
  xQueue_BT = xQueueCreate(10, sizeof(struct BT_MSG));

  SetSkillTrack();


  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */


  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  osThreadDef(ButtonTask, StartButtonTask, osPriorityBelowNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(ButtonTask), NULL);

  osThreadDef(SendBluetoothTask, SendBluetoothTask, osPriorityNormal, 0, 128);
  BT_TaskHandle = osThreadCreate(osThread(SendBluetoothTask), NULL);

  osThreadDef(SendRemoteVarTask, SendRemoteVarTask, osPriorityNormal, 0, 128);
  SendRemoteVar_TaskHandle = osThreadCreate(osThread(SendRemoteVarTask), NULL);

  osThreadDef(SteerControlTask, SteerControlTask, osPriorityHigh, 0, 192);
  SteerControl_TaskHandle = osThreadCreate(osThread(SteerControlTask), NULL);

  osThreadDef(BTReceiveTask, BTReceiveTask, osPriorityNormal, 0, 128);
  BT_Receive_TaskHandle = osThreadCreate(osThread(BTReceiveTask), NULL);

  //osThreadDef(TurnOnTask, TurnOnTask, osPriorityHigh, 0, 128);
  //Turn_On_TaskHandle = osThreadCreate(osThread(TurnOnTask), NULL);



  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */

  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}


void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void BT_send_msg(int * msg, string nev){
	struct BT_MSG msg_int;
	int2msg(&msg_int, msg, nev.c_str());
	xQueueSend( xQueue_BT, &msg_int, portMAX_DELAY);

}

void BT_send_msg(float * msg, string nev){
	struct BT_MSG msg_float;
	float2msg(&msg_float, msg, nev.c_str());
	xQueueSend( xQueue_BT, &msg_float, portMAX_DELAY);

}


// bal = pozitív
void SetServo_sensor(int pos)
{
	if(pos > SERVO_RANGE_SENSOR){pos = SERVO_RANGE_SENSOR;}
	if(pos < -SERVO_RANGE_SENSOR){pos = -SERVO_RANGE_SENSOR;}

	// 1500 = 1,5ms, ez a 0 pozíció
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,1500+pos);
}

// -500 és 500 közötti értéket fogad
void SetServo_motor(int pos)
{
	if(pos > SERVO_RANGE_MOTOR){pos = SERVO_RANGE_MOTOR;}
	if(pos < -SERVO_RANGE_MOTOR){pos = -SERVO_RANGE_MOTOR;}

	// 1500 = 1,5ms, ez a 0 pozíció
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1500-pos);
}

// TODO: sebességmérés külön, és rendes emergency break
void EmergencyBreak(int time)
{
	stateContext.stop();
	SET_SPEED = 0;
	if(time > 2000){time = 2000;}

	SetServo_motor(-100);
	osDelay(5);
	SetServo_motor(0);
	osDelay(5);
	SetServo_motor(-500);
	osDelay(time);
	SetServo_motor(0);
}

// -500 és 500 közötti értéket fogad DEFINEolva!
void SetServo_steering(int pos)
{
	if(pos > SERVO_RANGE_STEERING)
	{
		pos = SERVO_RANGE_STEERING;
		//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
	}
	else if(pos < -SERVO_RANGE_STEERING)
	{
		pos = -SERVO_RANGE_STEERING;
		//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
	}
	else
	{
		//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
	}
	// 1500 = 1,5ms, ez a 0 pozíció
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, 1500 - pos + servoOffset);
	//TODO offset
}

// float szög
void SetServo_steering(float angle_rad)
{


	float pos = (angle_rad / 6.28 * 360) * 10.66666;	// 10.6666 csinál fokból pwm-et
	int posInt = (int)pos;
	SetServo_steering(posInt);
	/*
	if(pos > SERVO_RANGE_STEERING)
	{
		pos = SERVO_RANGE_STEERING;
		//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
	}
	else if(pos < -SERVO_RANGE_STEERING)
	{
		pos = -SERVO_RANGE_STEERING;
		//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
	}
	else
	{
		//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
	}
	// 1500 = 1,5ms, ez a 0 pozíció
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, 1500 - (int)pos + servoOffset);*/

}




/* StartDefaultTask function */
// default tastk, csak egy villogÃ³ led
void StartDefaultTask()
{
	int i = 0;

	int encoderPosArray[5];
	int buffer_cntr = 0;
	float speed_temp = 0;
	float globalDistance_temp = 0;

	float encoderDifference = 0;
	float encoderDivider = 0;

	float distDiff = 0.0;
	float timeDiff = 0.05;

	for(int i = 0; i < 5; i++)
	{
		encoderPosArray[i] = 1000000000;
	}

	for(;;)
	{

		// sebesség mérés
		encoderPos = __HAL_TIM_GET_COUNTER(&htim2);

		globalDistance_temp = (float)(1000000000 - encoderPos)/2.35f;		// encoder leosztás még egy helyen szerepel!!!!

		encoderPosArray[buffer_cntr] = encoderPos;
		buffer_cntr++;
		if(buffer_cntr >= 5)
		{
			buffer_cntr = 0;
		}


		encoderDifference = encoderPosArray[buffer_cntr] - encoderPos;

		//distDiff = encoderDifference*encoderIncrementToMeter;
		//speed_temp = distDiff/timeDiff;

		encoderDivider = (2350.0f/20.0f);			// encoder leosztás még egy helyen szerepel!!!!
		speed_temp = (float)(encoderDifference/encoderDivider); //ez így m/s, ha 20 lukas a tárcsa és 50ms-enként mérünk (másodpercenként 20)

		osThreadSuspendAll();
		speed_global = speed_temp;
		globalDistance = globalDistance_temp;
		osThreadResumeAll();

		if (i == 50)
		{
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
			i = 0;
		}
		else
		{
			i++;
		}
		osDelay(10);
	}

}

void StartButtonTask()
{
	uint8_t wasPressed = 0;


	for (;;){

		// TODO: ez blokkol mindent?????
		while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)  == 1) {
			wasPressed = 1;
		}

		if (wasPressed){


			giro_init();
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);

			if(START_PIN() == false)	// ha rajta van a kábel
			{
				while(START_PIN() == false)
				{
					osDelay(50);
				}
				start_radio_done = false;
				stateContext.start(encoderPos);
				skillStateContext.setState(&SkillBaseState::skillStarted);
				stateData.event = UNSTABLE;
				osThreadResume(SteerControl_TaskHandle);
				osThreadResume(SendRemoteVar_TaskHandle);
			} else {
				stateContext.setState(&BaseState::started);
				skillStateContext.setState(&SkillBaseState::koztes);
				start_radio_done = true;
				stateData.event = UNSTABLE;
				osThreadResume(SteerControl_TaskHandle);
				osThreadResume(SendRemoteVar_TaskHandle);
			}


			wasPressed = 0;
		}
		osDelay(30);
	}
}

// bluetooth kÃ¼ldÅ‘ task, xQueue_BT-n keresztÃ¼l kapja a struct BT_MSG pointereket, abbÃ³l kÃ¼ldi az adatokat
// ha minden igaz csak akkor fut, ha tÃ©nyleg van dolga
void SendBluetoothTask()
{
	struct BT_MSG msg;

	for( ;; )
	{
		if(xQueue_BT != 0)
		{
			if (xQueueReceive(xQueue_BT, &msg, portMAX_DELAY))// blokk amÃ­g nincs adat
			{

				HAL_UART_Transmit_IT(&huart3, (uint8_t*) msg.data, msg.size);


				osSignalWait(0x0001,osWaitForever);

			}

		}
	}
}

// uart3 fogadása, BT üzenetek
void BTReceiveTask()
{
	uint8_t msg[10];
	int * int_ptr;
	float * flt_ptr;

	int sid = -2;

	int data;
	char * char_ptr = (char*)&data;

	int_ptr = (int*)&msg[1];
	flt_ptr = (float*)&msg[1];

	for( ;; )
	{


		HAL_UART_Receive_IT(&huart3,msg,(uint16_t)5);

		osSignalWait(0x0001,osWaitForever);

		// TODO: változó fogadásnál mi legyen?
		switch(msg[0])
		{
			case 0:
				//SET_SPEED = 0;
				//state_struct.state = -1;
				if (!safety_car) {
					skillStateContext.setState(&SkillBaseState::skillStopped);
					stateContext.stop();
					stopped = 1;
					HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
					osDelay(500);
					osThreadSuspend(SteerControl_TaskHandle);
					SetServo_motor(0);
				} else {
					stopped = 1;
					speed_control_enabled = false;
					SetServo_motor(0);

				}
				break;
			case 1:

				PIDk.dGain = (float)*int_ptr;
				BT_send_msg(&PIDk.dGain, "PIDkD");
				break;
			case 2:
				PIDk.pGain = (float)*int_ptr;
				BT_send_msg(&PIDk.pGain, "PIDkP");
				break;
			case 3:
				PIDk.iGain = *flt_ptr;
				BT_send_msg(&PIDk.iGain, "PIDkI");
				break;
			case 4:
				FAST = *flt_ptr;
				BT_send_msg(&FAST, "FAST");
				break;
			case 5:
				sid = stateContext.getStateId();
				BT_send_msg(&sid, "state");
				break;
			case 6:
				//PIDk.dGain = *flt_ptr;
				BT_send_msg(&PIDk.dGain, "PIDkD");
				break;
			case 7:
				//PIDk.pGain = *flt_ptr;
				BT_send_msg(&PIDk.pGain, "PIDkP");

				break;
			case 8:
				A = *flt_ptr;
				BT_send_msg(&A, "A");
				break;
			case 9:
				B = *flt_ptr;
				BT_send_msg(&B, "B");
				break;
			default:
				break;
		}
	}
}

void BT_send_msg(bool b, string s) {
	static int bInt;
	if (b) {
		bInt = 1;
	} else {
		bInt = 0;
	}
	BT_send_msg(&bInt, s);
}

void BT_send_msgFloat(float data, string s) {
	static float dataFloat;
	dataFloat = data;
	BT_send_msg(&dataFloat, s);
}

void BT_send_msgInt(int data, string s) {
	static int dataInt;
	dataInt = data;
	BT_send_msg(&dataInt, s);
}

extern float pTerm;
extern float dTerm;
extern float iTerm;

void SendRemoteVarTask()
{
	int sendRemoteCounter = 0;
	int slowSendMultiplier = 5;
	// minden remote vÃ¡ltozÃ³hez kÃ¼len kellenek ezek
	osThreadSuspend(SendRemoteVar_TaskHandle);
	float myfloat = 1;
	int eventInt = -1;
	int dirInt = -1;
	for(;;)
	{

		if (!stopped) {
			//BT_send_msg(&myfloat, "myfloat");
			myfloat +=1;
		}



		/*BT_send_msg(&speed_global, "speed");
		float asdf;
		asdf = calculateMovingAverage(FrontSensorMedian);
		BT_send_msg(&FrontSensorMedian, "contMed");
		BT_send_msg(&asdf, "contAwert");*/

		//minden ciklusban elküldi ezeket

		//BT_send_msg(&PIDm.iState, "mIState");
		//BT_send_msg(&Distance_sensors[2], "contLeft");
		//BT_send_msg(&Distance_sensors[3], "contRight");




		//minden slowSendMultiplier ciklusban küldi el ezeket
		if (sendRemoteCounter % slowSendMultiplier == 0) {
			BT_send_msg(&speed_global, "speed");

			//BT_send_msg(&pTerm, "contP");
			//BT_send_msg(&iTerm, "contI");
			//BT_send_msg(&dTerm, "contD");
			BT_send_msg(&Distance_sensors[1], "contFront");
			BT_send_msg(&speed_control, "control_speed");
			dirInt = direction;
			BT_send_msg( &dirInt, "dir");
			/*
			static float lastFr = 0;

			float d = (lastFr - fr_distance)*(-20000);
			lastFr = fr_distance;
			BT_send_msg(&d , "contD");
			float contNew = d + speed_control;
			BT_send_msg(&contNew , "contNew");


			BT_send_msg(&Distance_sensors[1], "contFro");
			//float asdf = fr_distance*1000;
			//BT_send_msg(&asdf, "contfdist");
			BT_send_msg(&SET_SPEED, "SET_SPEED");
			BT_send_msg(safety_car, "safety");
			BT_send_msg(skillTrack, "skillTr");



*/


			BT_send_msg(&activeLine1, "actL1");
			BT_send_msg(&activeLine2, "actL2");
			BT_send_msg(&pid, "PD");
			//BT_send_msg(&timer, "tick:" + std::string(itoa(systick_count(),buffer,10)) + "\n");
			//BT_send_msg(&timer, "radio:" + std::string(itoa(Radio_get_char(),buffer,10)) + "\n");
			BT_send_msg(&globalDistance, "globalDist");
			eventInt = stateData.event;
			BT_send_msg(&eventInt, "eventInt");
			BT_send_msg(&eventInt, "eInt");

			//BT_send_msg(&index, "index");
			//BT_send_msg(stable0lines, "stable0lines");
			//BT_send_msg(stable1lines, "stable1lines");
			BT_send_msgInt(skillStateContext.state->triggerGlobalDistance, "trigDist");

			BT_send_msg(&Distance_sensors[2], "left");
			BT_send_msg(&Distance_sensors[3], "right");

			BT_send_msg(bordas_bal, "bBordas");
			BT_send_msg(bordas_jobb, "jBordas");
			BT_send_msg(fal_bal, "bFal");
			BT_send_msg(fal_jobb, "jFal");
			//BT_send_msg(&encoderPos, "encoder");
			BT_send_msg(checkDirection, "checkDir");

			BT_send_msg(giro_fall, "fall");
			//BT_send_msg(giro_lejto, "lejto");
			//BT_send_msg(giro_emelkedo, "emelked");
			BT_send_msgFloat(giro_get_angle_Y(), "lejtSzog");

			//BT_send_msg(&PIDk.dState, "contDst");
			//BT_send_msg(&fr_distance, "contDist");
			//BT_send_msg(&distance_error, "contErr");

			//BT_send_msg(&timer, "enc:" + std::string(itoa(encoderPos,buffer,10)) + "\n");

			//BT_send_msg(&timer, "fal:" + std::string(itoa(vanfal,buffer,10)) + "\n");
			//BT_send_msg(&timer, "borda:" + std::string(itoa(bordas_bal,buffer,10)) + "\n");
			//BT_send_msg(&timer, "Z:" + std::string(itoa(giro_get_angle_Z(),buffer,10)) + "\n");
			//BT_send_msg(&timer, "lim:" + std::string(itoa(speed_under_X,buffer,10)) + "\n");
			//BT_send_msg(&stopped, "stopped");
			//sendSensors();
			//sendDebugVars();
			//sendTuning();
			sendStateData();
			//sendPIDs();
		}

		sendRemoteCounter++;
		osDelay(50);
	}

}

void sendSensors() {

	for(int i = 0; i<32; i++)
	{
		szenzorsor_temp_1[i] = szenzorsor_1[i];
		szenzorsor_temp_2[i] = szenzorsor_2[i];
	}

	for(int i = 0; i<32; i++)
	{
		if(i<10)
		{
			BT_send_msg(&szenzorsor_temp_1[i], "sens10" + std::string(itoa(i,buffer,10)));
			BT_send_msg(&szenzorsor_temp_2[i], "sens20" + std::string(itoa(i,buffer,10)));
		}else{
			BT_send_msg(&szenzorsor_temp_1[i], "sens1" + std::string(itoa(i,buffer,10)));
			BT_send_msg(&szenzorsor_temp_2[i], "sens2" + std::string(itoa(i,buffer,10)));
		}
	}

	//BT_send_msg(&speed_global, "speed");
}


void sendDebugVars() {
	//BT_send_msg(&SET_SPEED, "SET_SPEED");
	//BT_send_msg(&linePosM, "linePosM");

	//BT_send_msg(&testLinePos, "testLinePos");
	BT_send_msg(&pid, "PIDBOOL");

	//BT_send_msg(&activeLine1, "actL1");
	//BT_send_msg(&activeLine2, "actL2");

	//BT_send_msg(&angle, "angle");
	//BT_send_msg(&control, "control");
	//BT_send_msg(&speed_error, "speed_error");
	BT_send_msg(&speed_control, "control_speed");
	//BT_send_msg(&last_speed_control, "lilast_speed_controlnePosM");
	//BT_send_msg(&last_active_line_pos1, "last_pos1");
	//BT_send_msg(&last_active_line_pos2, "last_pos2");

	//Állapotteres
	//BT_send_msg(&A, "A");
	//BT_send_msg(&B, "B");
	//BT_send_msg(&state_struct.state, "state");
	//BT_send_msg(&state_struct.nextState, "nextState");


	BT_send_msg(&globalLines.numLines1, "numLines1");
	BT_send_msg(&globalLines.numLines2, "numLines2");

	BT_send_msg(&globalLines.pos1[0], "front_0");
	BT_send_msg(&globalLines.pos1[1], "front_1");
	BT_send_msg(&globalLines.pos1[2], "front_2");
	BT_send_msg(&globalLines.pos2[0], "back_0");
	BT_send_msg(&globalLines.pos2[1], "back_1");
	BT_send_msg(&globalLines.pos2[2], "back_2");
}

void sendStateData() {
	static int stateId = -10;
	if (skillTrack) {
		stateId = skillStateContext.state->stateId;
		string stName = "stnm" + skillStateContext.state->name;
		BT_send_msg(&globalDistance, stName);
	} else {
		stateId = stateContext.getStateId();
		BT_send_msg(&stateId, "StateID");
		BT_send_msg(stable3lines, "stable3lines");
	}

	//int controlled =0;
	//if (skillStateContext.state->steeringControlled) {
		//controlled = 1;
	//}
	//BT_send_msg(&controlled, "controlled");

	//BT_send_msg(&skillStateContext.state->steeringAngle, "steerAngle");
	//BT_send_msg(&stAngle, "stAngle");


	//string stName = "stnm" + skillStateContext.state->name;
	//BT_send_msg(&globalDistance, stName);





}

void sendPIDs() {
	BT_send_msg(&PIDs.pGain, "PIDs_p");
	BT_send_msg(&PIDs.dGain, "PIDs_s");
}

void sendPIDm() {
	BT_send_msg(&PIDm.pGain, "PIDm_p");
	BT_send_msg(&PIDm.dGain, "PIDm_s");
}

float getDistance() {

	static float distanceArray[5];
	static int index = 0;
	static float dist;
	dist = (1.0f/((float)Distance_sensors[1]));

	if (index == 5) {
			index = 0;
		}

	distanceArray[index] = dist;


	static float min = 100000;
	min = 10000;

	for(int i = 0; i<5; i++) {
		if (distanceArray[i] < min) {
			min = distanceArray[i];
		}
	}

	if (min == 0) {
		min = dist;
	}

	index++;

	return min;

}



// szabályzó task
void SteerControlTask() {



	start_radio_done = false;
	float speed = 0;

	int led_cntr = 0;

	int numLinesArray[5];
	int numLinesArrayIndex = 0;
	int numLinesSum = 0;
	bool usePD = true;

	//LineS
	for (int i = 0; i < 5; i++) {
		numLinesArray[i] = 1;
	}

	float fr_distance_last = 0;
	float FrontSensorAverage_last;
	int no_line_cycle_count = 0;

	// állapotgép init
	//state_struct.state = 0;

	// állapot visszacsatolás paraméterei
	//A = 0.4;	// sebesség függés	// d5% = v*A + B
	//B = 0.4;	// konstans

	// szervo PD szabályzó struktúrája

	PIDs.pGain = 30;
	PIDs.iGain = 0;
	PIDs.dGain = -230;
	PIDs.iMax = 300;
	PIDs.iMin = -300;
	PIDs.iState = 0;
	PIDs.dState = 0;

	// motor PI szabályzó struktúra
	if (skillTrack) {
		PIDm.pGain = 500;		// 100-> 5m/s hibánál lesz 500 a jel (max)
		PIDm.iGain = 3;			// pGain/100?
		PIDm.dGain = 0;
		PIDm.iMax = 500;
		PIDm.iMin = -100;
		PIDm.iState = 0;
		PIDm.dState = 0;
	} else {

		PIDm.pGain = 400;		// 100-> 5m/s hibánál lesz 500 a jel (max)
		PIDm.iGain = 0.7;
		PIDm.dGain = 0;
		PIDm.iMax = 100;
		PIDm.iMin = 0;
		PIDm.iState = 0;
		PIDm.dState = 0;

	}

	float error = 0;

	// követés szabályzó struktúra
	PIDk.pGain = -15000.0f;		// 100-> 5m/s hibánál lesz 500 a jel (max)
	PIDk.iGain = -200;			// pGain/100?
	PIDk.dGain = -200000;
	PIDk.iMax = 0.1;
	PIDk.iMin = -0.1;
	PIDk.iState = 0;
	PIDk.dState = 0;

	stateData.event = UNSTABLE;

	osThreadSuspend(SteerControl_TaskHandle);
	int ll = -100;
	BT_send_msg(&ll, "LastLine");

	for (;;) {
		// várakozás a startjelre
		while (!start_radio_done && skillTrack) {
			if (Radio_get_char() == 48) {
				start_radio_done = true;
				stateData.event = RADIOSTART;
			}
			osDelay(5);
		}

		//__HAL_TIM_SET_COUNTER(&htim5,0);
		timeCounter++;

		// sebesség mérés
		osThreadSuspendAll();
		speed = speed_global;
		osThreadResumeAll();


		// motor szabályozás
		if (speed_control_enabled) {
			if (skillTrack) {
				speed_error = SET_SPEED - speed;
				speed_control = UpdatePID1(&PIDm, speed_error, speed);

				// negatív irányt megerõsíteni	// motor bekötéstõl függ!!!

				if (SET_SPEED == 0) {	//TODO
					PIDm.iState = 0;
				}

				if (speed_control < 0 && SET_SPEED > -0.1) {
					//speed_control *= 10;
					if (speed_control > -80) {
						speed_control = -80;
					}
					if (last_speed_control < 0) {
						//speed_control = 0;
					}
				}

				// fékezés logika és gyorsulás logika
				if (last_speed_control > 0 && speed_control < 0) {
					//speed_control = 0;	//TODO ez fölösleges, nem?
				} else if (speed_control > 0
						&& speed_control > (last_speed_control + ACC_MAX)
						&& last_speed_control >= 0) {
					speed_control = last_speed_control + ACC_MAX; // gyorsulás korlát
				}

				SetServo_motor((int) speed_control);
			} else {
				safety_car = stateContext.state->isSafety;
				if (!safety_car) {
					speed_error = SET_SPEED - speed;
					speed_control = UpdatePID1(&PIDm, speed_error, speed);

					// negatív irányt megerõsíteni	// motor bekötéstõl függ!!!

					/*if (SET_SPEED == 0) {	//TODO
					 PIDm.iState = 0;
					 }*/

					if (speed_control < 0 && SET_SPEED > -0.05) {
						//speed_control *= 10;
						if (speed_control > -80) {
							speed_control = -80;
						}
						if (last_speed_control < 0) {
							//speed_control = 0;
						}
					}

					// fékezés logika és gyorsulás logika
					if (last_speed_control > 0 && speed_control < 0) {
						//speed_control = 0;	//TODO ez fölösleges, nem?
					} else if (speed_control > 0
							&& speed_control > (last_speed_control + ACC_MAX)
							&& last_speed_control >= 0) {
						speed_control = last_speed_control + ACC_MAX; // gyorsulás korlát
					}

					SetServo_motor((int) speed_control);
				} else if (safety_car) {
					// hiba negatív, ha messzebb vagyunk -> negatív PGain
					ADC2_read();
					fr_distance = (1.0f / ((float) Distance_sensors[1])); //getDistance();
					//float asdf;
					FrontSensorAverage = calculateMovingAverage(fr_distance);

					//fr_distance = (1.0f/FrontSensorAverage);

					//max távolodás
					static float max_tavolodas = 0.000255f;
					if (fr_distance > fr_distance_last + max_tavolodas) {
						fr_distance = fr_distance_last + max_tavolodas;
					} else if (fr_distance < fr_distance_last - max_tavolodas) {
						fr_distance = fr_distance_last - max_tavolodas;
					}

					fr_distance_last = fr_distance;
					//BT_send_msg(&distance, "dist");
					distance_error = SET_DISTANCE - fr_distance;
					speed_control = UpdatePID1(&PIDk, distance_error,
							FrontSensorAverage);

					// negatív irányt megerõsíteni	// motor bekötéstõl függ!!!

					/*if(speed_control < 0)
					 {
					 //speed_control *= 10;
					 if (speed_control > -80) {
					 speed_control = -80;
					 }
					 if (last_speed_control < 0) {
					 //speed_control = 0;
					 }
					 }*/

					if (speed < 0.01 && speed_control < 0) {
						speed_control = 0;
					} else {
						if (SET_SPEED == 0) {
							speed_control = 0;
						} else if (speed > SET_SPEED) {
							if (speed_control > SET_SPEED * 35.0f)// csak ha nagyobbat akarna adni
									{
								speed_control = SET_SPEED * 35.0f;
							}
						} else {

						}
					}

					float safety_accmax = 15;
					// fékezés logika és gyorsulás logika
					if (speed_control > 0
							&& speed_control
									> (last_speed_control + safety_accmax)
							&& last_speed_control >= 0) {
						speed_control = last_speed_control + safety_accmax; // gyorsulás korlát
					}

					// túl közel védelem
					if (Distance_sensors[1] > 230) {
						SetServo_motor(0);

					} else {
						SetServo_motor((int) speed_control);
					}
				}
			}
		}

		//motorszabályzás vége

		last_speed_control = speed_control;

		encoderPos = __HAL_TIM_GET_COUNTER(&htim2);		// állapotgépnek

		// szenzor adatok beolvasása
		ReadSensors();
		//ReadSensorsDummy();

		if (skillTrack) {
			ADC2_read();		// blokkol, 40us

			wall_detection();	// falas bool-okat állítja
			wall_borda_detection();

		}
		giro_integrate();
		is_speed_under_X(speed, speed_limit);
		//update_direction();

		// szenzor adatok feldolgozása
		globalLines = getLinePos(20);

		if (skillTrack) {
			///// stabil vonalszám
			get_stable_line_count(globalLines.numLines1);
			get_stable_line_count2(globalLines.numLines1,
					globalLines.numLines2);
		} else {

			///// stabil 3 vonal
			numLinesArray[numLinesArrayIndex] = globalLines.numLines1;
			numLinesArrayIndex++;
			if (numLinesArrayIndex >= 5) {
				numLinesArrayIndex = 0;
			}
			numLinesSum = 0;
			for (int i = 0; i < 5; i++) {
				numLinesSum += numLinesArray[i];
			}

			if (numLinesSum > 12) {
				stable3lines = true;
				//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
			} else if (numLinesSum < 8) {
				stable3lines = false;
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
			}
		}

		getActiveLinePos(&globalLines, &last_active_line_pos1,
				&last_active_line_pos2, &activeLine1, &activeLine2);
		last_active_line_pos1 = activeLine1;
		last_active_line_pos2 = activeLine2;

		angle = calculateAngle(activeLine1, activeLine2);

		linePosM = (activeLine1 - 15.5) * 5.9 / 1000; // pozíció, méterben, középen 0

		if (skillTrack) {
			utanfutoPressed = get_switch();
			skillStateContext.state->update();
			usePD = false;
			SET_SPEED = skillStateContext.state->targetSpeed;
			steeringControl = skillStateContext.state->steeringControlled;
			checkDirection = skillStateContext.state->chkDir;
		} else {
			steeringControl = true;
			stateContext.update(stable3lines, encoderPos);
			SET_SPEED = stateContext.state->targetSpeed;
			//usePD = stateContext.isSteeringPD();
			usePD = !speed_under_X;
		}

		// vonalkövetés szabályozó

		if (steeringControl) {
			if (globalLines.numLines1 != -1)	// ha látunk vonalat
					{
				if (usePD) {
					pid = 1;
					error = activeLine1 - 15.5;
					control = UpdatePID1(&PIDs, error, globalLines.pos1[0]);
					if (speed > 2) {
						//control /= (speed/2.0);
					}
					SetServo_steering((int) control); // PID-hez
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
				} else {
					pid = 0;
					if (speed < 0.2) {
						speed = 0.2;
					}

					control = UpdateStateSpace(A, B, speed, linePosM, angle);
					SetServo_steering(control);
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
				}

				no_line_cycle_count = 0; 	// láttunk vonalat
			} else if (globalLines.numLines2 != -1) {
				no_line_cycle_count = 0;
			} else {
				no_line_cycle_count++;
				if (no_line_cycle_count > NO_LINE_CYCLES) {
					//SET_SPEED = 0;
					stateContext.stop();
					SetServo_motor(0);
					BT_send_msg(&activeLine1, "LastLine");

					//osThreadSuspend(SteerControl_TaskHandle);
					speed_control_enabled = false;
				}
			}
		} else {
			if (checkDirection) {
				if (direction == RIGHT) {
					stAngle = skillStateContext.state->steeringAngle;
				} else if (direction == LEFT) {
					stAngle = skillStateContext.state->steeringAngle * (-1);
				} else {
					stAngle = skillStateContext.state->steeringAngle;
				}
			} else {
				stAngle = skillStateContext.state->steeringAngle;
			}

			SetServo_steering(stAngle);
		}

		if (led_cntr == 10) {
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
			led_cntr = 0;
		} else {
			led_cntr++;
		}
		//osThreadSuspend(SteerControl_TaskHandle);

		timer = __HAL_TIM_GET_COUNTER(&htim5);
		if (timer > 13000) {
			static int ccounter = 0;
			//BT_send_msg(&timer, "time:" + std::string(itoa(timer,buffer,10)) + "\n");
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
			BT_send_msg(&timer,"tCycle" + std::string(itoa(ccounter,buffer,10)));
			ccounter++;
		}
		__HAL_TIM_SET_COUNTER(&htim5, 0);

		osDelay(9);
	}
}

void SetSkillTrack()
{
	if(DIP(4))
		{
			skillTrack = true;
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
		}else{
			skillTrack = false;
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
		}
}

void TurnOnTask()
{
	bool started = false;
	for(;;)
	{
		if(!started)
		{
			if(START_PIN())
			{
				started = true;
			}
		}

		if(started)
		{
			if(DIP(4))
			{
				skillTrack = true;
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
			}else{
				skillTrack = false;
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
			}

			osDelay(100);
			osThreadResume(SteerControl_TaskHandle);

			giro_init();

			osThreadResume(SendRemoteVar_TaskHandle);



			stateContext.start(encoderPos);
			stateData.event = RADIOSTART;

			osThreadSuspend(Turn_On_TaskHandle);
		}
		osDelay(100);
	}
}

// indítás óta eltelt millisecec száma
int systick_count()
{
	return xTaskGetTickCount();
}

void SetServo_sensor_jobb()
{
	SetServo_sensor(-1*FrontSensorTurn);
}
void SetServo_sensor_bal()
{
	SetServo_sensor(FrontSensorTurn);
}


// 5 sec blokk, ha nem jön üzenet
// start: 53-52-51-50-49...48  // 77 = timeout, 88 egyéb hiba
uint8_t Radio_get_char()
{
	static HAL_StatusTypeDef uart_status;
	static uint8_t msg = 99;

	uart_status = HAL_UART_Receive(&huart1,&msg,(uint16_t)1,5000);
	if(uart_status == HAL_OK)
	{
		return msg;
	}else if(uart_status == HAL_TIMEOUT)
	{
		return 77;
	}else{
		return 88;
	}
}

bool get_switch()
{
	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11))
	{
		return true;
	}else{
		return false;
	}
}

// true, ha ON felé van
bool DIP(int num)
{
	switch (num)
	{
	case 1:
		if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11)){return false;}
		else {return true;}
		break;
	case 2:
		if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9)){return false;}
		else {return true;}
		break;
	case 3:
		if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8)){return false;}
		else {return true;}
		break;
	case 4:
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8)){return false;}
		else {return true;}
		break;
	default:
		return false;
		break;
	}
}

// true, ha le van húzva
bool START_PIN()
{
	if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_6))
	{
		return true;
	}else{
		return false;
	}
}

void update_direction()
{
	if(bordas_jobb)
	{
		direction = LEFT;
	}else if(bordas_bal){
		direction = RIGHT;
	}
}

void get_stable_line_count(int numlines)
{
	static int numlines_array[5];
	static int array_cntr = 0;
	static int num0, num1, num2, num3;

	num0 = num1 = num2 = num3 = 0;
	stable0lines = stable1lines = stable2lines = stable3lines = keresztvonal = false;

	if(numlines == -2)	// keresztvonalra nem várunk 5 ütemet
	{
		keresztvonal = true;
		return;
	}

	numlines_array[array_cntr] = numlines;
	array_cntr++;
	if(array_cntr >= 5)
	{
		array_cntr = 0;
	}

	for(int i=0; i<5; i++)
	{
		switch (numlines_array[i])
		{
			case -1:
				num0++;
				break;
			case 1:
				num1++;
				break;
			case 2:
				num2++;
				break;
			case 3:
				num3++;
				break;
		}
	}

	if(num0 >= 4){ stable0lines = true; }
	if(num1 >= 4){
				stable1lines = true;
		}
	if(num2 >= 4){ stable2lines = true; }
	if(num3 >= 4){ stable3lines = true; }

}

void get_stable_line_count2(int numlines1, int numlines2)
{
	static int numlines_array1[5];
	static int numlines_array2[5];
	static int array_cntr = 0;
	static int num1;
	static int num0;

	num0 = num1 = 0;
	stable1linesForBoth = false;
	stable0linesForBoth = false;


	numlines_array1[array_cntr] = numlines1;
	numlines_array2[array_cntr] = numlines2;
	array_cntr++;
	if(array_cntr >= 5)
	{
		array_cntr = 0;
	}

	for(int i=0; i<5; i++)
	{
		if (numlines_array1[i] == 1 && numlines_array2[i] == 1) {
			num1++;
		} else if (numlines_array1[i] == -1 && numlines_array2[i] == -1) {
			num0++;
		}
	}

	if(num0 >= 4) {
		stable0linesForBoth = true;
	}
	if(num1 >= 4){
		stable1linesForBoth = true;
	}
	//if(num2 >= 4){ stable2lines = true; }
	//if(num3 >= 4){ stable3lines = true; }

}

void getActiveLinePos(LineState * Lines, float *last_pos1, float *last_pos2, float * active1, float * active2)
{
	if (Lines->numLines1 == 3) {
		*active1 = Lines->pos1[1];
	}
	else if(Lines->numLines1 == 2)		// elöl 2 vonal
	{
		//ha ügyességi
		if(skillTrack && stable2lines)
		{
			if(direction == LEFT)
			{
				*active1 = Lines->pos1[0];
			}
			else
			{
				*active1 = Lines->pos1[1];
			}
		}
		else	// ha gorsasági
		{
			if( abs( (*last_pos1) - Lines->pos1[0] ) < abs( (*last_pos1) - Lines->pos1[1]) )
			{
				*active1 = Lines->pos1[0];
			}
			else
			{
				*active1 = Lines->pos1[1];
			}
		}
	}
	else if (Lines->numLines1 == 1)
	{
		*active1 = Lines->pos1[0];
	}
	else if (Lines->numLines1 == -1)
	{
		*active1 = *last_pos1;
	}

	if (Lines->numLines2 == 3) {
		*active2 = Lines->pos2[1];
	}
	else if(Lines->numLines2 == 2)
	{
		//ha ügyességi
		if(skillTrack)
		{
			if(direction == LEFT)
			{
				*active2 = Lines->pos2[0];
			}
			else
			{
				*active2 = Lines->pos2[1];
			}
		}
		else	// ha gorsasági
		{
			if( abs( (*last_pos2) - Lines->pos2[0]) < abs( (*last_pos2) - Lines->pos2[1]) )
			{
				*active2 = Lines->pos2[0];
			}
			else
			{
				*active2 = Lines->pos2[1];
			}
		}
	}
	else if (Lines->numLines2 == 1)
	{
		*active2 = Lines->pos2[0];
	}
	else if (Lines->numLines2 == -1)
	{
		*active2 = *last_pos2;
	}

	//TODO külön függvénybe megadott numLines kéne paraméterben

}

void is_speed_under_X(float speed, float limit)
{
	static float array[3];
	static int cntr = 0;
	array[cntr] = speed;
	if(cntr > 1)
	{
		cntr = 0;
	}else{
		cntr++;
	}

	for(int i=0; i<3; i++)
	{
		if(array[i]>limit)
		{
			speed_under_X = false;
			return;
		}
	}
	speed_under_X = true;
}

// hal uart callback
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART3)
	{
		osSignalSet(BT_TaskHandle,0x0001);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART3)
	{
		osSignalSet(BT_Receive_TaskHandle,0x0001);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
    if(AdcHandle->Instance == ADC1)	// szenzorsor 4 jele
    {
    	osSemaphoreRelease(ADC1_complete);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
	// TIM6 -> szenzor felfutás idõzítése
	if(htim->Instance == TIM6)
	{
		// SteerControlTask 1-es signal-ja
		osSignalSet(SteerControl_TaskHandle,0x0001);
		//HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15);
	}
}


// interrupt függvények csak így mennek
extern "C"
{
	// uart2 megszakÃ­tÃ¡s kezelÅ‘, meghÃ­vjuk a HAL irq kezelÅ‘jÃ©t
	void USART3_IRQHandler(void){
		HAL_UART_IRQHandler(&huart3);
	}

    void DMA2_Stream0_IRQHandler()
    {
        HAL_DMA_IRQHandler(&hdma_adc1);
    }

    void ADC_IRQHandler()
    {
    	HAL_ADC_IRQHandler(&hadc1);
    }


    // ezzel várunk a szenzor felfutására
    void TIM6_DAC_IRQHandler()
    {
    	HAL_TIM_IRQHandler(&htim6);
    }

    void TIM7_IRQHandler()
	{
		HAL_TIM_IRQHandler(&htim7);
	}
}

// ADC init (szenzorsor 4 jele)
void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION8b;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc1);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 3;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 4;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/* ADC2 init function */
void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION8b;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 5;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc2);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 3;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 4;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 5;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);


}

/* SPI2 init function */
void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi2.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi2);

}

// LED driver vezérlés
void MX_SPI3_Init(void)
{

  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi3.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi3);

}

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  TIM_OC_InitTypeDef sConfigOC;

  // alap clk = 168MHz
  // 10ms-es pwm periódus kell, azon belül 1-2ms a pulzus


  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 167;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9999;	// 10/20ms itt állítható
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;	// ez nem mûködik
  htim1.Init.RepetitionCounter = 0;
  HAL_TIM_PWM_Init(&htim1);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);

}

// encoder számláló
void MX_TIM2_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4000000000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_INDIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 2;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 2;
  HAL_TIM_Encoder_Init(&htim2, &sConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  __HAL_TIM_SET_COUNTER(&htim2, 1000000000);

  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);

}

// idõméréshez, 1MHz-en számol (1us)
void MX_TIM5_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 83;	// 1us/1MHz
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim5);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig);

  HAL_TIM_Base_Start(&htim5);
}

// szenzor felfutás idõzítése
void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 83;	// 1us/1MHz
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 60000;
  HAL_TIM_Base_Init(&htim6);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig);

  //HAL_TIM_Base_Start_IT(&htim6);
  //HAL_TIM_Base_Stop_IT(&htim6);
}

// ezzel idõzítjük a szabályozó task lefutását (vagy legyen Delay(10)?)
void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 167;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 200;
  HAL_TIM_Base_Init(&htim7);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig);

}

/* USART1 init function */
// TODO: interrupt
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

}

void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

// uart3 Ã©s hozzÃ¡tartozÃ³ interruptok inicializÃ¡lÃ¡sa BT modulhoz
void USART3_UART_Init()
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart3);

  HAL_NVIC_SetPriority(USART3_IRQn,14,0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}













