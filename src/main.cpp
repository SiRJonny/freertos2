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

#define SERVO_RANGE_MOTOR 700	// max eltérés 0-tól, 1500us +/- SERVO_RANGE a max kiadott jel
#define SERVO_RANGE_STEERING 300	// max eltérés 0-tól, 1500us +/- SERVO_RANGE a max kiadott jel
#define MOTOR_CONTROL_ENABLED 1
#define SERVO_CONTROL_ENABLED 1

#define BTN_TUNEPID 0

#define SERVO_CONTROL_STATESPACE 1
#define SERVO_CONTROL_PID 0


#define PID_PGAIN 20
#define PID_IGAIN 0
#define PID_DGAIN -500

float ACC_MAX = 200;		// egy szabályzó periódusban max ennyivel növekedhet a motor szervo jele
int NO_LINE_CYCLES = 50;

float SLOW = 1.3;
float FAST = 2.9;
float STOP = 0.0;

float PID_LIMIT = 1.1;

using namespace std;

float TEST_SPEED = 0;
float TEST_DELAY = 60000;


float linePosM;		// vonalpozíció méterben
float angle = 0;
float control = 0;
float speed_error = 0;
float speed_control = 0;
float last_speed_control = 0;
float last_active_line_pos1 = 15.5;
float last_active_line_pos2 = 15.5;

LineState globalLines;

int testLinePos = 10;

float A = 0.4;	// sebesség függés	// d5% = v*A + B
float B = 0.4;	// konstans

int pid = 0;

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId BT_TaskHandle;
osThreadId SendRemoteVar_TaskHandle;
osThreadId SteerControl_TaskHandle;
osThreadId BT_Receive_TaskHandle;

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







/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint32_t ADC1_BUFFER[4];
uint32_t szenzorsor_1[32];
uint32_t szenzorsor_2[32];

uint32_t szenzorsor_temp_1[32];
uint32_t szenzorsor_temp_2[32];

float posArray[100];
float controlArray[100];

PID_struct PIDs;
PID_struct PIDm;
state_machine_struct state_struct;

float SET_SPEED = 0;
float speed_global = 0;
int activeLineNum = 0;

bool TunePID = false;
char buffer[10];	//bt msg hez
int timer = 0; // idõméréshez

float activeLine1 = 0;  // középsõ vonal kiválasztása
float activeLine2 = 0;

int ONE = 1;
int stopped = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void StartDefaultTask();
void StartButtonTask();
void SendBluetoothTask();
void SendRemoteVarTask();
void SteerControlTask();
void BTReceiveTask();

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
void SetServo_steering(int pos); // -SERVO_RANGE_STEERING +SERVO_RANGE_STEERING
void SetServo_steering(float angle);  // kormányzás, szöggel
void getActiveLinePos(LineState * Lines, float *last_pos1, float *last_pos2, float * active1, float * active2);

void sendSensors();
void sendTuning();
void sendDebugVars();
void sendStateData();


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{


  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */


  xSem_USART_rdy_to_send = osSemaphoreCreate(osSemaphore(xSem_USART_rdy_to_send),1);
  ADC1_complete = osSemaphoreCreate(osSemaphore(ADC1_complete),1);
  osSemaphoreWait(ADC1_complete,osWaitForever);

	// itt, hogy BT tasknak mÃ¡r kÃ©sz legyen
  xQueue_BT = xQueueCreate(10, sizeof(struct BT_MSG));



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

  osThreadDef(SteerControlTask, SteerControlTask, osPriorityHigh, 0, 128);
  SteerControl_TaskHandle = osThreadCreate(osThread(SteerControlTask), NULL);

  osThreadDef(BTReceiveTask, BTReceiveTask, osPriorityNormal, 0, 128);
  BT_Receive_TaskHandle = osThreadCreate(osThread(BTReceiveTask), NULL);




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

/** System Clock Configuration
*/



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


/*
void BT_send_msg(int msg){
	struct BT_MSG msg_int;
	int2msg(&msg_int, msg, "unnamed\n");
	xQueueSend( xQueue_BT, &msg_int, portMAX_DELAY);

}*/


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


/*
void BT_send_msg(int * msg, string nev){
	// type: 1=int, 3=float, 4=double
	xQueueSend( xQueue_BT, &number2msg(msg, string(nev).c_str(), (uint8_t)1), portMAX_DELAY);
}

void BT_send_msg(float * msg, string nev){
	// type: 1=int, 3=float, 4=double
	xQueueSend( xQueue_BT, &number2msg(msg, string(nev).c_str(), (uint8_t)3), portMAX_DELAY);
}

void BT_send_msg(double * msg, string nev){
	// type: 1=int, 3=float, 4=double
	xQueueSend( xQueue_BT, &number2msg(msg, string(nev).c_str(), (uint8_t)4), portMAX_DELAY);
}
*/

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
	state_struct.state = -1;
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
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, 1500 + pos + 35);
	//TODO offset
}

// float szög
void SetServo_steering(float angle_rad)
{
	float pos = (angle_rad / 6.28 * 360) * 10.66666;	// 10.6666 csinál fokból pwm-et
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
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, 1500 + (int)pos + 35);
}




/* StartDefaultTask function */
// default tastk, csak egy villogÃ³ led
void StartDefaultTask()
{
	int i = 0;
	int last = 0;
	int current = 0;
	int encoderPos = 1000000000;
	int lastEncoderPos = 1000000000;	// encoder számláló innen indul, hogy semerre ne legyen túlcsordulás
	int encoderPosArray[5];
	int buffer_cntr = 0;
	float speed_temp = 0;

	for(int i = 0; i < 5; i++)
	{
		encoderPosArray[i] = 1000000000;
	}

	for(;;)
	{

		// sebesség mérés
		encoderPos = __HAL_TIM_GET_COUNTER(&htim2);
		encoderPosArray[buffer_cntr] = encoderPos;
		buffer_cntr++;
		if(buffer_cntr >= 5)
		{
			buffer_cntr = 0;
		}

		speed_temp = ((float)(encoderPosArray[buffer_cntr] - encoderPos))/(2571.0/20.0); //ez így m/s, ha 20 lukas a tárcsa és 50ms-enként mérünk (másodpercenként 20)

		osThreadSuspendAll();
		speed_global = speed_temp;
		osThreadResumeAll();

		//lastEncoderPos = encoderPos;


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

			/*SetServo_steering(70);
			osDelay(1000);
			SetServo_steering(0);*/
			//SET_SPEED = 1.2;
			//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14); // piros led, debug
			osThreadResume(SteerControl_TaskHandle);
			osThreadResume(SendRemoteVar_TaskHandle);
			//osDelay(TEST_DELAY);
			//SET_SPEED = 0.0;
			//state_struct.state = -1;
			//osDelay(4000);
			//SetServo_motor(0);
			//osThreadSuspend(SteerControl_TaskHandle);


			/*SetServo_motor(90);
			osDelay(1000);
			SetServo_motor(-500);
			osDelay(1000);
			//SetServo_motor(-600);
			osDelay(1000);*/

			/*SET_SPEED = 4;
			osThreadResume(SteerControl_TaskHandle);
			osDelay(3000);
			SET_SPEED = 0;
			osDelay(3000);
			osThreadSuspend(SteerControl_TaskHandle);*/

			/*SET_SPEED = 2;
			osThreadResume(SteerControl_TaskHandle);
			osDelay(1000);
			SET_SPEED = 0;
			osDelay(3000);
			osThreadSuspend(SteerControl_TaskHandle);*/


			#if (BTN_TUNEPID == 1)
			{

				osThreadResume(SteerControl_TaskHandle);
				osDelay(200);
				SET_SPEED = 2;
				TunePID = true;
			}
			#endif

			/*

			SetServo_steering(240);
			osDelay(2000);
			SetServo_steering(260);
			osDelay(2000);
			SetServo_steering(280);
			osDelay(2000);
			SetServo_steering(300);
*/


			/*osDelay(5);
			osThreadResume(SendRemoteVar_TaskHandle);*/

			/*SetServo_motor(100);
			osDelay(1000);
			SetServo_motor(-100);
			osDelay(3);
			SetServo_motor(0);
			osDelay(3);
			SetServo_motor(-500);
			osDelay(1000);
			SetServo_motor(0);*/

			/*string temp = "^#^$^%";
			struct BT_MSG msg;
			temp.copy(msg.data,sizeof(temp),0);
			msg.size = 6;

			HAL_UART_Transmit_IT(&huart3, (uint8_t*) msg.data, msg.size);*/



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
				state_struct.state = -1;
				stopped = 1;
				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
				//osDelay(2000);
				//osThreadSuspend(SteerControl_TaskHandle);
				break;
			case 1:

				PIDs.dGain = *int_ptr;

				BT_send_msg(&PIDs.dGain, "PIDd");
				break;
			case 2:
				PIDs.pGain = *int_ptr;

				BT_send_msg(&PIDs.pGain, "PIDp");
				break;
			case 3:
				SLOW = *flt_ptr;
				BT_send_msg(&SLOW, "SLOW");
				break;
			case 4:
				FAST = *flt_ptr;
				BT_send_msg(&FAST, "FAST");
				break;
			case 5:
				BT_send_msg(&state_struct.state, "state");
				break;
			case 6:
				testLinePos = *int_ptr;
				BT_send_msg(&testLinePos, "testLinePos");
				break;
			case 7:
				TEST_DELAY = *int_ptr;
				BT_send_msg(&TEST_DELAY, "testLinePos");
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

void SendRemoteVarTask()
{

	// minden remote vÃ¡ltozÃ³hez kÃ¼len kellenek ezek
	osThreadSuspend(SendRemoteVar_TaskHandle);
	float myfloat = 1;
	int myint = 1;

	for(;;)
	{

		if (!stopped) {
			BT_send_msg(&myfloat, "myfloat");
			myfloat +=1;
		}

		BT_send_msg(&stopped, "stopped");

		//sendSensors();
		sendDebugVars();
		//sendTuning();

		//osThreadSuspend(SendRemoteVar_TaskHandle); // minden elkÃ¼ldve, pihenÃ¼nk (osThreadResume-ra megint elkÃ¼ld mindent)
	    //HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
	    //osThreadSuspend(SendRemoteVar_TaskHandle);

		osDelay(300);
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

	BT_send_msg(&speed_global, "speed");
}

void sendTuning() {
	for(int i = 0; i < 100; i++) {
		if (i<10)
		{
			BT_send_msg(&posArray[i], "diag10" + std::string(itoa(i,buffer,10)));
			BT_send_msg(&controlArray[i], "diag20" + std::string(itoa(i,buffer,10)));
		}
		else
		{
			BT_send_msg(&posArray[i], "diag1" + std::string(itoa(i,buffer,10)));
			BT_send_msg(&controlArray[i], "diag2" + std::string(itoa(i,buffer,10)));
		}
	}
}

void sendDebugVars() {
	BT_send_msg(&speed_global, "speed");
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

	//BT_send_msg(&globalLines.pos1[0], "front_0");
	//BT_send_msg(&globalLines.pos1[1], "front_1");
	//BT_send_msg(&globalLines.pos1[2], "front_2");
	//BT_send_msg(&globalLines.pos2[0], "back_0");
	//BT_send_msg(&globalLines.pos2[1], "back_1");
	//BT_send_msg(&globalLines.pos2[2], "back_2");
}

void sendStateData() {


}

void sendPIDs() {
	BT_send_msg(&PIDs.pGain, "PIDs_p");
	BT_send_msg(&PIDs.dGain, "PIDs_s");
}

void sendPIDm() {
	BT_send_msg(&PIDm.pGain, "PIDm_p");
	BT_send_msg(&PIDm.dGain, "PIDm_s");
}


// szenzorsorok beolvasása, ha kész, szüneteli magát
void SteerControlTask()
{
	int encoderPos = 1000000000;
	int lastEncoderPos = 1000000000;	// encoder számláló innen indul, hogy semerre ne legyen túlcsordulás
	float speed = 0;

	int led_cntr = 0;

	int numLinesArray[5];
	int numLinesArrayIndex = 0;
	bool stable3lines = false;
	int numLinesSum = 0;
	bool usePD = true;

	//LineS
	for (int i = 0; i<5; i++) {
		numLinesArray[i] = 1;
	}


	int no_line_cycle_count = 0;

	// állapotgép init
	state_struct.state = 0;

	// állapot visszacsatolás paraméterei
	//A = 0.4;	// sebesség függés	// d5% = v*A + B
	//B = 0.4;	// konstans

	// szervo PD szabályzó struktúrája
	PIDs.pGain = 15;
	PIDs.iGain = 0;
	PIDs.dGain = -250;
	PIDs.iMax = 300;
	PIDs.iMin = -300;
	PIDs.iState = 0;
	PIDs.dState = 0;

	// motor PI szabályzó struktúra
	PIDm.pGain = 500;		// 100-> 5m/s hibánál lesz 500 a jel (max)
	PIDm.iGain = 2;			// pGain/100?
	PIDm.dGain = 0;
	PIDm.iMax = 300;
	PIDm.iMin = -100;
	PIDm.iState = 0;
	PIDm.dState = 0;

	/// PID tune segédváltozók
	bool tune_started = false;
	int cntr = 0;
	int tune_cntr = 0;
	char buffer[10];

	float error = 0;



	osThreadSuspend(SteerControl_TaskHandle);

	for(;;)
	{

		//__HAL_TIM_SET_COUNTER(&htim5,0);

		// TODO: sebességet elég ritkábban mérni? úgysem tud gyorsan változni -> pontosabb
		// de gyorsítás így késleltetve történik...

		// sebesség mérés
		osThreadSuspendAll();
		speed = speed_global;
		osThreadResumeAll();

		if (SET_SPEED > 2)
		{
			PIDm.iGain = 2;
		} else {
			PIDm.iGain = 0;
		}


		// motor szabályozó
		#if ( MOTOR_CONTROL_ENABLED == 1)
		{
			speed_error = SET_SPEED - speed;
			speed_control = UpdatePID1(&PIDm, speed_error, speed);

			// negatív irányt megerõsíteni	// motor bekötéstõl függ!!!

			if(speed_control < 0)
			{
				speed_control *= 0.5;
				if (speed_control > -80) {
					speed_control = -80;
				}
				if (last_speed_control < 0) {
					speed_control = 0;
				}
			}


			// fékezés logika és gyorsulás logika
			if(last_speed_control > 0 && speed_control < 0)
			{
				speed_control = 0;
			}
			else if(speed_control > 0 && speed_control > (last_speed_control + ACC_MAX) && last_speed_control >= 0)
			{
				speed_control = last_speed_control + ACC_MAX; 		// gyorsulás korlát
			}

			SetServo_motor( (int)speed_control );

		}
		#endif
		last_speed_control = speed_control;

		///////// Bluetooth debug küldés	////////////////
		//BT_send_msg(&speed, "speed");
		//BT_send_msg(&speed_control, "controlSpeed");
		//BT_send_msg(&PIDm.iState, "contIState");

		encoderPos = __HAL_TIM_GET_COUNTER(&htim2);		// állapotgépnek

		// szenzor adatok beolvasása
		ReadSensors();
		//ReadSensorsDummy();

		// szenzor adatok feldolgozása
		globalLines = getLinePos(20);

		///// stabil 3 vonal
		numLinesArray[numLinesArrayIndex] = globalLines.numLines1;
		numLinesArrayIndex++;
		if (numLinesArrayIndex >= 5)
		{
			numLinesArrayIndex = 0;
		}
		numLinesSum = 0;
		for(int i = 0; i < 5; i++)
		{
			numLinesSum += numLinesArray[i];
		}

		if (numLinesSum > 12)
		{
			stable3lines = true;
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		}else if (numLinesSum < 8){
			stable3lines = false;
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		}


		getActiveLinePos(&globalLines, &last_active_line_pos1, &last_active_line_pos2, &activeLine1, &activeLine2);
		last_active_line_pos1 = activeLine1;
		last_active_line_pos2 = activeLine2;

		angle = calculateAngle(activeLine1,activeLine2);


		linePosM = (activeLine1-15.5) * 5.9 / 1000; // pozíció, méterben, középen 0

		//////////// állapotgép   /////////////
		StateMachine(&state_struct, &stable3lines, encoderPos);

		if (state_struct.nextState == 2)
		{
			usePD = true;
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
		} else if ( (state_struct.nextState == 4) && !stable3lines) {
			usePD = false;
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
		}


		// vonalkövetés szabályozó
		if(globalLines.numLines1 != -1)	// ha látunk vonalat
		{
			if  (usePD)
			{
				pid = 1;
				error = activeLine1 - 15.5;
				control = UpdatePID1(&PIDs,error,globalLines.pos1[0]);
				if(speed > 2)
				{
					//control /= (speed/2.0);
				}
				SetServo_steering((int)control); // PID-hez
			} else
			{
				pid = 0;
				if (speed < 0.2)
				{
					speed = 0.2;
				}


				control = UpdateStateSpace(A, B, speed, linePosM, angle);
				SetServo_steering(control);
			}

			no_line_cycle_count = 0; 	// láttunk vonalat
		}
		else if (globalLines.numLines2 != -1)
		{
			no_line_cycle_count = 0;
		}
		else
		{
			no_line_cycle_count++;
			if (no_line_cycle_count > NO_LINE_CYCLES)
			{
				//SET_SPEED = 0;
				state_struct.state = -1;
				SetServo_motor(0);
				//osThreadSuspend(SteerControl_TaskHandle);
			}

		}






		//timer = __HAL_TIM_GET_COUNTER(&htim5);
		//BT_send_msg(&timer, "ctrl:" + std::string(itoa(timer,buffer,10)) + "\n");

		//////////// szabályzó hangolás  ////////////
		if(TunePID && !tune_started)
		{
			if(globalLines.pos1[0] > 20 || globalLines.pos1[0] < 12)
			{
				tune_started = true;
				tune_cntr = 0;
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
				speed_global = speed;
			}
		}
		if(tune_started)
		{
			if(tune_cntr < 100)
			{
				posArray[tune_cntr] = globalLines.pos1[0]-15.5;
				//controlArray[tune_cntr] = control*50; // így +/-15,7 a szervó tartomány (de a szabályzó adhat ki nagyobbat)
				controlArray[tune_cntr] = control/12;
			}
			tune_cntr++;

			if(tune_cntr > 100)
			{
				SET_SPEED = 0;
			}

			if(tune_cntr > 300)
			{
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
				EmergencyBreak(700);
				TunePID = false;
				tune_started = false;


				osThreadResume(SendRemoteVar_TaskHandle);
				//osThreadSuspend(SteerControl_TaskHandle);
			}
		}

		if(led_cntr == 30)
		{
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
			led_cntr = 0;
		}
		else
		{
			led_cntr++;
		}
		//osThreadSuspend(SteerControl_TaskHandle);
		osDelay(9);
	}
}

void getActiveLinePos(LineState * Lines, float *last_pos1, float *last_pos2, float * active1, float * active2)
{
	if (Lines->numLines1 == 3) {
		*active1 = Lines->pos1[1];
	}
	else if(Lines->numLines1 == 2)
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
		if( abs( (*last_pos2) - Lines->pos2[0]) < abs( (*last_pos2) - Lines->pos2[1]) )
		{
			*active2 = Lines->pos2[0];
		}
		else
		{
			*active2 = Lines->pos2[1];
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
  hadc2.Init.Resolution = ADC_RESOLUTION12b;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc2);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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













