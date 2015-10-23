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

using namespace std;



/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId BT_TaskHandle;
osThreadId SendRemoteVar_TaskHandle;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

osSemaphoreId xSem_USART_rdy_to_send;
osSemaphoreDef(xSem_USART_rdy_to_send);

osSemaphoreId ADC1_complete;
osSemaphoreDef(ADC1_complete);

QueueHandle_t xQueue_BT;


osMailQDef(mailbox_bt, 10, BT_MSG);
osMailQId  mailbox_bt;




/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint32_t ADC1_BUFFER[4];
uint32_t szenzorsor_1[32];
uint32_t szenzorsor_2[32];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void StartDefaultTask();
void StartButtonTask();
void SendBluetoothTask();
void SendRemoteVarTask();

void USART3_UART_Init();
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);


/*void BT_send_msg(int msg);
void BT_send_msg(int msg, char* nev);*/
void BT_send_msg(int msg, string nev);
void BT_send_msg(float msg, string nev);



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

	// itt, hogy BT tasknak már kész legyen
  xQueue_BT = xQueueCreate(10, sizeof(struct BT_MSG));

  mailbox_bt = osMailCreate(osMailQ(mailbox_bt), BT_TaskHandle);

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */


  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  osThreadDef(ButtonTask, StartButtonTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(ButtonTask), NULL);

  osThreadDef(SendBluetoothTask, SendBluetoothTask, osPriorityNormal, 0, 128);
  BT_TaskHandle = osThreadCreate(osThread(SendBluetoothTask), NULL);

  osThreadDef(SendRemoteVarTask, SendRemoteVarTask, osPriorityNormal, 0, 128);
  SendRemoteVar_TaskHandle = osThreadCreate(osThread(SendRemoteVarTask), NULL);





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

}

void BT_send_msg(int msg, char* nev){
	struct BT_MSG msg_int;
	int2msg(&msg_int, msg, nev);
	xQueueSend( xQueue_BT, &msg_int, portMAX_DELAY);

}
*/

void BT_send_msg(int msg, string nev){
	// type: 1=int, 3=float, 4=double
	xQueueSend( xQueue_BT, &number2msg(msg, string(nev).c_str(), (uint8_t)1), portMAX_DELAY);
}

void BT_send_msg(float msg, string nev){
	// type: 1=int, 3=float, 4=double
	xQueueSend( xQueue_BT, &number2msg(msg, string(nev).c_str(), (uint8_t)3), portMAX_DELAY);
}

void BT_send_msg(double msg, string nev){
	// type: 1=int, 3=float, 4=double
	xQueueSend( xQueue_BT, &number2msg(msg, string(nev).c_str(), (uint8_t)4), portMAX_DELAY);
}



/* StartDefaultTask function */
// default tastk, csak egy villogó led
void StartDefaultTask()
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {

    osDelay(500);
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);

  }
  /* USER CODE END 5 */
}

void StartButtonTask()
{
	uint8_t wasPressed = 0;
	int adc_result[4];
	char buffer[10];

	for (;;){

		// TODO: ez blokkol mindent?????
		while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)  == 1) {
			wasPressed = 1;
		}

		if (wasPressed){


			// remote változók elküldése
			//osThreadResume(SendRemoteVar_TaskHandle);

			ADC1_read();
			for(int i = 0; i<4; i++)
			{
				BT_send_msg(adc_result[i], "adc1: " + string(itoa(ADC1_BUFFER[i],buffer,10)) + "\n");
			}

			for(int i=0; i<15; i++){
				//BT_send_msg((float)((float)i + 444464.543), "xx_" + string(itoa(i,buffer,10)) + "_xx\n");
				//BT_send_msg(i+444464, "xx_" + string(itoa(i,buffer,10)) + "_xx\n");
				//BT_send_msg((double)((double)i + 4423464.54323), "xx_" + string(itoa(i,buffer,10)) + "_xx\n");
			}

			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14); // piros led, debug

			wasPressed = 0;
		}
		osDelay(30);
	}
}

// bluetooth küldő task, xQueue_BT-n keresztül kapja a struct BT_MSG pointereket, abból küldi az adatokat
// ha minden igaz csak akkor fut, ha tényleg van dolga
void SendBluetoothTask()
{
	struct BT_MSG msg;
	osEvent evt;

	for( ;; )
	{
		if(xQueue_BT != 0)
		{
			if (xQueueReceive(xQueue_BT, &msg, portMAX_DELAY))// blokk amíg nincs adat
			{

				HAL_UART_Transmit_IT(&huart3, (uint8_t*) msg.data, msg.size);


				osSignalWait(0x0001,osWaitForever);

			}

		}
	}
}

void SendRemoteVarTask()
{

	// minden remote változóhez külen kellenek ezek
	osThreadSuspend(SendRemoteVar_TaskHandle);

	for(;;)
	{



		// minden változóhoz konverzió és küldés
	BT_send_msg(123456,"teszt");


		osThreadSuspend(SendRemoteVar_TaskHandle); // minden elküldve, pihenünk (osThreadResume-ra megint elküld mindent)
	}

}


// hal uart callback
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART3)
	{
		osSignalSet(BT_TaskHandle,0x0001);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
    if(AdcHandle->Instance == ADC1)	// szenzorsor 4 jele
    {
    	osSemaphoreRelease(ADC1_complete);
    }
}



// interrupt f�ggv�nyek csak �gy mennek
extern "C"
{
	// uart2 megszakítás kezelő, meghívjuk a HAL irq kezelőjét
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
}

// ADC init (szenzorsor 4 jele)
void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION12b;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 3;
  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
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

/* SPI3 init function */
void MX_SPI3_Init(void)
{

  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
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

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4);

}

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_IC_Init(&htim2);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1);

  HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2);

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

// uart2 és hozzátartozó interruptok inicializálása
// azért main-ben, mert á kell adni neki a handle-t
void USART3_UART_Init()
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
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













