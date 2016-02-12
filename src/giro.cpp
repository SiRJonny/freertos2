/*
 * giro.cpp
 *
 *  Created on: 2016. jan. 28.
 *      Author: Csabi
 */

#include "giro.hpp"
#include "cmsis_os.h"


extern SPI_HandleTypeDef hspi2;

extern float giro_drift_Y;
extern float giro_drift_Z;
extern float giro_accu_Y;
extern float giro_accu_Z;
extern bool giro_stopped;
extern bool giro_fall;

uint8_t data;
uint8_t temp;
uint8_t temp2[2];
uint8_t * char_ptr2;
int16_t * int16_ptr;
uint8_t ch_temp[2];


// balra pozitív
float giro_get_angle_Z()
{
	return giro_accu_Z/10000.0f;
}

// felfele pozitív
float giro_get_angle_Y()
{
	return giro_accu_Y/10000.0f;
}

void giro_start_measurement()
{
	giro_accu_Y = 0;
	giro_accu_Z = 0;
}

void giro_integrate()
{
	static float giro_Y;
	static float giro_Z;
	giro_Y = (float)giro_read_channel(1);
	giro_accu_Y += (float)giro_read_channel(1)-giro_drift_Y;
	giro_Z = (float)giro_read_channel(2);
	giro_accu_Z += giro_Z-giro_drift_Z;
	set_giro_stopped(giro_Z);
	set_giro_fall(giro_Y);
}

void set_giro_fall(float Y)
{
	static float array_Y[5];
	static int cntr = 0;

	array_Y[cntr] = Y;
	cntr++;
	if(cntr >= 5)
	{
		cntr = 0;
	}

	for(int i=0; i<5; i++)
	{
		if(array_Y[i] > -100)
		{
			giro_fall = false;
			return;
		}
	}
	giro_fall = true;
}


void set_giro_stopped(float Z)
{
	static float array_Z[5];
	static int cntr = 0;

	array_Z[cntr] = Z;
	cntr++;
	if(cntr >= 5)
	{
		cntr = 0;
	}

	for(int i=0; i<5; i++)
	{
		if(array_Z[i] > 100 || array_Z[i] < -100)
		{
			giro_stopped = false;
			return;
		}
	}
	giro_stopped = true;
}

void giro_init()
{
	static float girodata_Y = 0;
	static float girodata_Z = 0;

	giro_start_measurement();	// accu-k nullázása

	giro_write_reg(0x20, 0x0F);
	osDelay(500);
	for(int i=0; i<50; i++)
	{
		giro_read_channel(1);
		giro_read_channel(2);
		osDelay(5);
	}

	for(int i=0; i<512; i++)
	{
		girodata_Y += giro_read_channel(1);
		girodata_Z += giro_read_channel(2);
		osDelay(5);
	}

	girodata_Y /= 512;
	girodata_Z /= 512;

	giro_drift_Z = girodata_Z;
	giro_drift_Y = girodata_Y;
}

// csatorna beolvasás, X = 0
int giro_read_channel(int channel)
{
	uint8_t address;
	address = 0x28 + (2*channel);

	ch_temp[0] = giro_read_reg(address);
	ch_temp[1] = giro_read_reg(address + 1);

	int16_ptr = (int16_t*)&ch_temp[0];


	return (int)*int16_ptr;
}

uint8_t giro_read_reg(uint8_t address)
{
	temp = address | 0x80; 	// read bit

	giro_CS_off();
	giro_CS_on();

	HAL_SPI_Transmit(&hspi2, &temp, 1, 100);
	HAL_SPI_Receive(&hspi2, &data, 1, 100);

	giro_CS_off();

	return data;
}

void giro_write_reg(uint8_t address, uint8_t data)
{
	temp2[0] = address;
	temp2[1] = data;

	//char_ptr2 = &temp2;

	giro_CS_off();
	giro_CS_on();

	HAL_SPI_Transmit(&hspi2, &temp2[0], 2, 100);

	giro_CS_off();
}

void giro_CS_on()
{
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);
}

void giro_CS_off()
{
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_SET);
}
