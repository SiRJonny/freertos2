/*
 * giro.cpp
 *
 *  Created on: 2016. jan. 28.
 *      Author: Csabi
 */

#include "giro.hpp"
#include "cmsis_os.h"


extern SPI_HandleTypeDef hspi2;
extern int giro_200ms;

uint8_t data;
uint8_t temp;
uint8_t temp2[2];
uint8_t * char_ptr2;
int16_t * int16_ptr;
uint8_t ch_temp[2];


float giro_init()
{
	float girodata = 0;
	giro_write_reg(0x20, 0x0F);
	osDelay(1000);
	for(int i=0; i<100; i++)
	{
		giro_read_channel(2);
		osDelay(5);
	}

	for(int i=0; i<512; i++)
	{
		girodata += giro_read_channel(2);
		osDelay(5);
	}

	girodata /= 512;
	return girodata;
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
