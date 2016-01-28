/*
 * giro.cpp
 *
 *  Created on: 2016. jan. 28.
 *      Author: Csabi
 */

#include "giro.hpp"


extern SPI_HandleTypeDef hspi2;
uint8_t data;
uint8_t temp;
uint8_t temp2[2];
uint8_t * char_ptr2;

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
