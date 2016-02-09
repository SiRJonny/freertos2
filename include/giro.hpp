/*
 * giro.hpp
 *
 *  Created on: 2016. jan. 28.
 *      Author: Csabi
 */

#ifndef GIRO_HPP_
#define GIRO_HPP_

#include "stm32f4xx_hal.h"

#define ADDR_ZL 0x2C
#define ADDR_ZH 0x2D
#define GIRO_READ_BIT 0x80	//MSB = 1 = olvasás, ezt kell vagyolni a címmel


float giro_get_angle_Y();	// felfele pozitív
float giro_get_angle_Z();	// balra pozitív
void giro_start_measurement();
void giro_integrate();
void set_giro_stopped(float Z);
void set_giro_fall(float Y);

void giro_init();
int giro_read_channel(int channel);
uint8_t giro_read_reg(uint8_t address);
void giro_write_reg(uint8_t address, uint8_t data);
void giro_CS_on();
void giro_CS_off();






#endif /* GIRO_HPP_ */
