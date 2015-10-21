/*
 * BT_MSG.h
 *
 *  Created on: 2015. okt. 6.
 *      Author: Csabi
 *
 *  Bluetooth üzenet struktúra
 */

#ifndef INCLUDE_BT_MSG_H_
#define INCLUDE_BT_MSG_H_
#include "stm32f4xx_hal.h"


struct BT_MSG
{
	uint8_t datasize;
	uint8_t size;

	// 1. byte: data type	// 1:int, 2:uint, 3:float, 4:double
	// 2. byte: data size	// byte-ban
	// 3... : data (LSB elõször)
	// végül név byteok, /0-val lezárva
	char data[20];

};

void int2msg(struct BT_MSG * btmsg, int ertek, char* nev);
void float2msg(struct BT_MSG * btmsg, float ertek, char* nev);
void double2msg(struct BT_MSG * btmsg, double ertek, char* nev);


#endif /* INCLUDE_BT_MSG_H_ */
