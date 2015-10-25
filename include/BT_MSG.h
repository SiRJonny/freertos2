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

extern "C" {
#include "FreeRTOS.h"
}


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

template<class T>
struct BT_MSG number2msg(T number, const char* name, uint8_t type)
{
	uint8_t i;
	uint8_t * ptr;
	BT_MSG btmsg;

	btmsg.data[0] = type;	//1:int, 3:float, 4:double

	btmsg.data[1] = sizeof(T);

	ptr = (uint8_t*)&number;


	for( i=2 ; i<(sizeof(T)+2) ; i++)
	{
		btmsg.data[i] = *ptr;
		ptr++;
	}


	while (*name)
	{
		btmsg.data[i] = *name;
		name++;
		i++;
		if(i == sizeof(btmsg.data)-1)	// utolsó hely, oda a lezáró karakter jön
		{
			break;
		}
	}
	btmsg.data[i] = '\0';
	btmsg.datasize = sizeof(T);
	btmsg.size = i+1;

	return btmsg;
}

template<class T>
void number22msg(T number)
{
	int a = 5;
}



#endif /* INCLUDE_BT_MSG_H_ */
