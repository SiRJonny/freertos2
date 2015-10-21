/*
 * BT_MSG.c
 *
 *  Created on: 2015. okt. 6.
 *      Author: Csabi
 */


#include "BT_MSG.h"

using namespace std;

int szuper_szamlalo = 2342345;



void int2msg(struct BT_MSG * btmsg, int ertek, char* nev)
{

	uint8_t i;
	uint8_t * ptr;

	btmsg->data[0] = 1;	// 1 = int
	btmsg->data[1] = 4;	// 32 bit

	ptr = (uint8_t*)&ertek;

	//osThreadSuspendAll();
	for( i=2 ; i<(sizeof(int)+2) ; i++)
	{
		btmsg->data[i] = *ptr;
		ptr++;
	}
	//osThreadResumeAll();

	while (*nev)
	{
		btmsg->data[i] = *nev;
		nev++;
		i++;
	}
	btmsg->data[i] = '\0';
	btmsg->datasize = sizeof(int);
	btmsg->size = i+1;
}



void float2msg(struct BT_MSG * btmsg, float ertek, char* nev)
{
	uint8_t i;
	uint8_t * ptr;

	btmsg->data[0] = 3;		// 3 = float
	btmsg->data[1] = 4;		// 32 bit

	ptr = (uint8_t *)&ertek;
	for( i=2 ; i<(sizeof(float)+2) ; i++)
	{
		btmsg->data[i] = *ptr;
		ptr++;
	}

	while (*nev)
	{
		btmsg->data[i] = *nev;
		nev++;
		i++;
	}
	btmsg->data[i] = '\0';
	btmsg->datasize = sizeof(float);
	btmsg->size = i+1;
}

void double2msg(struct BT_MSG * btmsg, double ertek, char* nev)
{
	uint8_t i;
	uint8_t * ptr;

	btmsg->data[0] = 4;		// 4 = double
	btmsg->data[1] = 8;		// 64 bit

	ptr = (uint8_t *)&ertek;
	for( i=2 ; i<(sizeof(double)+2) ; i++)
	{
		btmsg->data[i] = *ptr;
		ptr++;
	}

	while (*nev)
	{
		btmsg->data[i] = *nev;
		nev++;
		i++;
	}
	btmsg->data[i] = '\0';
	btmsg->datasize = sizeof(double);
	btmsg->size = i+1;
}

// TODO: értelmesebben, egy függvénnyel vagy valami

/*template<class T>
struct BT_MSG number2msg(T number, char* name, uint8_t type)
{
	uint8_t i;
	uint8_t * ptr;
	BT_MSG btmsg;

	btmsg.data[0] = type;	//1:int, 3:float, 4:double

	btmsg.data[1] = sizeof(T);

	ptr = (uint8_t*)&number;

	//osThreadSuspendAll();
	for( i=2 ; i<(sizeof(int)+2) ; i++)
	{
		btmsg.data[i] = *ptr;
		ptr++;
	}
	//osThreadResumeAll();

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
}*/

/*template<class T>
void number22msg(T number)
{
	int a = 6;
	//number2msg(123,"asd",(uint8_t)3);
}*/







