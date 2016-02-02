/*
 * config.h
 *
 *  Created on: 2016. febr. 2.
 *      Author: Gabor
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include <ProcessSensors.h>
#include <StatePattern.hpp>
#include <Controllers.h>


extern int SERVO_RANGE_MOTOR;	// max elt�r�s 0-t�l, 1500us +/- SERVO_RANGE a max kiadott jel
extern int SERVO_RANGE_STEERING;	// max elt�r�s 0-t�l, 1500us +/- SERVO_RANGE a max kiadott jel

//sebess�ges dolgok
extern float SLOW;
extern float FAST;
extern float STOP;
extern float SET_SPEED;
extern float speed_global;

//kanyarszervo
extern int servoOffset;

//encoder
extern int encoderPos;
extern float encoderIncrementToMeter;

//szenzorsor
extern uint32_t ADC1_BUFFER[4];
extern uint32_t szenzorsor_1[32];
extern uint32_t szenzorsor_2[32];

extern uint32_t szenzorsor_temp_1[32];
extern uint32_t szenzorsor_temp_2[32];

//vonal
extern float linePosM;		// vonalpoz�ci� m�terben
extern float angle;
extern float control;
extern float speed_error;
extern float speed_control;
extern float last_speed_control;
extern float last_active_line_pos1;
extern float last_active_line_pos2;

extern LineState globalLines;

extern bool stable3lines;
extern float activeLine1;  // k�z�ps� vonal kiv�laszt�sa
extern float activeLine2;

//szab�lyz�k
extern float ACC_MAX;		// egy szab�lyz� peri�dusban max ennyivel n�vekedhet a motor szervo jele
extern int NO_LINE_CYCLES;
extern float A;	// sebess�g f�gg�s	// d5% = v*A + B
extern float B;	// konstans

extern bool TunePID;
extern int pid;

extern PID_struct PIDs;
extern PID_struct PIDm;

//�llapotg�pek

extern StateContext stateContext;

//egyeb
extern char buffer[10];	//bt msg hez
extern int timer; // id�m�r�shez

extern int stopped;

#endif /* CONFIG_H_ */